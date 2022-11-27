#include "Arduino.h"
#include "WiFi.h"
#include <Wire.h>
#include <string>
#include <vector>
#include <ctime>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <PubSubClient.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

using namespace std;

#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif


//Pin definition
#define SCL                     27    //SCL pin for I2C
#define SDA                     26    //SDA pin for I2C
#define Battery_measurement     15    //Pin used to enable battery state measurement
#define LED                     21    //Programmable LED
#define M_DATA_RDY              25    //Magnetometer data ready
#define INT1                    32    //IMU interrupt 1
#define M_INT                   33    //Magnetometer interrupt
#define AG_DATA_EN              34    //IMU data enable
#define INT2                    32    //IMU interrupt 2

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
  <html>
    <head>
      <title>ESP Input Form</title>a
      <meta name="viewport" content="width=device-width, initial-scale=1">
    </head>
    <body>
      <form action="/get" target="hidden-form">
        <div>
          wifi_ssid (current value %wifi_ssid%): <input type="text" name="wifi_ssid">
        </div>
        <div>
          wifi_password (current value %wifi_password%): <input type="number " name="wifi_password">
        </div>
        <div>
          ip_mqtt (current value %ip_mqtt%): <input type="number " name="ip_mqtt">
        </div>
        <div>
          user_id (current value %user_id%): <input type="number " name="user_id">
        </div>
        <input type="submit" value="Submit">
      </form>
      <iframe style="display:none" name="hidden-form"></iframe>
    </body>
  </html>)rawliteral";

//Esp network credentials
AsyncWebServer server(80);
const char* ap_ssid = "band";
const char* ap_password = "123456789";

//Wifi credentials
String wifi_ssid;
String wifi_password;
String ip_mqtt;
String user_id;

WiFiClient wifiClient;
PubSubClient MQTTclient(wifiClient);
String MQTT_topic;
char MQTT_payload[64];

SemaphoreHandle_t sema_MQTT_Parser;
SemaphoreHandle_t sema_data_rdy;

hw_timer_t * IMU_timer = NULL;

LSM9DS1 imu;

struct IMU_data{  //single dataframe from IMU
public:
    long int id;
    float a[3];
    float g[3];
    float m[3];

    IMU_data(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz){
      a[0] = ax;
      a[1] = ay;
      a[2] = az;
      g[0] = ax;
      g[1] = ay;
      g[2] = az;
      m[0] = ax;
      m[1] = ay;
      m[2] = az;
    }
};

struct settings_container{//container for settings
public:
  float sample_frq;       //data sample frequency [Hz] max 80
  float stream_frq;       //send data with given frq [Hz]
  int gyro_scale;         //gyro scale  [245,500,2000]
  int accel_scale;        //accel scale [2,4,8,16]
  int mag_scale;          //mag_scale   [4,8,12,16]
  bool ON_OFF;            //stream ON OFF

  settings_container(){
    sample_frq = 10;
    stream_frq = 0.1;
    gyro_scale = 245;
    accel_scale = 2;
    mag_scale = 4;
    ON_OFF = false;
  }
};

settings_container settings;

String readFile(fs::FS &fs, const char * path){   //read string from file
  Serial.printf("Reading file: %s\r\n", path);
  File file = fs.open(path, "r");
  if(!file || file.isDirectory()){
    Serial.println("- empty file or failed to open file");
    return String();
  }
  Serial.println("- read from file:");
  String fileContent;
  while(file.available()){
    fileContent+=String((char)file.read());
  }
  file.close();
  Serial.println(fileContent);
  return fileContent;
}

void writeFile(fs::FS &fs, const char * path, const char * message){    //append string to file
  Serial.printf("Writing file: %s\r\n", path);
  File file = fs.open(path, "w");
  if(!file){
    Serial.println("- failed to open file for writing");
    return;
  }
  if(file.print(message)){
    Serial.println("- file written");
  } else {
    Serial.println("- write failed");
  }
  file.close();
}

String send_current_credentials(const String& var){    //send saved in SPIFFS wifi and mqtt credentials to server
  if(var == "wifi_ssid"){
    return readFile(SPIFFS, "/wifi_ssid.txt");
  }
  else if(var == "wifi_password"){
    return readFile(SPIFFS, "/wifi_password.txt");
  }
  else if(var == "ip_mqtt"){
    return readFile(SPIFFS, "/ip_mqtt.txt");
  }  
  else if(var == "user_id"){
    return readFile(SPIFFS, "/user_id.txt");
  }
  return String();
}

void config_page_setup(){   //sets up config page
  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  WiFi.mode(WIFI_AP_STA);

  //AP mode
  WiFi.softAP(ap_ssid,ap_password);
  IPAddress IP = WiFi.softAPIP();
  Serial.printf("IP address: %s \n", IP.toString().c_str());
  server.begin();

  //sending html to server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, send_current_credentials);
  });

  //get entered credentials and save them to SPIFFS
  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    if (request->hasParam("wifi_ssid")) {
      inputMessage = request->getParam("wifi_ssid")->value();
      writeFile(SPIFFS, "/wifi_ssid.txt", inputMessage.c_str());
    }
    if (request->hasParam("wifi_password")) {
      inputMessage = request->getParam("wifi_password")->value();
      writeFile(SPIFFS, "/wifi_password.txt", inputMessage.c_str());
    }
    if (request->hasParam("ip_mqtt")) {
      inputMessage = request->getParam("ip_mqtt")->value();
      writeFile(SPIFFS, "/ip_mqtt.txt", inputMessage.c_str());
    }
    if (request->hasParam("user_id")) {
      inputMessage = request->getParam("user_id")->value();
      writeFile(SPIFFS, "/user_id.txt", inputMessage.c_str());
    } else {
      inputMessage = "No message sent";
    }
    Serial.println(inputMessage);
    request->send(200, "text/text", inputMessage);
    });
  server.onNotFound([](AsyncWebServerRequest *request){request->send(404, "text/plain", "Not found");});
  server.begin();
}

void MQTTcallback(char* topic, byte* payload, unsigned int length){   //recives setting commands
  if(length <=64){
    MQTT_topic = topic;
    int i = 0;
    for(i; i < length; i++){
      MQTT_payload[i] = ((char)payload[i]);
    }
    MQTT_payload[i] = NULL;
    xSemaphoreGive(sema_MQTT_Parser);
  } else {
    Serial.println("Wrong commend");
    MQTTclient.publish("Debug","Wrong commend");
  }
}

void connection_watchdog(void *parameter){    //connection watchdog
  while(1){
    if(wifiClient.connected() && WiFi.status() == WL_CONNECTED){
      MQTTclient.loop();
    } else {
      settings.ON_OFF = false;
      Serial.println("Connection failed, new credentials may be needed");
      if(WiFi.status() != WL_CONNECTED){
        while(WiFi.status() != WL_CONNECTED){
          wifi_ssid = readFile(SPIFFS, "/wifi_ssid.txt");
          wifi_password = readFile(SPIFFS, "/wifi_password.txt");
          WiFi.begin(wifi_ssid.c_str(), wifi_password.c_str());
          vTaskDelay(5000 / portTICK_PERIOD_MS);
        }
        Serial.println("Connected to WiFi");
      }
      while ( !MQTTclient.connected() ){
        ip_mqtt = readFile(SPIFFS, "/ip_mqtt.txt");
        user_id = readFile(SPIFFS, "/user_id.txt");
        MQTTclient.setServer(ip_mqtt.c_str(), 1883);
        MQTTclient.connect(user_id.c_str());
        vTaskDelay(5000 / portTICK_PERIOD_MS);
      }
      Serial.println("Connected to MQTT");
      MQTTclient.publish("Debug","Connected to MQTT");
      MQTTclient.setCallback(MQTTcallback);
      MQTTclient.subscribe("settings");
    }
    vTaskDelay(250 / portTICK_PERIOD_MS);
  }
}

void parse_MQTT(void *parameter){    //parse MQTT messeage and actualize settings
  char argument[14];
  char function[50];
  while(1){
    xSemaphoreTake(sema_MQTT_Parser, portMAX_DELAY); 
    if(MQTT_topic == "settings"){
      if(MQTT_payload[0] != NULL){
        char* iter_MQTT_payload = MQTT_payload;
        char* iter_function = function;
        char* iter_argument = argument;
        while(*iter_MQTT_payload != NULL && *iter_MQTT_payload != ' '){
          *iter_function = *iter_MQTT_payload;
          iter_function++;
          iter_MQTT_payload++;
        }
        iter_MQTT_payload++;
        while(*iter_MQTT_payload != NULL && *iter_MQTT_payload != ' '){
          *iter_argument = *iter_MQTT_payload;
          iter_MQTT_payload++;
          iter_argument++;
        }
        Serial.println(String(function));
        if(String(function) == "sample_frq"){
          Serial.println(atof(argument));
          if(atof(argument) > 0 && atof(argument) <= 80){
            Serial.print("changed sample frq to ");
            Serial.print(String(argument));
            Serial.println(" Hz");
            MQTTclient.publish("Debug","changed sample frq");
            settings.sample_frq = atof(argument);
            timerAlarmWrite(IMU_timer, (1 / settings.sample_frq) * 1000000, true);
          } else {
            Serial.println("Max sample frq is 80 Hz");
            MQTTclient.publish("Debug","Max sample frq is 80 Hz");
          }
        } else if(String(function) == "stream_frq"){
          Serial.println(atof(argument));
          Serial.print(String(argument));
          Serial.println(" Hz");
          MQTTclient.publish("Debug","changed stream frq");
          settings.stream_frq = atof(argument);
        } else if(String(function) == "gyro_scale"){
          Serial.println(atoi(argument));
          if(atoi(argument) == 245 || atoi(argument) == 500 || atoi(argument) == 2000){
            Serial.print("changed gyro scale to ");
            Serial.print(String(argument));
            Serial.println(" dps");
            MQTTclient.publish("Debug","changed gyro scale");
            settings.gyro_scale = atoi(argument);
            imu.settings.gyro.scale = settings.gyro_scale;
          } else {
            Serial.println("Avaiable gyro scale : 245,500,2000");
            MQTTclient.publish("Debug","Avaiable gyro scale : 245,500,2000");
          }
        } else if(String(function) == "accel_scale"){
          Serial.println(atoi(argument));
          if(atoi(argument) == 2 || atoi(argument) == 4 || atoi(argument) == 8 || atoi(argument) == 16){
            Serial.print("changed accel scale to ");
            MQTTclient.publish("Debug","changed accel scale");
            Serial.print(String(argument));
            Serial.println(" g");
            settings.accel_scale = atoi(argument);
            imu.settings.accel.scale = settings.accel_scale;
          } else {
            Serial.println("Avaiable gyro scale : 2,4,8,16");
            MQTTclient.publish("Debug","Avaiable gyro scale : 2,4,8,16");
          }
        } else if(String(function) == "mag_scale"){
          Serial.println(atoi(argument));
          if(atoi(argument) == 4 || atoi(argument) == 8 || atoi(argument) == 12 || atoi(argument) == 16){
            Serial.print("changed accel scale to ");
            MQTTclient.publish("Debug","changed accel scale ");
            Serial.print(String(argument));
            Serial.println(" Gs");
            settings.mag_scale = atoi(argument);
            imu.settings.mag.scale = settings.mag_scale;
          } else {
            Serial.println("Avaiable gyro scale : 4,8,12,16");
            MQTTclient.publish("Debug","Avaiable gyro scale : 4,8,12,16");
          }
        } else if(String(function) == "ON_OFF"){
          if(String(argument) == "ON"){
            Serial.println(String(argument));
            settings.ON_OFF = true;
            MQTTclient.publish("Debug","IMU ON");
          } else if(String(argument) == "OFF"){
            Serial.println(String(argument));
            settings.ON_OFF = false;
            MQTTclient.publish("Debug","IMU OFF");
          }
        } else if(String(function) == "current_settings"){
          Serial.println(settings.sample_frq);
          Serial.println(settings.stream_frq);
          Serial.println(settings.gyro_scale);
          Serial.println(settings.accel_scale);
          Serial.println(settings.mag_scale);
          Serial.println(settings.ON_OFF);
          MQTTclient.publish("Debug",String(settings.sample_frq).c_str());
          MQTTclient.publish("Debug",String(settings.stream_frq).c_str());
          MQTTclient.publish("Debug",String(settings.gyro_scale).c_str());
          MQTTclient.publish("Debug",String(settings.accel_scale).c_str());
          MQTTclient.publish("Debug",String(settings.mag_scale).c_str());
          MQTTclient.publish("Debug",String(settings.ON_OFF).c_str());
        } else {
          Serial.println("Wrong command");
          MQTTclient.publish("Debug","Wrong command");
        }
        memset(function, NULL, 50);
        memset(argument, NULL, 14);
      }
    }
    memset(MQTT_payload, NULL, 64);
    MQTT_topic = "";
  }
}

void IRAM_ATTR IMU_timer_callback(){
  if(settings.ON_OFF == true){
    xSemaphoreGive(sema_data_rdy);
  }
}

void send_data(void *parameter){
  while(1){
    xSemaphoreTake(sema_data_rdy, portMAX_DELAY);
    //if(imu.gyroAvailable() && imu.accelAvailable() && imu.magAvailable()){
      imu.readGyro();
      imu.readAccel();
      imu.readMag();
      String msg = "a : " + String(imu.calcAccel(imu.ax)) + " " + String(imu.calcAccel(imu.ay)) + " " + String(imu.calcAccel(imu.az)) + " g : " + String(imu.calcGyro(imu.gx)) + " " + String(imu.calcGyro(imu.gy)) + " " + String(imu.calcGyro(imu.gz)) + " m : " + String(imu.calcMag(imu.mx)) + " " + String(imu.calcMag(imu.my)) + " " + String(imu.calcMag(imu.mz));
      MQTTclient.publish("Data", msg.c_str());
    //} else {
      //Serial.println("data not rdy");
    //}
  }
}

void setup() {
  Serial.begin(115200);
  config_page_setup();
  Wire.begin(SDA,SCL);

  if (imu.begin(0x6A, 0x1C) == false){
    Serial.println("Failed to communicate with LSM9DS1.");
    while(1);
  } else {
    Serial.println("imu good");
    imu.settings.gyro.sampleRate = 4;
    imu.settings.accel.sampleRate = 4;
    imu.settings.mag.sampleRate = 7;
  }
  sema_MQTT_Parser = xSemaphoreCreateBinary();
  sema_data_rdy = xSemaphoreCreateBinary();
  IMU_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(IMU_timer, &IMU_timer_callback, true);
  timerAlarmWrite(IMU_timer, (1 / settings.sample_frq) * 1000000, true);
  timerAlarmEnable(IMU_timer);
  xTaskCreatePinnedToCore(connection_watchdog, "connection_watchdog", 2500, NULL, 1, NULL, app_cpu);
  xTaskCreatePinnedToCore(parse_MQTT, "parse_MQTT", 2500, NULL, 1, NULL, app_cpu);
  xTaskCreatePinnedToCore(send_data, "send_data", 2500, NULL, 1, NULL, app_cpu);

  vTaskDelete(NULL);
}

void loop() {}