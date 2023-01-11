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


//definicje pinow
#define SCL                     27    //SCL pin
#define SDA                     26    //SDA pin
#define LED                     21    //programowalny LED

//kod html wysylany na strone konfiguracyjna
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
  <html>
    <head>
      <title>Strona konfiguracyjna</title>
      <meta name="viewport" content="width=device-width, initial-scale=1">
    </head>
    <body>
      <form action="/get" target="hidden-form">
        <div>
          Ssid sieci WiFi (aktualnie %wifi_ssid%): <input type="text" name="wifi_ssid">
        </div>
        <div>
          Haslo sieci WiFi (aktualnie %wifi_password%): <input type="number " name="wifi_password">
        </div>
        <div>
          Ip brokera MQTT (aktualnie %ip_mqtt%): <input type="number " name="ip_mqtt">
        </div>
        <input type="wyślij" value="Submit">
      </form>
      <iframe style="display:none" name="hidden-form"></iframe>
    </body>
  </html>)rawliteral";

//dane dostepowe do sieci ESP
AsyncWebServer server(80);
const char* ap_ssid = "Moduł IMU";
const char* ap_password = "123456789";

//dane dostepowe do sieci WiFi
String wifi_ssid;
String wifi_password;
String ip_mqtt;

WiFiClient wifiClient;
PubSubClient MQTTclient(wifiClient);
String MQTT_topic;
char MQTT_payload[64];

SemaphoreHandle_t sema_MQTT_Parser;
SemaphoreHandle_t sema_data_rdy;

hw_timer_t * IMU_timer = NULL;

LSM9DS1 imu;

struct settings_container{//kontener dla ustawien urzadzenia
public:
  float stream_frq;       //częstotliwość zbierania i wysyłania danych [Hz] maksymalnie 80 
  int gyro_scale;         //skala żyroskopu  [245,500,2000]
  int accel_scale;        //skala akcelerometru [2,4,8,16]
  int mag_scale;          //skala magnetometru   [4,8,12,16]
  bool ON_OFF;            //wysyłanie danych ON/OFF

  settings_container(){
    stream_frq = 80;
    gyro_scale = 245;
    accel_scale = 2;
    mag_scale = 4;
    ON_OFF = false;
  }
};

settings_container settings;

String readFile(fs::FS &fs, const char * path){   //wczytaj string z pliku
  File file = fs.open(path, "r");
  if (!file || file.isDirectory()){
    Serial.println("blad zapisu do pliku");
    return String();
  }
  String fileContent;
  while (file.available()){
    fileContent+=String((char)file.read());
  }
  file.close();
  return fileContent;
}

void writeFile(fs::FS &fs, const char * path, const char * message){    //dodaj string do pliku
  File file = fs.open(path, "w");
  if(!file){
    Serial.println("blad przy otwieraniu pliku do zapisu");
    return;
  }
  if(!file.print(message)){
    Serial.println("blad zapisu do pliku");
  }
  file.close();
}

String send_current_credentials(const String& x){    //wyslij aktualnie zapamietane dane dostepowe do sieci na serwer
  if (x == "wifi_ssid"){
    return readFile(SPIFFS, "/wifi_ssid.txt");
  } else if (x == "wifi_password"){
    return readFile(SPIFFS, "/wifi_password.txt");
  } else if (x == "ip_mqtt"){
    return readFile(SPIFFS, "/ip_mqtt.txt");
  }  
  return String();
}

void config_page_setup(){   //funkcja odpowiedzialna za utrzymanie strony konfiguracyjnej
  if(!SPIFFS.begin(true)){
    Serial.println("błąd SPIFFS");
    return;
  }
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(ap_ssid,ap_password);
  IPAddress IP = WiFi.softAPIP();
  Serial.printf("IP adrest strony konfiguracyjnej: %s \n", IP.toString().c_str()); //wyswietla ip strony konfiguracyjnej
  server.begin();

  //wysylanie kodu html oraz aktualnie zapamietane dane dostepowe do sieci na zadanie serwera
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, send_current_credentials);
  });

  //odebranie danych od urzytkownika oraz zapisanie ich w pamieci nieulotnej
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

//odbiera wiadomosc wyslana przy pomocy MQTT przez uzytkownika, 
//zapisuje temat oraz tresc do zmiennych globalnych
//odblokowuje proces analizujacy odebrana wiadomosc
void MQTTcallback(char* topic, byte* payload, unsigned int length){
  if(length <=64){
    MQTT_topic = topic;
    int i = 0;
    for(i; i < length; i++){
      MQTT_payload[i] = ((char)payload[i]);
    }
    MQTT_payload[i] = NULL;
    xSemaphoreGive(sema_MQTT_Parser);
  } else {
    Serial.println("Bledne polecenie");
    MQTTclient.publish("Debug", "Bledne polecenie");
  }
}

//straznik polaczenia
//nawiazuje polaczenie z siecia WiFi oraz brokerm MQTT
//utrzymuje połaczenie oraz ponawia w przypadku zerwania
void connection_watchdog(void *parameter){
  while (1){
    //sprawdza polaczenie z siecia WiFi
    if (wifiClient.connected() && WiFi.status() == WL_CONNECTED){
      MQTTclient.loop();
    } else { //w sytuacji braku polaczenia nastepuje proba jego nawiazania w petli co 5 sekund
      settings.ON_OFF = false;
      Serial.println("Polaczenie nieudane, mozliwa potrzeba nowych danych dostepu do sieci");
      if (WiFi.status() != WL_CONNECTED){
        while (WiFi.status() != WL_CONNECTED){
          wifi_ssid = readFile(SPIFFS, "/wifi_ssid.txt");
          wifi_password = readFile(SPIFFS, "/wifi_password.txt");
          WiFi.begin(wifi_ssid.c_str(), wifi_password.c_str());
          vTaskDelay(5000 / portTICK_PERIOD_MS);
        }
        Serial.println("Polaczenie z WiFi udane");
      }
      //po zapewnieniu polaczenia z siecia WiFi nastepuje proba połaczenia z brokerem MQTT w petli co 5 sekund
      while (!MQTTclient.connected()){
        ip_mqtt = readFile(SPIFFS, "/ip_mqtt.txt");
        MQTTclient.setServer(ip_mqtt.c_str(), 1883);
        MQTTclient.connect("user");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
      }
      MQTTclient.publish("Debug", "Polaczenie z MQTT udane");
      MQTTclient.setCallback(MQTTcallback);
      MQTTclient.subscribe("Ustawienia");
    }
    vTaskDelay(250 / portTICK_PERIOD_MS);
  }
}

//Proces wybudzany w momencie otrzymania wiadomosci od uzytkownika
void parse_MQTT(void *parameter){
  char argument[14];
  char function[50];
  while (1){
    xSemaphoreTake(sema_MQTT_Parser, portMAX_DELAY); 
    if (MQTT_topic == "Ustawienia"){
      if (MQTT_payload[0] != NULL){
        //analiza wiadomosci, rozdzial na polecenie i argument
        char* iter_MQTT_payload = MQTT_payload;
        char* iter_function = function;
        char* iter_argument = argument;
        while (*iter_MQTT_payload != NULL && *iter_MQTT_payload != ' '){
          *iter_function = *iter_MQTT_payload;
          iter_function++;
          iter_MQTT_payload++;
        }
        iter_MQTT_payload++;
        while (*iter_MQTT_payload != NULL && *iter_MQTT_payload != ' '){
          *iter_argument = *iter_MQTT_payload;
          iter_MQTT_payload++;
          iter_argument++;
        }
        //w zaleznosci od tresci wiadomosci podejmowane są dzialania
        //polecenie zmieniające częstotliwosc zbierania i wysylania danych
        if (String(function) == "czestotliwosc"){
          if (atof(argument) > 0 && atof(argument) <= 80){
            MQTTclient.publish("Debug", "Zmieniono czestotliwosc");
            settings.stream_frq = atof(argument);
            //zmiana paremetow przerwania czasowego ktore realizuje wysylanie danych
            timerAlarmWrite(IMU_timer, (1 / settings.stream_frq) * 1000000, true);
          } else {
            MQTTclient.publish("Debug", "Maksymalna czetotliwosc to 80Hz");
          }
        //polecenie zmieniajace skale zyroskopu
        } else if (String(function) == "Skala_zyroskopu"){
          if (atoi(argument) == 245 || atoi(argument) == 500 || atoi(argument) == 2000){
            MQTTclient.publish("Debug", "Zmieniono skalę zyroskopu");
            settings.gyro_scale = atoi(argument);
            //zmiana ustawien IMU
            imu.settings.gyro.scale = settings.gyro_scale;
          } else {
            MQTTclient.publish("Debug", "Dostepne skalowanie dla zyroskopu : 245,500,2000");
          }
        //polecenie zmieniajace skale akcelerometru
        } else if (String(function) == "Skala_akcelerometru"){
          if (atoi(argument) == 2 || atoi(argument) == 4 || atoi(argument) == 8 || atoi(argument) == 16){
            MQTTclient.publish("Debug", "Zmieniono skale akcelerometru");
            settings.accel_scale = atoi(argument);
            //zmiana ustawien IMU
            imu.settings.accel.scale = settings.accel_scale;
          } else {
            MQTTclient.publish("Debug", "Dostwpne skalowanie dla akcelerometru : 2,4,8,16");
          }
        //polecenie zmieniające skalw magnetometru
        } else if (String(function) == "Skala_magnetometru"){
          if (atoi(argument) == 4 || atoi(argument) == 8 || atoi(argument) == 12 || atoi(argument) == 16){
            MQTTclient.publish("Debug", "Zmieniono skale magnetometru ");
            settings.mag_scale = atoi(argument);
            //zmiana ustawien IMU
            imu.settings.mag.scale = settings.mag_scale;
          } else {
            MQTTclient.publish("Debug", "Dostepne skalowanie magnetometru : 4,8,12,16");
          }
        //polecenie aktywujace strumieniowaie danych
        } else if (String(function) == "ON/OFF"){
          if(String(argument) == "ON"){
            settings.ON_OFF = true;
            MQTTclient.publish("Debug", "IMU ON");
          } else if (String(argument) == "OFF"){
            settings.ON_OFF = false;
            MQTTclient.publish("Debug", "IMU OFF");
          }
        //polecenie wyswietlajace aktualne ustawienia
        } else if (String(function) == "Ustawienia"){
          MQTTclient.publish("Debug", String(settings.stream_frq).c_str());
          MQTTclient.publish("Debug", String(settings.gyro_scale).c_str());
          MQTTclient.publish("Debug", String(settings.accel_scale).c_str());
          MQTTclient.publish("Debug", String(settings.mag_scale).c_str());
          MQTTclient.publish("Debug", String(settings.ON_OFF).c_str());
        //polecenie rozpoczynajace proces kalibracji IMU
        } else if (String(function) == "Kalibruj"){
          imu.calibrate();
          imu.calibrateMag();
          MQTTclient.publish("Debug", "Kalibracja zakonczona");
        } else if (String(function) == "pasmo_zyroskopu"){
          if (atoi(argument) == 0 || atoi(argument) == 1 || atoi(argument) == 2 || atoi(argument) == 3){
            MQTTclient.publish("Debug", "Zmieniono pasmo zyroskopu");
            imu.settings.gyro.bandwidth = atoi(argument);
          } else {
            MQTTclient.publish("Debug","Dostepne opcje to : 0, 1, 2, 3");
          }
        } else if (String(function) == "odciecie_zyroskopu"){
          if (atoi(argument) == 0 || atoi(argument) == 1 || atoi(argument) == 2 || atoi(argument) == 3 ||
              atoi(argument) == 4 || atoi(argument) == 5 || atoi(argument) == 6 || atoi(argument) == 7 ||
              atoi(argument) == 8 || atoi(argument) == 9){
            MQTTclient.publish("Debug", "Zmieniono odciecie zyroskopu");
            imu.settings.gyro.HPFCutoff = atoi(argument);
          } else {
            MQTTclient.publish("Debug", "Dostepne opcje to : 0, 1, 2, 3, 4, 5, 6, 7, 8, 9");
          }
        } else if (String(function) == "pasmo_akcelerometru"){
          if (atoi(argument) == 0 || atoi(argument) == 1 || atoi(argument) == 2 || atoi(argument) == 3){
            MQTTclient.publish("Debug", "Zmieniono pasmo akcelerometru");
            imu.settings.accel.highResBandwidth = atoi(argument);
          } else {
            MQTTclient.publish("Debug","Dostepne opcje to : 0, 1, 2, 3");
          }
        } else {
          MQTTclient.publish("Debug","Błedne polecenie");
        }
        memset(function, NULL, 50);
        memset(argument, NULL, 14);
      }
    }
    memset(MQTT_payload, NULL, 64);
    MQTT_topic = "";
  }
}

//przerwanie czasowe odblokowujace proces wysylajacy dane
void IRAM_ATTR IMU_timer_callback(){
  if (settings.ON_OFF == true){
    xSemaphoreGive(sema_data_rdy);
  }
}

//proces wysylajacy dane
void send_data(void *parameter){
  while (1){
    xSemaphoreTake(sema_data_rdy, portMAX_DELAY);
    imu.readGyro();
    imu.readAccel();
    imu.readMag();
    String msg = String(imu.calcAccel(imu.ax)) + " " 
               + String(imu.calcAccel(imu.ay)) + " " 
               + String(imu.calcAccel(imu.az)) + " " 
               + String(imu.calcGyro(imu.gx)) + " " 
               + String(imu.calcGyro(imu.gy)) + " " 
               + String(imu.calcGyro(imu.gz)) + " " 
               + String(imu.calcMag(imu.mx)) + " " 
               + String(imu.calcMag(imu.my)) + " " 
               + String(imu.calcMag(imu.mz));
    MQTTclient.publish("Dane", msg.c_str());
  }
}

//funkcja setup, wykonuje się jednorazowo po rozpoczeciu pracy urzadzenia
void setup() {
  Serial.begin(115200);
  //aktywacja punktu dostepu i strony konfiguracyjnej
  config_page_setup();
  Wire.begin(SDA,SCL);
  //inicjalizacja IMU
  if (imu.begin(0x6A, 0x1C) == false){
    Serial.println("Failed to communicate with LSM9DS1.");
    while(1);
  } else {
    Serial.println("imu good");
    imu.settings.gyro.sampleRate = 4;
    imu.settings.accel.sampleRate = 4;
    imu.settings.mag.sampleRate = 7;
    imu.settings.accel.highResEnable = true;
    imu.settings.gyro.HPFEnable = true;
  }
  sema_MQTT_Parser = xSemaphoreCreateBinary();
  sema_data_rdy = xSemaphoreCreateBinary();
  //inicjalizacja przerwania czasowego
  IMU_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(IMU_timer, &IMU_timer_callback, true);
  timerAlarmWrite(IMU_timer, (1 / settings.stream_frq) * 1000000, true);
  timerAlarmEnable(IMU_timer);
  //inicjalizacja procesu straznika polaczenia
  xTaskCreatePinnedToCore(connection_watchdog, "connection_watchdog", 2500, NULL, 1, NULL, app_cpu);
  //inicjalizacja procesu analizujacego przychodzace wiadomosci
  xTaskCreatePinnedToCore(parse_MQTT, "parse_MQTT", 2500, NULL, 1, NULL, app_cpu);
  //inicjalizacja procesu wysylającego dane
  xTaskCreatePinnedToCore(send_data, "send_data", 2500, NULL, 1, NULL, app_cpu);

  vTaskDelete(NULL);
}

void loop() {}