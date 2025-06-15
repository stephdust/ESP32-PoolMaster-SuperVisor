/*
  PoolMaster SuperVisor - Act as a watchdog, OOBE, upload manager and supervise PoolMaster
  WifiManager : https://github.com/tzapu/WiFiManager WiFi Configuration Magic
  I2C Slave and Master :  https://deepbluembedded.com/arduino-i2c-slave/
                          https://randomnerdtutorials.com/esp32-i2c-master-slave-arduino/
                            // logo convertor https://elmah.io/tools/base64-image-encoder/
  HTML, DOM, CSS :
    https://mollify.noroff.dev/content/feu1/javascript-1/module-4/update-html?nav=
    https://www.w3schools.com/howto/howto_js_progressbar.asp
    https://www.youtube.com/watch?v=rtx27XZadIw

  Logs from Serial :
    https://github.com/WolfgangFranke/ESP32_remote_longterm_logging/blob/master/Ardunio-Code/ESP32_WebServer_Highcharts_WebLogger/ESP32_WebServer_Highcharts_WebLogger.ino
    https://stackoverflow.com/questions/61755373/create-html-table-using-json-data-in-javascript

  OTA : https://github.com/espressif/arduino-esp32/blob/master/libraries/HTTPUpdateServer/src/HTTPUpdateServer.h
*/

// TODO :
//  Propose to fully format ESP32-PoolMaster (copy bootloader+partition+firmware.bin)
//  replace elegantOTA (remove commercial stuffs + too big)
//  future : send info to external OLED on I2c Master
//  add PaperTrail but consumes 8% of the disk
//  https + authentication

#define PMSV_VERSION  "0.90"
#define TARGET_TELNET
//#define TARGET_WEBSERIAL
// PAPERTRAIL consumes 8% of the flashdisk !
//#define TARGET_PAPERTRAIL

#include <WiFiManager.h>          
#include <WiFi.h>
#include <WiFiMulti.h>

#ifdef TARGET_WEBSERIAL
#include <WebSerial.h>
#endif

#include <TimeLib.h>
#include <ElegantOTA.h>

#ifdef TARGET_PAPERTRAIL
#include <Elog.h>
#include <ElogMacros.h>
#endif

#include <ESPNexUpload.h>
#include <HTTPClient.h>
#include "soc/rtc_wdt.h"
#include "esp32_flasher.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "driver/i2c.h"
#include <ESPmDNS.h>
#include <Preferences.h>
#include <uptime.h>
#include <AsyncMqttClient.h>
#include <ArduinoJson.h>

const long gmtOffset_sec = 3600;
const int daylightOffset_sec = 3600;

#ifdef TARGET_PAPERTRAIL
// PaperTrail
#define PL_LOG 0  // PoolMaster Logs
#define WD_LOG 1  // WatchDog Logs
#endif

#ifdef TARGET_TELNET
  //how many clients should be able to telnet to this ESP32
  #define MAX_SRV_CLIENTS 3
#endif

#define BUFFER_SIZE 1024
#define LOG_BUFFER_SIZE 1024

// Update triggers
volatile bool mustUpdateNextion       = false;
volatile bool mustUpdatePoolMaster    = false;
volatile bool mustRebootPoolMaster    = false;

// Update binary storage (HTTP Server)
const char* defaultUpdatehost         = "myUpdateHttpServer:myport";
const char* defaultNextionPath        = "/poolmaster/Nextion.tft";
const char* defaultPoolmasterPath     = "/poolmaster/PoolMaster.bin";
//const char* defaultUpdateurlwatchdog = "/poolmaster/WatchDog.bin"; // not used today

#ifdef TARGET_PAPERTRAIL
// PaperTrail log management
const char* defaultpapertrailhost      = "mypapertrailserver";
const char* defaultpapertrailport     = "21858";
#endif
// MQTT Server
const char* defaultmqtt_server        = "mymqttserver";
const char* defaultmqtt_port          = "1883";
const char* defaultmqtt_topic         = "PoolMaster";
const char* defaultmqtt_username      = "";
const char* defaultmqtt_password      = "";

AsyncMqttClient MqttClient;
TimerHandle_t MqttReconnectTimer;
#define PAYLOAD_BUFFER_LENGTH 1024

#define _LHOSTNAME_   16
#define _LSSID_       32
#define _LURL_        128
#define _LVERSION_    12
#define _LTOPIC_      128
#define _LMAC_        20

char  myhostname[_LHOSTNAME_] = {0};
char  hostname[_LHOSTNAME_]   = {0};
char  currentUptime[25];

JsonDocument PMInfo;
JsonDocument SVSettings;

// Nextion/Poolmaster Update counter for feedback
int UpdateCounter = 0;
float percentCounter=0;
int contentLength = 0;
char barBuf[64] = {0};

// Nextion PIN Numbers
#define NEXT_RX           33 // Nextion RX pin
#define NEXT_TX           32 // Nextion TX pin

// PoolMaster PIN Numbers
#define ENABLE_PIN        25
#define BOOT_PIN          26
// Enable and Boot pin numbers
const int ENPin = ENABLE_PIN;
const int BOOTPin = BOOT_PIN;

// this ESP32 is a I2C Slave for PoolMaster
// PoolMaster can get info from SuperVisor and vice-versa
#define SDA_S             SDA
#define SCL_S             SCL
#define I2C_SLAVE_ADDR    0x07
#define I2C_MAXMESSAGE    64
const char _DELIMITER_[] = {0xEA,0xEA,0xEA}; // ΩΩΩ

// IC2 Master for future OLED display
#define SDA_M             14
#define SCL_M             12

// Wifi Manager
# define RESET_WIFI_PIN     19  // 27 or 19.  GPIO=LOW -> start WifiManager, reset settings when held 3sec
WiFiManager wifiManager;
#define _DEFAULT_NAME_       "PoolMaster"
#define SuperVisor_Suffix     "_SV"
Preferences preferences;
bool shouldSaveConfig = false;

#ifdef TARGET_TELNET
  WiFiServer Telnetserver(23);
  WiFiClient serverClients[MAX_SRV_CLIENTS];
#endif
AsyncWebServer Webserver(80);
static bool OTAinProgress=0;

// Local logline buffers
char sbuf[BUFFER_SIZE];
char local_sbuf[LOG_BUFFER_SIZE];

#define LogLines_size   10  // max number of log-lines in log ring buffer
#define LogLines_maxLen 100 // max lenght of each Log-Line text
class LogsRingBuffer
{
  private:
    int readindex = -1;
    int writeindex = -1;
    char LogLines_array[LogLines_size][LogLines_maxLen] = {0};  
    
  public:
    void push(const char* prefix, char* line) {
      int src, dst;
      for (src = 0, dst = 0; src < strlen(line); src++)
        if ((line[src] != '\r') && (line[src] != '\n')) 
          line[dst++] = line[src];
      line[dst] = 0;
      if (line[0] == 0) return;
      writeindex++;
      writeindex %= LogLines_size;
      snprintf(LogLines_array[writeindex], LogLines_maxLen-1, "%s%s", prefix, line);
      LogLines_array[(writeindex+1)%LogLines_size][0] = 0 ; // tag next circular line, obsolete
    }
    void push(const char* prefix, const char* line) {
      strcpy(sbuf, line);
      push(prefix, sbuf);
    }
    char* pull() {
      readindex++;
      readindex %= LogLines_size;
      char* l = LogLines_array[readindex];
      if (*l == 0) { // obsolete data
        readindex--;
        return 0;
      }
      return l;
   }
};
LogsRingBuffer myLogsRingBuffer;

// OTA
unsigned long ota_progress_millis = 0;

/*! Send Logs to the various facilities
 *
 * \param _log_message The message to be sent.
 * \param _targets The targets where message should be printed (1-Telnet 2-WebSerial 3-PaperTrail or any combination)
 * \param _telnet_separator The newline character to be used when printing with Telnet
 *
 * \return None
 */
void Local_Logs_Dispatch(const char *_log_message, uint8_t _targets = 7, const char* _telnet_separator = "\r\n")
{

#ifdef TARGET_TELNET
  if(_targets & 1) {  // First bit is for telnet
    // Telnet
    for (int i = 0; i < MAX_SRV_CLIENTS; i++) {
      if (serverClients[i] && serverClients[i].connected()) {
        serverClients[i].write(_log_message, strlen(_log_message));
        delay(1);
        serverClients[i].write(_telnet_separator, strlen(_telnet_separator));
        delay(1);
      }
    }
  }
#endif

#ifdef TARGET_WEBSERIAL
  if(_targets & 2) {  // Second bit is for WebSerial
    // WebSerial
    WebSerial.printf("%s",_log_message);
  }
#endif

#ifdef TARGET_PAPERTRAIL
  if ((papertrailhost != "") && (papertrailhost != defaultpapertrailhost) && (_targets & 4)) {  // Third bit is for ParerTrail
    // Cloud PaperTrail
    logger.log(WD_LOG, 1, "%s", _log_message);
  }
#endif

  myLogsRingBuffer.push("SV ", _log_message);
}

// Monitor free stack (display smallest value)
void stack_mon(UBaseType_t &hwm)
{
  UBaseType_t temp = uxTaskGetStackHighWaterMark(nullptr);
  if(!hwm || temp < hwm)
  {
    hwm = temp;
    Serial.printf("[stack_mon] %s: %d bytes\n",pcTaskGetTaskName(NULL), hwm);
  }  
}

/*! Thread safe, memory safe non blocking read until delimiter is found.
 *
 * \param stream Stream.
 * \param buf Receiving buffer.
 * \param delim Delimeter.
 * \param i Buffer index.
 * \param j Delimiter index.
 *
 * \return true when delimiter is found or the buffer is full, false otherwise.
 */
template <size_t n> bool readUntil_r(
  Stream& stream, char (&buf)[n], char const* delim, size_t& i, size_t& j) {
for (; i < n and delim[j]; i++) {
  if (not stream.available()) {
    return false;
  }
  buf[i] = stream.read();

  if (buf[i] == delim[j]) {
    j++;
  }
  else {
    j = 0;
  }
}
for (; i < n; i++) {
  buf[i] = 0;
}
i = 0;
j = 0;
return true;
}

/*! Memory safe non blocking read until delimiter is found.
*
* \param stream Stream.
* \param buf Receiving buffer.
* \param delim Delimeter.
*
* \return true when delimiter is found or the buffer is full, false otherwise.
*/
template <size_t n> bool readUntil(Stream& stream, char (&buf)[n], char const* delim) {
  static size_t i = 0;
  static size_t j = 0;
  return readUntil_r(stream, buf, delim, i, j);
}


///////////////// Update POOLMASTER ///////////////////
////////////////////////////////////////////////////////
//void TaskUpdatePoolMaster(void *pvParameters=nullptr)
void TaskUpdatePoolMaster(void)
{
  //static UBaseType_t hwm=0;     // free stack size
  //rtc_wdt_protect_off();
  //rtc_wdt_disable();
  HTTPClient http;
  char url[_LURL_];
  const char* host=SVSettings["Update Host"];
  const char* path=SVSettings["Poolmaster Path"];
  sprintf(url, "http://%s%s",host,path);
  snprintf(local_sbuf,sizeof(local_sbuf),"Requesting URL: %s",url);
  Local_Logs_Dispatch(local_sbuf);
  // begin http client
  if(!http.begin(url)){
      Local_Logs_Dispatch("Connection failed");
    return;
  }

  // This will send the (get) request to the server
  int code          = http.GET();
  contentLength     = http.getSize();
        
  // Update the nextion display
  if(code == 200){
    OTAinProgress=1;
    Local_Logs_Dispatch("File received. Update PoolMaster...");
    bool result;
    snprintf(local_sbuf,sizeof(local_sbuf),"Start upload. File size is: %d bytes",contentLength);
    strcpy(barBuf, local_sbuf);
    Local_Logs_Dispatch(local_sbuf);
    // Initialize ESP32Flasher
    ESP32Flasher espflasher;
    // set callback: What to do / show during upload..... Optional! Called every transfert integer %
    UpdateCounter=0;
    espflasher.setUpdateProgressCallback([](){
      UpdateCounter++;
      percentCounter = (float(UpdateCounter*1024)/contentLength)*100.0;
      snprintf(local_sbuf,sizeof(local_sbuf),"PoolMaster update %4.1f%%",percentCounter);
      strcpy(barBuf, local_sbuf);
      Local_Logs_Dispatch(local_sbuf,1,"\r");
    });
    espflasher.espFlasherInit();//sets up Serial communication to another esp32
    int connect_status = espflasher.espConnect();
    if (connect_status != SUCCESS) 
      Local_Logs_Dispatch("Cannot connect to target");
    else {
      Local_Logs_Dispatch("Connected to target");
      espflasher.espFlashBinStream(*http.getStreamPtr(),contentLength);
    }
  }
  else {
    snprintf(local_sbuf,sizeof(local_sbuf),"HTTP error: %d",http.errorToString(code).c_str());
    Local_Logs_Dispatch(local_sbuf);
  }
  http.end();
  Local_Logs_Dispatch("Closing connection");
  OTAinProgress=0;
  percentCounter=0;
  //stack_mon(hwm);
}

void TaskUpdateNextion(void)
{
  Local_Logs_Dispatch("Nextion Update Requested");
  Local_Logs_Dispatch("Stopping PoolMaster...");
  pinMode(ENPin, OUTPUT);
  digitalWrite(ENPin, LOW);
  Local_Logs_Dispatch("Upgrading Nextion ...");

  HTTPClient http;
  char url[_LURL_];
  const char* host=SVSettings["Update Host"];
  const char* path=SVSettings["Nextion Path"];
  sprintf(url, "http://%s%s",host,path); 
  snprintf(local_sbuf,sizeof(local_sbuf),"Requesting URL: %s",url);
  Local_Logs_Dispatch(local_sbuf);
 
  if(!http.begin(url)){
    Local_Logs_Dispatch("Connection failed");
    return;
    }

    
  // This will send the (get) request to the server
  int code          = http.GET();
  contentLength     = http.getSize();
        
  // Update the nextion display
  if(code == 200){
    OTAinProgress=1;
    Local_Logs_Dispatch("File received. Update Nextion...");
    bool result;

    // initialize ESPNexUpload
    ESPNexUpload nextion(115200);
    // set callback: What to do / show during upload..... Optional! Called every 2048 bytes
    UpdateCounter=0;
    nextion.setUpdateProgressCallback([](){
      UpdateCounter++;
      percentCounter = (float(UpdateCounter*2048)/contentLength)*100.0;
      snprintf(local_sbuf,sizeof(local_sbuf),"Nextion update %4.1f%%",percentCounter);
      strcpy(barBuf, local_sbuf);
      Local_Logs_Dispatch(local_sbuf,1,"\r");
    });
    // prepare upload: setup serial connection, send update command and send the expected update size
    result = nextion.prepareUpload(contentLength);
    if(!result){
        snprintf(local_sbuf,sizeof(local_sbuf),"Error: %s",nextion.statusMessage.c_str());
        Local_Logs_Dispatch(local_sbuf);
        //Serial.println("Error: " + nextion.statusMessage);
    } 
    else {
      snprintf(local_sbuf,sizeof(local_sbuf),"Start upload. File size is: %d bytes",contentLength);
      strcpy(barBuf, local_sbuf);
      Local_Logs_Dispatch(local_sbuf);
      // Upload the received byte Stream to the nextion
      result = nextion.upload(*http.getStreamPtr());
      if(result)
        Local_Logs_Dispatch("Successfully updated Nextion");
      else {
        snprintf(local_sbuf,sizeof(local_sbuf),"Error updating Nextion: %s",nextion.statusMessage.c_str());
        Local_Logs_Dispatch(local_sbuf);
      }

      // end: wait(delay) for the nextion to finish the update process, send nextion reset command and end the serial connection to the nextion
      nextion.end();
      pinMode(NEXT_RX,INPUT);
      pinMode(NEXT_TX,INPUT);

      percentCounter=0;
      OTAinProgress=0;
    }
  }
  else {
    // else print http error
    snprintf(local_sbuf,sizeof(local_sbuf),"HTTP error: %d",http.errorToString(code).c_str());
    Local_Logs_Dispatch(local_sbuf);
  }

  http.end();
  Local_Logs_Dispatch("Closing connection");
  Local_Logs_Dispatch("Starting PoolMaster ...");
  digitalWrite(ENPin, HIGH);
  pinMode(ENPin, INPUT);
  //rtc_wdt_enable();
  //rtc_wdt_protect_on();
}

void TelnetToTaskUpdatePoolMaster()
{
  WiFiClient telnet;
  if (!telnet.connect(WiFi.localIP(), 23)) return;
  telnet.write("S\r\n");
  delay(3000); // must add a delay, otherwise telnet closes too early
  // telnet.end();
}

//////////////////////// ELEGANT OTA //////////////////////////
////////////////////////////////////////////////////////////
void onOTAStart() {
  // Log when OTA has started
  Local_Logs_Dispatch("OTA update started!");
  OTAinProgress=1;
  percentCounter=0;
  // <Add your own code here>
}

void onOTAProgress(size_t current, size_t final) {
  // Log every 1 second
  if (millis() - ota_progress_millis > 1000) {
    ota_progress_millis = millis();
      percentCounter = (float(current)/final)*100.0;
      snprintf(local_sbuf,sizeof(local_sbuf),"SurperVisor update %4.1f%%",percentCounter);
      strcpy(barBuf, local_sbuf);
      //snprintf(local_sbuf,sizeof(local_sbuf),"OTA Progress Current: %u bytes, Final: %u bytes", current, final);
      Local_Logs_Dispatch(local_sbuf);
  }
}

void onOTAEnd(bool success) {
  // Log when OTA has finished
  if (success) {
    Local_Logs_Dispatch("OTA update finished successfully!");
  } else {
    Local_Logs_Dispatch("There was an error during OTA update!");
  }
  OTAinProgress=0;
  percentCounter=0;
  // <Add your own code here>
}

void initElegantOTA() {
  ElegantOTA.begin(&Webserver);    // Start ElegantOTA
  // ElegantOTA callbacks
  ElegantOTA.onStart(onOTAStart);
  ElegantOTA.onProgress(onOTAProgress);
  ElegantOTA.onEnd(onOTAEnd);
}

//////////////////////// COMMANDS //////////////////////////
////////////////////////////////////////////////////////////
void cmdExecute(char _command) {
  //snprintf(local_sbuf,sizeof(local_sbuf),"Command Arrived %s",_command);
  //Local_Logs_Dispatch(local_sbuf);
  switch (_command) {
    case 'R': // WatchDog Reboot
      delay(100);
      ESP.restart();
    break;
    case 'P':  // PoolMaster Stop
      Local_Logs_Dispatch("Stopping PoolMaster ...");
      pinMode(ENPin, OUTPUT);
      digitalWrite(ENPin, LOW);
    break;
    case 'Q':  // PoolMaster Start
      Local_Logs_Dispatch("Starting PoolMaster ...");
      if(digitalRead(ENPin)==LOW) {
        pinMode(ENPin, OUTPUT);
        digitalWrite(ENPin, HIGH);
        pinMode(ENPin, INPUT);
      }
    break;
    case 'S':  // PoolMaster Update
      //mustUpdatePoolMaster = true;
      TaskUpdatePoolMaster();
    break;
    case 'T':  // Nextion Update
      mustUpdateNextion = true;
    break;
    case 'H':  // Help
      Local_Logs_Dispatch("***********************");  
      Local_Logs_Dispatch("Help Message:");
      Local_Logs_Dispatch("R: Reboot WatchDog");
      Local_Logs_Dispatch("P: Stop PoolMaster");
      Local_Logs_Dispatch("Q: Start PoolMaster");
      Local_Logs_Dispatch("S: Update PoolMaster");
      Local_Logs_Dispatch("T: Update Nextion");
      Local_Logs_Dispatch("***********************");  
    break;
  }
}

//////////////////////// WEBSERIAL //////////////////////////
////////////////////////////////////////////////////////////
// ----------------------------------------------------------------------------
// Message Callback WebSocket
// ----------------------------------------------------------------------------
#ifdef TARGET_WEBSERIAL
void recvMsg(uint8_t *data, size_t len) {
  //WebSerial.println("Received Data...");
  String d = "";
  for(int i=0; i < len; i++){
    d += char(data[i]);
  }
  //cmdExecute(data[0]);
  //WebSerial.println(d);
}
#endif

void upcurrenttime()
{
    uptime::calculateUptime();
    sprintf(currentUptime, "%dd-%02dh-%02dm-%02ds", uptime::getDays(), uptime::getHours(), uptime::getMinutes(), uptime::getSeconds());
}

// MQTT Engine
// ***********
/*void onMqttSubscribe(uint16_t packetId, uint8_t qos) {}
void onMqttUnSubscribe(uint16_t packetId) {}
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {}
void onMqttPublish(uint16_t packetId) {}
*/

void mqttPublish()
{  
  char Payload[PAYLOAD_BUFFER_LENGTH];
  char topic[_LTOPIC_];
  const char* roottopic = SVSettings["MQTT Topic"];
  sprintf(topic, "%s/Supervision/PoolMaster", roottopic);
  if (!MqttClient.connected()) return;
  
  {
    // remove unnecessary data
  JsonDocument cleanJson = PMInfo;
  cleanJson.remove("MQTT Server");
  cleanJson.remove("MQTT Port");
  cleanJson.remove("MQTT Topic");
  cleanJson.remove("MQTT Username");
  cleanJson.remove("MQTT Password");
  cleanJson.remove("Update Host");
  cleanJson.remove("Poolmaster Path");
  cleanJson.remove("Nextion Path");

  size_t n = serializeJson(cleanJson, Payload);
  if (MqttClient.publish(topic, 1, true, Payload, n) == 0) {
      snprintf(local_sbuf,sizeof(local_sbuf),"Supervisor, unable to publish MQTT %s Payload: %s - Payload size: %d", topic, Payload, sizeof(Payload));
      Local_Logs_Dispatch(local_sbuf);
  }
  }
  
  sprintf(topic, "%s/Supervision/SuperVisor", roottopic);
  if (!MqttClient.connected()) return;
  {
    // remove unnecessary data
  JsonDocument cleanJson = SVSettings;
  cleanJson.remove("MQTT Server");
  cleanJson.remove("MQTT Port");
  cleanJson.remove("MQTT Topic");
  cleanJson.remove("MQTT Username");
  cleanJson.remove("MQTT Password");
  cleanJson.remove("LED");
  cleanJson.remove("Display Firmware");

  size_t n = serializeJson(cleanJson, Payload);
  if (MqttClient.publish(topic, 1, true, Payload, n) == 0) {
      snprintf(local_sbuf,sizeof(local_sbuf),"Supervisor, unable to publish MQTT %s Payload: %s - Payload size: %d", topic, Payload, sizeof(Payload));
      Local_Logs_Dispatch(local_sbuf);
  }
  }
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) 
{
    if (WiFi.isConnected()) xTimerStart(MqttReconnectTimer, 0);
}

void onMqttConnect(bool sessionPresent) 
{
  //mqttPublish();
}

void connectToMqtt() 
{
  MqttClient.connect();
}

void MqttInit()
{   
    MqttReconnectTimer = xTimerCreate("mqttTimer",
        pdMS_TO_TICKS(5000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
 
    MqttClient.onConnect(onMqttConnect);
    MqttClient.onDisconnect(onMqttDisconnect);
    //MqttClient.onSubscribe(onMqttSubscribe);
   // MqttClient.onUnsubscribe(onMqttUnSubscribe);
   // MqttClient.onMessage(onMqttMessage);
   // MqttClient.onPublish(onMqttPublish);
   const char* mqtt_server=SVSettings["MQTT Server"];
   int mqtt_port=atol(SVSettings["MQTT Port"]);
   const char* mqtt_username=SVSettings["MQTT Username"];
   const char* mqtt_password=SVSettings["MQTT Password"];

    MqttClient.setServer(mqtt_server, mqtt_port);
    if (*mqtt_username != 0)
        MqttClient.setCredentials(mqtt_username, mqtt_password);

    connectToMqtt();
}

void onI2CRequest(void) 
{
  // ESP32 Slave I2c has a strange behavior (S3 or C3 are differents)
  // https://forum.arduino.cc/t/understanding-esp32-wireslave-example/1000680/7
  // OnRequest must exist but empty
  // I2C slave flushes the buffer when onRequest is triggered !
  return; 
}

String PMRequest(char *question)
{
  if (strcmp(question, "GET_WIFI_SSID") == 0)  return wifiManager.getWiFiSSID();
  if (strcmp(question, "GET_WIFI_PASS") == 0)  return wifiManager.getWiFiPass();
  if (strcmp(question, "GET_HOSTNAME") == 0)   return String(hostname);

  //  PM will update its MQTT if Topic has changed
  if (strcmp(question, "GET_MQTT_SERVER") == 0)   return SVSettings["MQTT Server"];
  if (strcmp(question, "GET_MQTT_PORT") == 0)     return SVSettings["MQTT Port"];
  if (strcmp(question, "GET_MQTT_TOPIC") == 0)    return SVSettings["MQTT Topic"];
  if (strcmp(question, "GET_MQTT_USERNAME") == 0) return SVSettings["MQTT Username"];
  if (strcmp(question, "GET_MQTT_PASSWORD") == 0) return SVSettings["MQTT Password"];

  return String("");
}


void onI2CReceive(int len) 
{
  // Received a request from PoolMaster
  char question[I2C_MAXMESSAGE];
  int i=0;
  while (Wire.available()>0) question[i++] = Wire.read();
  question[i] = '\0';

  // deal with the message recevied from PoolMaster
  String Svalue = "Ok";

  // PoolMaster sends reports when question starts with PM_
  // consume too much I2C bandwidth, use Serial instead.
  //if (strncmp(question, "PM_", 3) == 0) {
  //  PMReport(question+3);
  //}
  // PoolMaster wants info from SuperVisor
  if (strncmp(question, "GET_", 4) == 0) {
    Svalue = PMRequest(question);
  }
  else {  // Just a message to print
    sprintf(local_sbuf,"%s",question);
    Local_Logs_Dispatch(local_sbuf);
  }
 
  // SlaveWrite should be in function onRequest except for ESP32 nonS3-nonC3 !
  // fill the buffer, use I2C_MAXMESSAGE not strlen, _DELIMITER_ is EOL
  Svalue += _DELIMITER_;
  Wire.slaveWrite((uint8_t *)Svalue.c_str(), I2C_MAXMESSAGE); 
}

void saveConfigCallback() 
{
   shouldSaveConfig = true;
}

// HTML pages ***
// **************
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>PoolMaster SuperVisor</title>
  </head>
<style>
* {
  margin: 0;
  padding: 0;
  box-sizing: border-box;
  }
body{
  background: linear-gradient(AliceBlue, LightCyan, MediumBlue);
  background-size: cover;
  background-attachment: fixed;
  }
.navbar{
  height: 50px;
  display: flex; 
  align-items: center;
  width: 100%;
  padding: 14px 14px;
  background-color: #1b4cd3;
  position: fixed;
  z-index: 1000;
  }
.navbar .nav-header{
  display: inline;
  }
.navbar .nav-header .nav-logo{
  display: inline-block;
  margin-top: 5px;
  }
.navbar .nav-title{
    display: none;
  }
.navbar .nav-links{
  display: inline;
  float: left;
  font-size: 18px;
  }
.navbar .nav-links a{
  padding: 10px 12px;
  text-decoration: none;
  font-weight: 550;
  color: white;
  }
.navbar .nav-links button{
  padding: 10px 12px;
  text-decoration: none;
  font-weight: 550;
  cursor: pointer;
  color: white;
  border: none;
  outline: none;
  background-color: #1b4cd3;
  }
.navbar .nav-links a:hover{
  background-color: rgba(0,0,0,0.3);
  }
  .navbar .nav-links button:hover{
  background-color: rgba(0,0,0,0.3);
  }
.navbar #nav-check, .navbar .nav-btn{
  display: none;
  }
@media (max-width: 600px){
  .navbar .nav-btn{
    display: inline-block;
    position: absolute;
    top: 0px;
    right: -20px;
    }
  .navbar #idnav-title, .navbar .nav-title{
    margin-top: 3px;
    color: white;
    display: block;
    user-select: none; 
    font-size: large;
    padding: 15px;
  }
  .navbar .nav-btn label{
    display: inline-block;
    width: 80px;
    height: 50px;
    padding: 15px;
    }
  .navbar .nav-btn label span{
    display: block;
    height: 10px;
    width: 25px;
    border-top: 3px solid #eee;
    }
  .navbar .nav-btn label:hover, .navbar #nav-check:checked ~ .nav-btn label{
    background-color: rgb(9,14,90);
    }
  .navbar .nav-links{
    position: absolute;
    display: block;
    text-align: 50%;
    background-color: rgb(9,14,90);
    transition: all 0.3s ease-in;
    overflow-y: hidden;
    top: 50px;
    right: 0px;
    }
  .navbar .nav-links a{
    display: block;
    }
  .navbar .nav-links button{
    display: block;
    background-color: rgb(9,14,90);
    border: none;
    outline: none;
    }
  .navbar #nav-check:not(:checked) ~ .nav-links{
    height: 0px;
    }
  .navbar #nav-check:checked ~ .nav-links{
    height: calc(100vh - 50px);
    overflow-y: auto;
    }
  }
.tabcontent {
  color: black;
  display: none;
  padding: 60px 7px 10px;
  height: 100%;
}
.textblocklogs {
  background-color: white;
  font-size: small;
  }
.textblockabout {
  background-color: white;
  font-size: small;
  }
.infotable {
  background-color: white;
  padding: 0px 7px 0px 7px;
  font-size: small;
  }
.infotable th:nth-child(1) {
  text-align: left;
  width: 130px;
}
.infotable th:nth-child(2) {
  text-align: left;
  white-space: nowrap; 
  text-overflow:ellipsis; 
  overflow: hidden;
}
.normalButton {
  background-color: Silver;
  display: inline-block; font-weight: bold; border: 1px solid #2d6898;
  color: white;
  padding: 10px 15px;
  border-radius: 4px;
  cursor: pointer;
  margin-right: 10px;
  height:40px;
  width:300px;
}
.redButton {
  background-color: Red;
  display: inline-block; font-weight: bold; border: 1px solid #2d6898;
  color: white;
  padding: 10px 15px;
  border-radius: 4px;
  cursor: pointer;
  margin-right: 10px;
  height:40px;
  width:300px;
}
.blackButton {
  background-color: Black;
  display: inline-block; font-weight: bold; border: 1px solid #2d6898;
  color: white;
  padding: 10px 15px;
  border-radius: 4px;
  cursor: pointer;
  margin-right: 10px;
  height:40px;
  width:300px;
}
.shareButton {
  background-color: SkyBlue;
  display: inline-block; font-weight: bold; border: 1px solid #2d6898;
  color: white;
  padding: 10px 15px;
  border-radius: 4px;
  cursor: pointer;
  margin-right: 10px;
  height:40px;
  width:300px;
}
.formsettings form  { display: table;      }
.formsettings p     { display: table-row;  }
.formsettings label { display: table-cell; }
.formsettings input { display: table-cell; width: 300px;}
.marge5 { margin-left: 2em; }
#myProgress {
  display: none
  width:300px;
  background-color: #A9A9A9;
  border-radius: 15px;
}
#myBar {
  display: none
  width: 0;
  height: 20px;
  background-color: blue;
  text-align: center;
  line-height: 20px;
  color: white;
  border-radius: 15px;
  white-space: nowrap;
}
</style>

<div class="navbar">
  <div class="nav-header">
    <div class="nav-logo"> 
      <a href="#">
         <img alt="logo" src="data:image/jpeg;base64,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" />
      </a>
    </div>
  </div>
  <div class="nav-title" id="idnav-title">PoolMaster Supervision</div>
  <input type="checkbox" id="nav-check">
  <div class="nav-btn">
    <label for="nav-check">
      <span></span>
      <span></span>
      <span></span>
    </label>
  </div>
  <div class="nav-links">
    <button class="tablink" onclick="openPage('Poolmaster', this)" id="defaultOpen">Poolmaster</button>
    <button class="tablink" onclick="openPage('Settings', this)">Settings</button>
    <button class="tablink" onclick="openPage('Logs', this)">Logs</button>
    <button class="tablink" onclick="openPage('Tools', this)">Tools</button>
    <button class="tablink" onclick="openPage('About', this)">About</button>
  </div>
  <script>
function openPage(pageName,elmnt) {
  var i, tabcontent, tablinks;
  document.getElementById("nav-check").checked = false;
  tabcontent = document.getElementsByClassName("tabcontent");
  for (i = 0; i < tabcontent.length; i++) {
    tabcontent[i].style.display = "none";
  }
  tablinks = document.getElementsByClassName("tablink");
  for (i = 0; i < tablinks.length; i++) {
    tablinks[i].style.backgroundColor = "";
  }
  document.getElementById(pageName).style.display = "block";
}
function startTab() {
  document.getElementById("defaultOpen").click();
}
let mqtt_server = "";
let mqtt_port = "";
let mqtt_topic = "";
let mqtt_username = "";
let mqtt_password = "";
let update_host = "";
let poolmasterpath = "";
let nextionpath = "";

</script>
</div>

<body>

<!-- 1st INFO Page -->
<div id="Poolmaster" class="tabcontent">
  <script>
    function generateTable(source, target, title, loadsettings) {
      var xhttp = new XMLHttpRequest();
      xhttp.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          data = JSON.parse(this.responseText);
          let html = "<p><table border='0'>";
          html += "<tr><th>" + title + "</th></tr>";
          for (var key in data) {
            if (data[key] != "none") {
              html += "<tr><td>" + key + ":</td>"; 
              html += "<td>" + data[key] + "</td></tr>";
            }
          }
          html += "</p></table>";
          document.getElementById(target).innerHTML = html;
          if (loadsettings == 1) {
            mqtt_server    = data["MQTT Server"];
            mqtt_port      = data["MQTT Port"];
            mqtt_topic     = data["MQTT Topic"];
            update_host    = data["Update Host"];
            poolmasterpath = data["Poolmaster Path"];
            nextionpath    = data["Nextion Path"];
          }
        }
      }
      xhttp.open("GET", source, true);
      xhttp.send();
    }
    function setUpdateBar(source, target, title, loadsettings) {
      var xhttp = new XMLHttpRequest();
      xhttp.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          data = JSON.parse(this.responseText);
          var elem = document.getElementById('myBar');
          var progress = document.getElementById('myProgress');
          var pc=data['pc'];
          if (pc == 0) {
            elem.style.display = "none";
            progress.style.display = "none";
          } else {
            progress.style.display = "block";
            elem.style.display = "block";
            elem.style.width = pc +"%";
            elem.innerHTML = data['text'];
          }
        }
      }
      xhttp.open("GET", "/getprogressbar", true);
      xhttp.send();
    }

    setInterval(function() {
      generateTable("/getpminfo","pm-table-container","PoolMaster:", 0);
      generateTable("/getsvinfo","sv-table-container","SuperVisor:", 1);
      setUpdateBar();
    }, 900);
  </script>
  <div id="myProgress">
    <div id="myBar"></div>
  </div>
  <div class="infotable" id="pm-table-container"></div>
  <br>
  <div class="infotable" id="sv-table-container"></div>
</div>

<!-- SETTINGS -->
<div id="Settings" class="tabcontent">
  <script>
    function loadsettingsvalues() {
      document.getElementById("MQTT Server").value = mqtt_server
      document.getElementById("MQTT Port").value = mqtt_port;
      document.getElementById("MQTT Topic").value = mqtt_topic;
      document.getElementById("Update Host").value = update_host;
      document.getElementById("Poolmaster Path").value = poolmasterpath;
      document.getElementById("Nextion Path").value = nextionpath;
    }
  </script>
  <br>
  <button class="redButton" onclick="loadsettingsvalues()">Click me to load values</button><br>
  <br>
  <form class="formsettings" id="svsettings" action='/set' method='get'>
    <label for="MQTT Server">MQTT Server:</label>
    <input type="text" name="MQTT Server" id="MQTT Server">
    <br>
    <label for="MQTT Port">MQTT Port :</label>
    <input type="text" name="MQTT Port" id="MQTT Port">
    <br>
    <label for="MQTT Topic">MQTT Topic :</label>
    <input type="text" name="MQTT Topic" id="MQTT Topic">
    <br>
    <label for="MQTT Username">MQTT Username :</label>
    <input type="text" name="MQTT Username" id="MQTT Username">
    <br>
    <label for="MQTT Password">MQTT Password :</label>
    <input type="password" name="MQTT Password" id="MQTT Password">
    <br>
    <br>
    <label for="Update Host">Update Host (http://):</label>
    <input type="text" name="Update Host" id="Update Host">
    <br>
    <label for="Poolmaster Path">PoolMaster FW Path:</label>
    <input type="text" name="Poolmaster Path" id="Poolmaster Path">
    <br>
    <label for="Nextion Path">Display FW Path:</label>
    <input type="text" name="Nextion Path" id="Nextion Path">
    <br>
    <br>
  <button type="submit" class="normalButton">Save</button>
  <br>
 </form>
  <br>
</div>

<!-- LOGS -->
<div id="Logs" class="tabcontent">
  <script>
    var ScrollFlag = 0;
    function StopScroll()  { ScrollFlag = 0; }
    function StartScroll() { ScrollFlag = 1; }
    function ScrollClean() { 
      var container = document.getElementById('textblock-logs');
      container.innerText = "";
    }
    setInterval(function() {
      var xhttp = new XMLHttpRequest();
      xhttp.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
          var container = document.getElementById('textblock-logs');
          var newElm = document.createElement('p');
          newElm.innerText = this.responseText.replace(/(\n\n|\r)/gm,"\n");
          container.appendChild(newElm);
          if (ScrollFlag == 1) {
            scrollingElement = (document.scrollingElement || document.body);
            scrollingElement.scrollTop = scrollingElement.scrollHeight;
           //container.scrollTop = scrollingElement.scrollHeight; 
          }}}
      xhttp.open("GET", "/getlogs", true);
      xhttp.send();
    }, 500);
  </script>
  <h3 id="Logs">Logs from PoolMaster (PM) and SuperVisor (SV)</h3>
  <br>
  <button onClick="StopScroll()">Stop Scrolling</button>
  <button onClick="StartScroll()">Start Scrolling</button>
  <button onClick="ScrollClean()">Clean</button>
  <hr /><div class="textblocklogs" id="textblock-logs"><p>Logs</p></div><hr />
  <button onClick="StopScroll()">Stop Scrolling</button>
  <button onClick="StartScroll()">Start Scrolling</button>
  <button onClick="ScrollClean()">Clean</button>
  <div><p></p></div>
</div>

<!-- TOOLS -->
<div id="Tools" class="tabcontent">
  <br> 
  <form action="/set">
    <input type="hidden" id="rebootPoolMaster" name="rebootPoolMaster">
    <input type="submit" class="blackButton" value="Restart PoolMaster">
  </form>
  <br>
  <form action="/set">
    <input type="hidden" id="rebootSuperVisor" name="rebootSuperVisor">
    <input type="submit" class="blackButton" value="Restart SuperVisor">
  </form>
  <br>
  <br>
  <form action="/set">
    <input type="hidden" id="UpdatePoolMaster" name="UpdatePoolMaster">
    <input type="submit" class="redButton" value="Update PoolMaster">
  </form>
  <br>
  <form action="/set">
    <input type="hidden" id="UpdateNextion" name="UpdateNextion">
    <input type="submit" class="redButton" value="Update Display Nextion">
  </form>
  <br>
    <a href="/update"><button class="redButton">Update SuperVisor (Elg. OTA)</button></a>
  <br><br>
</div>

<!-- ABOUT -->
<div id="About" class="tabcontent">
  <br>
  <br>
  <hr />
  <div class="textblockabout">
  THE SOFTWARE IS PROVIDED “AS IS”,
  <br>
  WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
  <br> 
  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
  <br>
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
  <br>
  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
  <br>
  DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
  <br>TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE 
  <br>
  OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
  </div><hr/>
  <br> 
  <hr />
  <div class="textblockabout">
  Credits:
  <br><a class="marge5" href="https://github.com/christophebelmont">Christophe Belmont</a>
  <br><a class="marge5" href="https://github.com/Gixy31/ESP32-PoolMaster">Gixy31</a>
  <br><a class="marge5" href="https://github.com/Loic74650/PoolMaster">Loic74650</a>
  </div><hr/>
  <p>
</div>

  <script>
    startTab();
  </script>
</body></html>)rawliteral";

void InitSortedSettings()
{
  // JSON will have values in this order
  // Required to display webpage info correctly.
  const char sortedSettings[][20] = {
    "Chip Model",
    "Flash Size",
    "CPU Freq (Mhz)",
    "CPU Cores",
    "Firmware",
    "Display Firmware",
    "Hostname",
    "IP Address",
    "MAC Address",
    "Wifi SSID",
    "Wifi RSSI",
    "LED",
    "Uptime",
    "HEAP size (KB)",
    "HEAP free (KB)",
    "HEAP maxAlloc",
    "Update Host",
    "Poolmaster Path",
    "Nextion Path",
    "MQTT Server",
    "MQTT Port",
    "MQTT Topic",
  };

  int n = sizeof(sortedSettings)/sizeof(sortedSettings[0]);
  for (int i=0; i<n; i++) {
    const char* key=sortedSettings[i];
    PMInfo[key]     = "unknown";
    SVSettings[key] = "unknown";
  } 
  // remove unwanted info
  PMInfo["MQTT Username"]         = "none";
  PMInfo["MQTT Password"]         = "none";
  PMInfo["Update Host"]           = "none";
  PMInfo["Poolmaster Path"]       = "none";
  PMInfo["Nextion Path"]          = "none";
  SVSettings["LED"]               = "none";
  SVSettings["Display Firmware"]  = "none";
  
  // load static info
  SVSettings["MAC Address"]       = WiFi.macAddress();
  SVSettings["Firmware"]          = PMSV_VERSION;
  SVSettings["Chip Model"]        = ESP.getChipModel();
  SVSettings["Flash Size"]        = ESP.getFlashChipSize();
  SVSettings["CPU Freq (Mhz)"]    = ESP.getCpuFreqMHz();
  SVSettings["CPU Cores"]         = ESP.getChipCores();
}

void loadSettings()
{
  InitSortedSettings();
  preferences.begin("PMSV", true);
  //hostname = preferences.getString("Hostname", "");
  preferences.getString("Hostname",hostname,15);

  SVSettings["Hostname"]         = hostname;

  SVSettings["MQTT Server"]      = preferences.getString("MQTT Server", defaultmqtt_server);
  SVSettings["MQTT Port"]        = preferences.getString("MQTT Port", defaultmqtt_port);
  SVSettings["MQTT Topic"]       = preferences.getString("MQTT Topic", defaultmqtt_topic);
  SVSettings["MQTT Username"]    = preferences.getString("MQTT Username", defaultmqtt_username);
  SVSettings["MQTT Password"]    = preferences.getString("MQTT Password", defaultmqtt_password);

  SVSettings["Update Host"]      = preferences.getString("Update Host", defaultUpdatehost);
  SVSettings["Poolmaster Path"]  = preferences.getString("Poolmaster Path", defaultPoolmasterPath);
  SVSettings["Nextion Path"]     = preferences.getString("Nextion Path", defaultNextionPath);

#ifdef TARGET_PAPERTRAIL
  SVSettings["Papertrail Host"]  = preferences.getString("Nextion Path", defaultpapertrailhost);
  SVSettings["Papertrail Port"]  = preferences.getString("Nextion Path", defaultpapertrailport);
#endif

  preferences.end();
}

String WebHandleLogs()
{ 
  String Slogs = "";
  while (char* theline = myLogsRingBuffer.pull()) {
    Slogs += String(theline) + "\n";
  }
  return Slogs; 
};

String WebHandleProgressBar()
{ 
  char Payload[PAYLOAD_BUFFER_LENGTH];
  sprintf(Payload, "{ \"text\":\"%s\", \"pc\":\"%3.1f%\"}", barBuf, percentCounter);
  return String(Payload);
}; 

String WebHandlePMInfo()
{ 
  if (OTAinProgress) return String("");
  char Payload[PAYLOAD_BUFFER_LENGTH];
  size_t n = serializeJson(PMInfo, Payload);
  return String(Payload);
}; 

String WebHandleSVSettings()
{ 
  if (OTAinProgress) return String("");
  char buffer[15];
  upcurrenttime();
  SVSettings["Uptime"]      = currentUptime;
  SVSettings["Hostname"]    = WiFi.getHostname();
  SVSettings["IP Address"]  = WiFi.localIP().toString();
  SVSettings["Wifi SSID"]   = wifiManager.getWiFiSSID();
  SVSettings["Wifi RSSI"]   = WiFi.RSSI();
  dtostrf(ESP.getHeapSize() / 1024.0, 3, 3, buffer);
  SVSettings["HEAP size (KB)"] = buffer;
  dtostrf(ESP.getFreeHeap() / 1024.0, 3, 3, buffer);
  SVSettings["HEAP free (KB)"] = buffer;
  dtostrf(ESP.getMaxAllocHeap() / 1024.0, 3, 3, buffer);
  SVSettings["HEAP maxAlloc"]  = buffer;

  // remove unnecessary data
  JsonDocument cleanJson = SVSettings;
  cleanJson.remove("MQTT Username");
  cleanJson.remove("MQTT Password");
  char Payload[PAYLOAD_BUFFER_LENGTH];
  size_t n = serializeJson(cleanJson, Payload);

  return String(Payload);
}; 

void WebSetActionManageParam(AsyncWebServerRequest *request, const char* key) 
{
  if (request->hasParam(key)) {
      String setting=request->arg(key);
      if (setting != "") {
        SVSettings[key] = String(setting);;
        preferences.putString(key, setting);
      }
    }
}
void WebSetAction(AsyncWebServerRequest *request) 
{
  preferences.begin("PMSV", false);
  WebSetActionManageParam(request, "MQTT Server");
  WebSetActionManageParam(request, "MQTT Port");
  WebSetActionManageParam(request, "MQTT Topic");
  WebSetActionManageParam(request, "MQTT Username");
  WebSetActionManageParam(request, "MQTT Password");
  WebSetActionManageParam(request, "Update Host");
  WebSetActionManageParam(request, "Poolmaster Path");
  WebSetActionManageParam(request, "Nextion Path");
  
#ifdef TARGET_PAPERTRAIL 
  WebSetActionManageParam(request, "Papertrail Host");
  WebSetActionManageParam(request, "Papertrail Port");
#endif  
  preferences.end();

  if (request->hasParam("rebootSuperVisor")) {
    request->redirect("/"); 
    delay(300);
    ESP.restart();
  }

  if (request->hasParam("rebootPoolMaster")) 
    mustRebootPoolMaster = true;
    
  if (request->hasParam("UpdatePoolMaster"))
    mustUpdatePoolMaster = true;

  if (request->hasParam("UpdateNextion"))
    mustUpdateNextion = true;
  
  request->redirect("/"); 
}

void InitWebServer()
{
  initElegantOTA();

  #ifdef TARGET_WEBSERIAL
  // Start WebSerial
  WebSerial.begin(&Webserver);
  WebSerial.onMessage(&recvMsg); /* Attach Message Callback */
  #endif

   // Send web page with input fields to client
  Webserver.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", index_html);
  });
  Webserver.onNotFound([](AsyncWebServerRequest* request) { request->redirect("/"); });
  Webserver.on("/getlogs", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", WebHandleLogs());
  });
  Webserver.on("/getpminfo", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", WebHandlePMInfo());
  });
  Webserver.on("/getsvinfo", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", WebHandleSVSettings());
  });
  Webserver.on("/getprogressbar", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", WebHandleProgressBar());
  });
  Webserver.on("/set", HTTP_GET, WebSetAction);
  //Webserver.on("/set", HTTP_POST, WebSetAction);

  Webserver.begin();
}


///////////// Update NEXTION and al ////////////////
////////////////////////////////////////////////////
void TheTasksLoop(void *pvParameters)
{
  static UBaseType_t hwm=0;     // free stack size
  rtc_wdt_protect_off();
  rtc_wdt_disable();
  int delaymqtt=0; 
  for (;;) {
    delay(500);
    if(mustUpdateNextion) {
      mustUpdateNextion = false;
      TaskUpdateNextion();
    }
    if (mustUpdatePoolMaster) {
      mustUpdatePoolMaster = false;
      TelnetToTaskUpdatePoolMaster();
    }
    if (mustRebootPoolMaster) {
      mustRebootPoolMaster = false;
      pinMode(ENPin, OUTPUT);
      digitalWrite(ENPin, LOW);
      delay(pdMS_TO_TICKS(200));
      pinMode(ENPin, OUTPUT);
      digitalWrite(ENPin, HIGH);
      pinMode(ENPin, INPUT);
    }
    // send mqtt data every xx cycles
    if (delaymqtt++ == 10) {
      delaymqtt=0;
      mqttPublish();
    } 
    stack_mon(hwm);
  }
}

//////////////////////// SETUP //////////////////////////
/////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);

  loadSettings();

  // Start WifiManager and resetPin
  pinMode(RESET_WIFI_PIN, INPUT_PULLUP);

  // Set hostname if any
  if (hostname[0] != '\0') {  // with ESP32, set hostname before wifi.mode !
    sprintf(myhostname, "%s%s", hostname, SuperVisor_Suffix); // SuperVisor Hostname
    WiFi.setHostname(myhostname);
  }
  WiFi.mode(WIFI_STA);
  
  // Create uniq SSID
  char ssid[32];uint32_t id=0;
  for(int i=0; i<17; i=i+8) id |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  sprintf(ssid, "%s%08X", _DEFAULT_NAME_, id);

  // Choose Hostname(s) for our ESP32s in WifiManager
  WiFiManagerParameter custom_hostname("Hostname", "Hostname", _DEFAULT_NAME_, 40);
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  wifiManager.addParameter(&custom_hostname);
  std::vector<const char*> wmMenuItems = {"wifi"};
  wifiManager.setMenu(wmMenuItems);
  wifiManager.setCustomHeadElement("<style>body{max-width:500px;margin:auto}</style>");
  if (!wifiManager.autoConnect(ssid)) {
		delay(pdMS_TO_TICKS(3000));
		ESP.restart();
  }
  if (shouldSaveConfig) {
    //* this is the 1st reboot after wifimanager
    String customhostname = custom_hostname.getValue();
    preferences.begin("PMSV", false);
    preferences.putString("Hostname", customhostname);
    preferences.end();
    delay(pdMS_TO_TICKS(1000));
		ESP.restart();    // ESP32 must reboot but why, anyway
  }

  MDNS.addService("http", "tcp", 80);
  MDNS.begin(myhostname);      

  Local_Logs_Dispatch("");
  Local_Logs_Dispatch("WiFi connected ");
  snprintf(local_sbuf,sizeof(local_sbuf),"Hostname %s, IP address: %s ", WiFi.getHostname(), WiFi.localIP());
  Local_Logs_Dispatch(local_sbuf);

  // Start I2C Slave channel, talking to PoolMaster
  // Using PoolMaster "Extensions"
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);
  Wire.begin(I2C_SLAVE_ADDR, SDA_S, SCL_S, 100000U);

  // Start I2C Master channel for OLED
  Wire1.begin(SDA_M, SCL_M);

  // Config NTP
  configTime(gmtOffset_sec, daylightOffset_sec, "pool.ntp.org");
  time_t now = time(nullptr);
  while (now < SECS_YR_2000) {
    delay(100);
    now = time(nullptr);
  }
  setTime(now);

  InitWebServer();
  MqttInit();

  //Connect to serial receiving messages from PoolMaster
  Serial2.setTimeout(100);
  Serial2.setRxBufferSize(1024);
  Serial2.begin(115200);

  //Start Telnet Server
  #ifdef TARGET_TELNET
  Telnetserver.begin();
  Telnetserver.setNoDelay(true);
  #endif

  Local_Logs_Dispatch("Ready! Use 'telnet ");
  Local_Logs_Dispatch(" 23' to connect");

#ifdef TARGET_PAPERTRAIL
  // Start PaperTrail Logging
  if ((papertrailhost != "") && (papertrailhost != defaultpapertrailhost)) {
    uint16_t port = atoi(papertrailport.c_str());
    logger.configureSyslog(papertrailhost.c_str(), port, ""); // Syslog server IP, port and device name
    //logger.registerSerial(COUNTER, DEBUG, "COUNT", Serial); // Log both to serial...
    logger.registerSyslog(PL_LOG, DEBUG, FAC_LOCAL0, "poolmaster"); // ...and syslog. Set the facility to user
    logger.registerSyslog(WD_LOG, DEBUG, FAC_LOCAL0, "watchdog"); // ...and syslog. Set the facility to user
    logger.log(WD_LOG, INFO, "PoolMaster Logs Started");
    logger.log(WD_LOG, INFO, "WatchDog Logs Started");
  }
 #endif  

  Local_Logs_Dispatch("Creating Tasks");
  // Create loop tasks in the scheduler.
  //------------------------------------
  int app_cpu = xPortGetCoreID();

  xTaskCreatePinnedToCore(
    TheTasksLoop, 
    "TheTasksLoop",
    4500, // The stack size can be checked by calling `uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    NULL,  // No parameter is used
    1,  // Priority
    nullptr,  // Task handle is not used here
    app_cpu
  );
}

void WifiManagerCheckResetLoop() {
  // check for button press
  if (digitalRead(RESET_WIFI_PIN) == LOW ) {
    Serial.println("Button Pressed");
      // still holding button for 3000 ms, reset settings
    delay(pdMS_TO_TICKS(3000)); // reset delay hold
    if (digitalRead(RESET_WIFI_PIN) == LOW ) {
      Serial.println("Wifi Button Held");
      Serial.println("Erasing Config, restarting");
      preferences.begin("PMSV", false);
      preferences.clear();
      preferences.end();
      wifiManager.resetSettings();
      ESP.restart();
    }
  }
}

void parseMsgFromPM(char* msg)
{
  char delim[2];
  delim[0] = _DELIMITER_[0];
  delim[1] = 0;
  char *end;
  while (end = strstr(msg, delim)) {
    *end = 0;
    char *key = msg;
    char *val = strstr(msg, "=");
    *val = 0;
    val++;
    if (key && val) PMInfo[key] = val;
    msg=end+1;
  }
}


//////////////////////// MAIN LOOP //////////////////////////
/////////////////////////////////////////////////////////////
void loop() {
  uint8_t i;

  if (WiFi.status() == WL_CONNECTED) {
    #ifdef TARGET_TELNET
    //check if there are any new clients
    if (Telnetserver.hasClient()) {
      for (i = 0; i < MAX_SRV_CLIENTS; i++) {
        //find free/disconnected spot
        if (!serverClients[i] || !serverClients[i].connected()) {
          if (serverClients[i]) {
            serverClients[i].stop();
          }
          serverClients[i] = Telnetserver.accept();
          if (!serverClients[i]) {
            Local_Logs_Dispatch("available broken");
          }
          IPAddress ip_temp=serverClients[i].remoteIP();
          snprintf(local_sbuf,sizeof(local_sbuf),"New Client %d (%d.%d.%d.%d)",i,ip_temp[0],ip_temp[1],ip_temp[2],ip_temp[3]);
          Local_Logs_Dispatch(local_sbuf);
          break;
        }
      }
      if (i >= MAX_SRV_CLIENTS) {
        //no free/disconnected spot so reject
        Telnetserver.accept().stop();
      }
    }
    //check clients for data
    for (i = 0; i < MAX_SRV_CLIENTS; i++) {
      if (serverClients[i] && serverClients[i].connected()) {
        if (serverClients[i].available()) {
          //get data from the telnet client and push it to the UART
          while (serverClients[i].available()) {
            //Serial.write(serverClients[i].read());
            cmdExecute(serverClients[i].read());
          }
        }
      } else {
        if (serverClients[i]) {
          serverClients[i].stop();
        }
      }
    }
    #endif
    //check UART for data
    if (Serial2.available()) {
      if (readUntil(Serial2, sbuf, "\n")) {
        // `buf` contains the delimiter, it can now be used for parsing.

        if (sbuf[0] == _DELIMITER_[0]) {  // is it a private message from PoolMaster ?
          parseMsgFromPM(sbuf+1);        
        }
        else {

        #ifdef TARGET_TELNET 
        //push UART data to all connected telnet clients
        for (i = 0; i < MAX_SRV_CLIENTS; i++) {
          if (serverClients[i] && serverClients[i].connected()) {
            serverClients[i].write(sbuf, strlen(sbuf));
            delay(1);
          }
        }
        #endif

        // Cleanup buffer and send to the web
        for (int src = 0, dst = 0; src < sizeof(sbuf); src++)
        if (sbuf[src] != '\r') sbuf[dst++] = sbuf[src];

        #ifdef TARGET_WEBSERIAL
        // WebSerial
        WebSerial.printf("%s",sbuf);
        #endif

        myLogsRingBuffer.push("PM ", sbuf);

#ifdef TARGET_PAPERTRAIL
        // Cloud Logger PaperTrail
        if ((papertrailhost != "") && (papertrailhost != defaultpapertrailhost)) {

          int logLevel = 0;
          bool must_send_to_server = false;
          char *loglevel_position;
          // Parse line to get log level and convert to standard syslog levels
          loglevel_position = strstr(sbuf,"[DBG_ERROR  ]");
          if (loglevel_position != NULL) {
            logLevel = 3;
            must_send_to_server = true;
          }

          loglevel_position = strstr(sbuf,"[DBG_WARNING]");
          if (loglevel_position != NULL) {
            logLevel = 4;
            must_send_to_server = true;
          }

          loglevel_position = strstr(sbuf,"[DBG_INFO   ]");
          if (loglevel_position != NULL) {
            logLevel = 5;
            must_send_to_server = true;
          }

          loglevel_position = strstr(sbuf,"[DBG_DEBUG  ]");
          if (loglevel_position != NULL) {
            logLevel = 6;
            must_send_to_server = true;
          }

          // Do not send the verbose to the server
          loglevel_position = strstr(sbuf,"[DBG_VERBOSE]");
          if (loglevel_position != NULL)
            logLevel = 7;

          logger.log(PL_LOG, logLevel, "%s", sbuf);
        }
#endif
      }
      }
    }
  } else {
    Local_Logs_Dispatch("WiFi not connected!");
    #ifdef TARGET_TELNET
    for (i = 0; i < MAX_SRV_CLIENTS; i++) {
      if (serverClients[i]) {
        serverClients[i].stop();
      }
    }
    #endif
    delay(pdMS_TO_TICKS(1000));
  }

  WifiManagerCheckResetLoop();

  // Check for updates
  ElegantOTA.loop();
  #ifdef TARGET_WEBSERIAL
  // Update WebSerial
  WebSerial.loop();
  #endif
}

