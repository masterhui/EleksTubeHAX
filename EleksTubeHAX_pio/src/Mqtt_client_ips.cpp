/*
 * Author: Aljaz Ogrin
 * Project: Alternative firmware for EleksTube IPS clock
 * Original location: https://github.com/aly-fly/EleksTubeHAX
 * Hardware: ESP32
 * File description: Connects to the MQTT Broker "smartnest.cz". Uses a device "Thermostat".
 * Sends status and receives commands from WebApp, Android app or connected devices (SmartThings, Google assistant, Alexa, etc.)
 * Configuration: open file "GLOBAL_DEFINES.h"
 * Reference: https://github.com/aososam/Smartnest/tree/master/Devices/thermostat
 * Documentation: https://www.docu.smartnest.cz/
 */

#include "Mqtt_client_ips.h"
#include "WiFi.h"         // for ESP32
#include <PubSubClient.h> // Download and install this library first from: https://www.arduinolibraries.info/libraries/pub-sub-client
#include <ArduinoJson.h>
#include "TempSensor.h"
#include "TFTs.h"
#include "Backlights.h"
#include "Clock.h"
#include "main.h"

#define concat2(first, second) first second
#define concat3(first, second, third) first second third
#define concat4(first, second, third, fourth) first second third fourth

WiFiClient espClient;
PubSubClient MQTTclient(espClient);

#define MQTT_STATE_ON "ON"
#define MQTT_STATE_OFF "OFF"

#define MQTT_BRIGHTNESS_MIN 0
#define MQTT_BRIGHTNESS_MAX 255

#define MQTT_ITENSITY_MIN 0
#define MQTT_ITENSITY_MAX 7

// Add near the top with other defines
#define MQTT_LWT_TOPIC "tele/IPSTUBE/LWT"
#define MQTT_LWT_ONLINE "Online"
#define MQTT_LWT_OFFLINE "Offline"

// private:
int splitCommand(char *topic, char *tokens[], int tokensNumber, char *commandBuf, int commandBufLen);
bool discoveryReported = false;
void callback(char *topic, byte *payload, unsigned int length);

void MqttReportBattery();
void MqttReportStatus();
void MqttReportPowerState();
void MqttReportWiFiSignal();
void MqttReportTemperature();
void MqttReportNotification(String message);
void MqttReportGraphic(bool force);
void MqttReportBackOnChange();
void MqttReportBackEverything(bool force);
void MqttPeriodicReportBack();

char topic[100];
char msg[5];
uint32_t lastTimeSent = (uint32_t)(MQTT_REPORT_STATUS_EVERY_SEC * -1000);
uint8_t LastNotificationChecksum = 0;
uint32_t LastTimeTriedToConnect = 0;
uint32_t mqttDisconnectedSince = 0;

bool MqttConnected = true; // skip error message if disabled
// commands from server    // = "directive/status"
bool MqttCommandPower = true;
bool MqttCommandMainPower = true;
bool MqttCommandBackPower = true;
bool MqttCommandPowerReceived = false;
bool MqttCommandMainPowerReceived = false;
bool MqttCommandBackPowerReceived = false;

bool MqttCommandUseTwelveHours = false;
bool MqttCommandUseTwelveHoursReceived = false;
bool MqttCommandBlankZeroHours = false;
bool MqttCommandBlankZeroHoursReceived = false;
bool MqttCommandSummerTime = false;
bool MqttCommandSummerTimeReceived = false;

int MqttCommandState = 1;
bool MqttCommandStateReceived = false;

uint8_t MqttCommandBrightness = -1;
uint8_t MqttCommandMainBrightness = -1;
uint8_t MqttCommandBackBrightness = -1;
bool MqttCommandBrightnessReceived = false;
bool MqttCommandMainBrightnessReceived = false;
bool MqttCommandBackBrightnessReceived = false;

char MqttCommandPattern[24] = "";
bool MqttCommandPatternReceived = false;
char MqttCommandBackPattern[24] = "";
bool MqttCommandBackPatternReceived = false;

uint16_t MqttCommandBackColorPhase = -1;
bool MqttCommandBackColorPhaseReceived = false;

uint8_t MqttCommandGraphic = -1;
bool MqttCommandGraphicReceived = false;
uint8_t MqttCommandMainGraphic = -1;
bool MqttCommandMainGraphicReceived = false;

uint8_t MqttCommandPulseBpm = -1;
bool MqttCommandPulseBpmReceived = false;

uint8_t MqttCommandBreathBpm = -1;
bool MqttCommandBreathBpmReceived = false;

float MqttCommandRainbowSec = -1;
bool MqttCommandRainbowSecReceived = false;

bool MqttCommandCountdownStart = false;
bool MqttCommandCountdownStartReceived = false;
uint32_t MqttCommandCountdownDuration = 0;

bool MqttCommandCountdownStop = false;
bool MqttCommandCountdownStopReceived = false;

// status to server
bool MqttStatusPower = true;
bool MqttStatusMainPower = true;
bool MqttStatusBackPower = true;
bool MqttStatusUseTwelveHours = true;
bool MqttStatusBlankZeroHours = true;
bool MqttStatusSummerTime = false;
int MqttStatusState = 0;
int MqttStatusBattery = 7;
uint8_t MqttStatusBrightness = 0;
uint8_t MqttStatusMainBrightness = 0;
uint8_t MqttStatusBackBrightness = 0;
char MqttStatusPattern[24] = "";
char MqttStatusBackPattern[24] = "";
uint16_t MqttStatusBackColorPhase = 0;
uint8_t MqttStatusGraphic = 0;
uint8_t MqttStatusMainGraphic = 0;
uint8_t MqttStatusPulseBpm = 0;
uint8_t MqttStatusBreathBpm = 0;
float MqttStatusRainbowSec = 0;
bool MqttStatusAlternate = false;

int LastSentSignalLevel = 999;
int LastSentPowerState = -1;
int LastSentMainPowerState = -1;
int LastSentBackPowerState = -1;
int LastSentStatus = -1;
int LastSentBrightness = -1;
int LastSentMainBrightness = -1;
int LastSentBackBrightness = -1;
char LastSentPattern[24] = "";
char LastSentBackPattern[24] = "";
int LastSentBackColorPhase = -1;
int LastSentGraphic = -1;
int LastSentMainGraphic = -1;
bool LastSentUseTwelveHours = false;
bool LastSentBlankZeroHours = false;
bool LastSentSummerTime = false;
uint8_t LastSentPulseBpm = -1;
uint8_t LastSentBreathBpm = -1;
float LastSentRainbowSec = -1;
bool LastSentCountdownMode = false;
bool LastSentCountdownRunning = false;
uint32_t LastSentCountdownRemaining = 0xFFFFFFFFu;
bool LastSentAlternate = false;
char LastSentMode[20] = ""; // Add this line to declare the variable

bool MqttStatusCountdownRunning = false;
uint32_t MqttStatusCountdownRemaining = 0;

float MqttCommandTemperature = 0.0;
bool MqttCommandTemperatureReceived = false;
float MqttCommandHumidity = 0.0;
bool MqttCommandHumidityReceived = false;

bool MqttCommandModeReceived = false;
char MqttCommandMode[20] = ""; // Initialize with an empty string

bool MqttCommandAlternate = false;
bool MqttCommandAlternateReceived = false;

bool MqttCommandSaunaPowerReceived = false;
bool MqttCommandSaunaPower = false;

// Initialize sensor status (assume online by default)
bool MqttTemperatureSensorOnline = true;
bool MqttHumiditySensorOnline = true;

double round1(double value)
{
  return (int)(value * 10 + 0.5) / 10.0;
}

void sendToBroker(const char *topic, const char *message)
{
  if (MQTTclient.connected())
  {
    char topicArr[100];
    snprintf(topicArr, sizeof(topicArr), "%s/%s", MQTT_CLIENT, topic);
    MQTTclient.publish(topicArr, message, true);
#ifdef DEBUG_OUTPUT // long output
    Serial.print("Sending to MQTT: ");
    Serial.print(topicArr);
    Serial.print("/");
    Serial.println(message);
#endif
  }
}

void MqttReportState(bool force)
{
#ifdef MQTT_HOME_ASSISTANT
  if (MQTTclient.connected())
  {

    if (force || MqttStatusMainPower != LastSentMainPowerState || MqttStatusMainBrightness != LastSentMainBrightness || MqttStatusMainGraphic != LastSentMainGraphic)
    {

      JsonDocument state;
      state["state"] = MqttStatusMainPower == 0 ? MQTT_STATE_OFF : MQTT_STATE_ON;
      state["brightness"] = MqttStatusMainBrightness;
      state["effect"] = tfts.clockFaceToName(MqttStatusMainGraphic);

      char buffer[256];
      size_t n = serializeJson(state, buffer);
      const char *topic = concat2(MQTT_CLIENT, "/main");
      MQTTclient.publish(topic, buffer, true);
      LastSentMainPowerState = MqttStatusMainPower;
      LastSentMainBrightness = MqttStatusMainBrightness;
      LastSentMainGraphic = MqttStatusMainGraphic;

      Serial.print("TX MQTT: ");
      Serial.print(topic);
      Serial.print(" ");
      Serial.println(buffer);
    }

    if (force || MqttStatusBackPower != LastSentBackPowerState || MqttStatusBackBrightness != LastSentBackBrightness || strcmp(MqttStatusBackPattern, LastSentBackPattern) != 0 || MqttStatusBackColorPhase != LastSentBackColorPhase || MqttStatusPulseBpm != LastSentPulseBpm || MqttStatusBreathBpm != LastSentBreathBpm || MqttStatusRainbowSec != LastSentRainbowSec)
    {

      JsonDocument state;
      state["state"] = MqttStatusBackPower == 0 ? MQTT_STATE_OFF : MQTT_STATE_ON;
      state["brightness"] = MqttStatusBackBrightness;
      state["effect"] = MqttStatusBackPattern;
      state["color_mode"] = "hs";
      state["color"]["h"] = backlights.phaseToHue(MqttStatusBackColorPhase);
      state["color"]["s"] = 100.f;
      state["pulse_bpm"] = MqttStatusPulseBpm;
      state["breath_bpm"] = MqttStatusBreathBpm;
      state["rainbow_sec"] = round1(MqttStatusRainbowSec);

      char buffer[256];
      size_t n = serializeJson(state, buffer);
      const char *topic = concat2(MQTT_CLIENT, "/back");
      MQTTclient.publish(topic, buffer, true);
      LastSentBackPowerState = MqttStatusBackPower;
      LastSentBackBrightness = MqttStatusBackBrightness;
      strncpy(LastSentBackPattern, MqttStatusBackPattern, sizeof(LastSentBackPattern) - 1);
      LastSentBackPattern[sizeof(LastSentBackPattern) - 1] = '\0';
      LastSentBackColorPhase = MqttStatusBackColorPhase;

      Serial.print("TX MQTT: ");
      Serial.print(topic);
      Serial.print(" ");
      Serial.println(buffer);
    }

    if (force || MqttStatusUseTwelveHours != LastSentUseTwelveHours)
    {

      JsonDocument state;
      state["state"] = MqttStatusUseTwelveHours ? MQTT_STATE_ON : MQTT_STATE_OFF;

      char buffer[256];
      size_t n = serializeJson(state, buffer);
      const char *topic = concat2(MQTT_CLIENT, "/use_twelve_hours");
      MQTTclient.publish(topic, buffer, true);
      LastSentUseTwelveHours = MqttStatusUseTwelveHours;

      Serial.print("TX MQTT: ");
      Serial.print(topic);
      Serial.print(" ");
      Serial.println(buffer);
    }

    if (force || MqttStatusBlankZeroHours != LastSentBlankZeroHours)
    {

      JsonDocument state;
      state["state"] = MqttStatusBlankZeroHours ? MQTT_STATE_ON : MQTT_STATE_OFF;

      char buffer[256];
      size_t n = serializeJson(state, buffer);
      const char *topic = concat2(MQTT_CLIENT, "/blank_zero_hours");
      MQTTclient.publish(topic, buffer, true);
      LastSentBlankZeroHours = MqttStatusBlankZeroHours;

      Serial.print("TX MQTT: ");
      Serial.print(topic);
      Serial.print(" ");
      Serial.println(buffer);
    }

    if (force || MqttStatusSummerTime != LastSentSummerTime)
    {
      JsonDocument state;
      state["state"] = MqttStatusSummerTime ? MQTT_STATE_ON : MQTT_STATE_OFF;

      char buffer[256];
      serializeJson(state, buffer);
      const char *topic = concat2(MQTT_CLIENT, "/summer_time");
      MQTTclient.publish(topic, buffer, true);
      LastSentSummerTime = MqttStatusSummerTime;
#ifdef DEBUG_OUTPUT
      Serial.print("TX MQTT: ");
      Serial.print(topic);
      Serial.print(" ");
      Serial.println(buffer);
#endif
    }

    if (force || MqttStatusPulseBpm != LastSentPulseBpm)
    {

      JsonDocument state;
      state["state"] = MqttStatusPulseBpm;

      char buffer[256];
      size_t n = serializeJson(state, buffer);
      const char *topic = concat2(MQTT_CLIENT, "/pulse_bpm");
      MQTTclient.publish(topic, buffer, true);
      LastSentPulseBpm = MqttStatusPulseBpm;

      Serial.print("TX MQTT: ");
      Serial.print(topic);
      Serial.print(" ");
      Serial.println(buffer);
    }

    if (force || MqttStatusBreathBpm != LastSentBreathBpm)
    {

      JsonDocument state;
      state["state"] = MqttStatusBreathBpm;

      char buffer[256];
      size_t n = serializeJson(state, buffer);
      const char *topic = concat2(MQTT_CLIENT, "/breath_bpm");
      MQTTclient.publish(topic, buffer, true);
      LastSentBreathBpm = MqttStatusBreathBpm;

      Serial.print("TX MQTT: ");
      Serial.print(topic);
      Serial.print(" ");
      Serial.println(buffer);
    }

    if (force || MqttStatusRainbowSec != LastSentRainbowSec)
    {

      JsonDocument state;
      state["state"] = round1(MqttStatusRainbowSec);

      char buffer[256];
      serializeJson(state, buffer);
      const char *topic = concat2(MQTT_CLIENT, "/rainbow_duration");
      MQTTclient.publish(topic, buffer, true);
      LastSentRainbowSec = MqttStatusRainbowSec;

      Serial.print("TX MQTT: ");
      Serial.print(topic);
      Serial.print(" ");
      Serial.println(buffer);
    }

    if (force || MqttStatusAlternate != LastSentAlternate) {
        JsonDocument state;
        state["state"] = MqttStatusAlternate ? MQTT_STATE_ON : MQTT_STATE_OFF;

        char buffer[256];
        size_t n = serializeJson(state, buffer);
        const char *topic = concat2(MQTT_CLIENT, "/alternate");
        MQTTclient.publish(topic, buffer, true);
        LastSentAlternate = MqttStatusAlternate;

        Serial.print("TX MQTT: ");
        Serial.print(topic);
        Serial.print(" ");
        Serial.println(buffer);
    }

    // Status reporting for countdown commands
    if (force || MqttStatusCountdownRunning != LastSentCountdownRunning)
    {
      JsonDocument state;
      state["state"] = MqttStatusCountdownRunning ? MQTT_STATE_ON : MQTT_STATE_OFF;

      char buffer[256];
      size_t n = serializeJson(state, buffer);
      const char *topic = concat2(MQTT_CLIENT, "/countdown");
      MQTTclient.publish(topic, buffer, true);
      LastSentCountdownRunning = MqttStatusCountdownRunning;

      Serial.print("TX MQTT: ");
      Serial.print(topic);
      Serial.print(" ");
      Serial.println(buffer);
    }

    // Status reporting for mode
    if (force || strcmp(modeToString(getCurrentMode()), LastSentMode) != 0)
    {
      JsonDocument state;
      state["state"] = modeToString(getCurrentMode());

      char buffer[256];
      size_t n = serializeJson(state, buffer);
      const char *topic = concat2(MQTT_CLIENT, "/mode");
      MQTTclient.publish(topic, buffer, true);

      // Update LastSentMode only if it has changed
      strncpy(LastSentMode, modeToString(getCurrentMode()), sizeof(LastSentMode) - 1);
      LastSentMode[sizeof(LastSentMode) - 1] = '\0'; // Ensure null-termination

      Serial.print("TX MQTT: ");
      Serial.print(topic);
      Serial.print(" ");
      Serial.println(buffer);
    }
  }
#endif
}

void MqttStart()
{
#ifdef MQTT_ENABLED
  MqttConnected = false;
  if (((millis() - LastTimeTriedToConnect) > (MQTT_RECONNECT_WAIT_SEC * 1000)) || (LastTimeTriedToConnect == 0))
  {
    LastTimeTriedToConnect = millis();
    MQTTclient.setServer(MQTT_BROKER, MQTT_PORT);
    MQTTclient.setCallback(callback);
    MQTTclient.setBufferSize(2048);
    MQTTclient.setKeepAlive(30);
    MQTTclient.setSocketTimeout(5);

    Serial.println("");
    Serial.println("Connecting to MQTT...");
    
    // Connect with LWT parameters
    if (MQTTclient.connect(MQTT_CLIENT, MQTT_USERNAME, MQTT_PASSWORD, 
                          MQTT_LWT_TOPIC,  // LWT topic
                          0,               // LWT QoS
                          true,            // LWT retain
                          MQTT_LWT_OFFLINE // LWT offline message
                          ))
    {
      // Publish online status after successful connection
      MQTTclient.publish(MQTT_LWT_TOPIC, MQTT_LWT_ONLINE, true);
      Serial.println("MQTT connected");
      MqttConnected = true;
      mqttDisconnectedSince = 0;
      discoveryReported = false;
      MQTTclient.publish(concat2(MQTT_CLIENT, "/countdown/finished"), "", true);
      MQTTclient.publish(concat2(MQTT_CLIENT, "/countdown/remaining"), "", true);
    }
    else
    {
      if (MQTTclient.state() == 5)
      {
        Serial.println("Error: Connection not allowed by broker, possible reasons:");
        Serial.println("- Device is already online. Wait some seconds until it appears offline");
        Serial.println("- Wrong Username or password. Check credentials");
        Serial.println("- Client Id does not belong to this username, verify ClientId");
      }
      else
      {
        Serial.println("Error: Not possible to connect to Broker!");
        Serial.print("Error code:");
        Serial.println(MQTTclient.state());
      }
      return; // do not continue if not connected
    }

#ifndef MQTT_HOME_ASSISTANT
    char subscribeTopic[100];
    snprintf(subscribeTopic, sizeof(subscribeTopic), "%s/#", MQTT_CLIENT);
    MQTTclient.subscribe(subscribeTopic); // Subscribes to all messages send to the device

    sendToBroker("report/online", "true");                                // Reports that the device is online
    sendToBroker("report/firmware", FIRMWARE_VERSION);                    // Reports the firmware version
    sendToBroker("report/ip", (char *)WiFi.localIP().toString().c_str()); // Reports the ip
    sendToBroker("report/network", (char *)WiFi.SSID().c_str());          // Reports the network name
    MqttReportWiFiSignal();
#endif

#ifdef MQTT_HOME_ASSISTANT
    char subscribeTopic[100];
    snprintf(subscribeTopic, sizeof(subscribeTopic), "%s/main/set", MQTT_CLIENT);
    MQTTclient.subscribe(subscribeTopic);

    snprintf(subscribeTopic, sizeof(subscribeTopic), "%s/back/set", MQTT_CLIENT);
    MQTTclient.subscribe(subscribeTopic);

    snprintf(subscribeTopic, sizeof(subscribeTopic), "%s/use_twelve_hours/set", MQTT_CLIENT);
    MQTTclient.subscribe(subscribeTopic);

    snprintf(subscribeTopic, sizeof(subscribeTopic), "%s/blank_zero_hours/set", MQTT_CLIENT);
    MQTTclient.subscribe(subscribeTopic);

    snprintf(subscribeTopic, sizeof(subscribeTopic), "%s/summer_time/set", MQTT_CLIENT);
    MQTTclient.subscribe(subscribeTopic);

    snprintf(subscribeTopic, sizeof(subscribeTopic), "%s/pulse_bpm/set", MQTT_CLIENT);
    MQTTclient.subscribe(subscribeTopic);

    snprintf(subscribeTopic, sizeof(subscribeTopic), "%s/breath_bpm/set", MQTT_CLIENT);
    MQTTclient.subscribe(subscribeTopic);

    snprintf(subscribeTopic, sizeof(subscribeTopic), "%s/rainbow_duration/set", MQTT_CLIENT);
    MQTTclient.subscribe(subscribeTopic);

    // Subscribe to countdown control topics
    snprintf(subscribeTopic, sizeof(subscribeTopic), "%s/countdown/start", MQTT_CLIENT);
    MQTTclient.subscribe(subscribeTopic);
    
    snprintf(subscribeTopic, sizeof(subscribeTopic), "%s/countdown/stop", MQTT_CLIENT);
    MQTTclient.subscribe(subscribeTopic);

    // Subscribe to the temperature status topic
    snprintf(subscribeTopic, sizeof(subscribeTopic), "saunaBox/temperature");
    MQTTclient.subscribe(subscribeTopic);

    snprintf(subscribeTopic, sizeof(subscribeTopic), "saunaBox/humidity");
    MQTTclient.subscribe(subscribeTopic);

    // Subscribe to mode changes
    snprintf(subscribeTopic, sizeof(subscribeTopic), "%s/mode/set", MQTT_CLIENT);
    MQTTclient.subscribe(subscribeTopic);

    snprintf(subscribeTopic, sizeof(subscribeTopic), "%s/alternate/set", MQTT_CLIENT);
    MQTTclient.subscribe(subscribeTopic);

    snprintf(subscribeTopic, sizeof(subscribeTopic), "saunaBox/power_status", MQTT_CLIENT);
    MQTTclient.subscribe(subscribeTopic);

    snprintf(subscribeTopic, sizeof(subscribeTopic), "saunaBox/temperature_status");
    MQTTclient.subscribe(subscribeTopic);

    snprintf(subscribeTopic, sizeof(subscribeTopic), "saunaBox/humidity_status");
    MQTTclient.subscribe(subscribeTopic);
#endif
  }
#endif
}

int splitCommand(char *topic, char *tokens[], int tokensNumber, char *commandBuf, int commandBufLen)
{
  int mqttClientLength = strlen(MQTT_CLIENT);
  const char *rest = topic;
  if (strncmp(topic, MQTT_CLIENT, mqttClientLength) == 0 && topic[mqttClientLength] == '/')
  {
    rest = topic + mqttClientLength + 1;
  }

  strncpy(commandBuf, rest, commandBufLen - 1);
  commandBuf[commandBufLen - 1] = '\0';

  const char s[2] = "/";
  int pos = 0;
  tokens[0] = strtok(commandBuf, s);
  while (pos < tokensNumber - 1 && tokens[pos] != NULL)
  {
    pos++;
    tokens[pos] = strtok(NULL, s);
  }

  return pos;
}

void checkMqtt()
{
  MqttConnected = MQTTclient.connected();
  if (!MqttConnected)
  {
    if (mqttDisconnectedSince == 0)
    {
      mqttDisconnectedSince = millis();
      Serial.println("MQTT disconnected");
    }
    else if ((millis() - mqttDisconnectedSince) > (MQTT_GIVE_UP_RESTART_SEC * 1000UL))
    {
      Serial.println("MQTT down too long, restarting ESP32");
      delay(100);
      ESP.restart();
    }
    MqttStart();
  }
  else
  {
    mqttDisconnectedSince = 0;
  }
}

void callback(char *topic, byte *payload, unsigned int length)
{ // A new message has been received
#ifdef DEBUG_OUTPUT
  Serial.print("Received MQTT topic: ");
  Serial.print(topic); // long output
#endif
  int commandNumber = 10;
  char *command[10];
  char commandBuf[100];
  commandNumber = splitCommand(topic, command, commandNumber, commandBuf, sizeof(commandBuf));

  char message[length + 1];
  strncpy(message, (char *)payload, length);
  message[length] = '\0';

  if (commandNumber < 1)
  {
    Serial.println("Detected number of commands in MQTT message is lower then 2! -> Ignoring message because it is not valid!");
    return;
  }
#ifdef DEBUG_OUTPUT
  Serial.println();
  Serial.print("RX MQTT: ");
  Serial.print(topic);
  Serial.print(" ");
  Serial.println(message);
  Serial.print("command[0]: ");
  Serial.print(" ");
  Serial.println(command[0]);
  Serial.print("command[1]: ");
  Serial.print(" ");
  Serial.println(command[1]);
#endif

#ifndef MQTT_HOME_ASSISTANT
  //------------------Decide what to do depending on the topic and message---------------------------------
  if (strcmp(command[0], "directive") == 0 && strcmp(command[1], "powerState") == 0)
  { // Turn On or OFF
    if (strcmp(message, "ON") == 0)
    {
      MqttCommandPower = true;
      MqttCommandPowerReceived = true;
    }
    else if (strcmp(message, "OFF") == 0)
    {
      MqttCommandPower = false;
      MqttCommandPowerReceived = true;
    } //      SmartNest:                         // SmartThings
  }
  else if (strcmp(command[0], "directive") == 0 && (strcmp(command[1], "setpoint") == 0) || (strcmp(command[1], "percentage") == 0))
  {
    double valueD = atof(message);
    if (!isnan(valueD))
    {
      MqttCommandState = (int)valueD;
      MqttCommandStateReceived = true;
    }
  }
#endif

#ifdef MQTT_HOME_ASSISTANT
  if (strcmp(command[0], "main") == 0 && strcmp(command[1], "set") == 0)
  {
    JsonDocument doc;
    deserializeJson(doc, payload, length);

    if (doc["state"].is<const char *>())
    {
      MqttCommandMainPower = strcmp(doc["state"], MQTT_STATE_ON) == 0;
      MqttCommandMainPowerReceived = true;
    }
    if (!doc["brightness"].isNull())
    {
      MqttCommandMainBrightness = doc["brightness"].as<int>();
      MqttCommandMainBrightnessReceived = true;
    }
    if (doc["effect"].is<const char *>())
    {
      MqttCommandMainGraphic = tfts.nameToClockFace(doc["effect"]);
      MqttCommandMainGraphicReceived = true;
    }

    doc.clear();
  }
  if (strcmp(command[0], "back") == 0 && strcmp(command[1], "set") == 0)
  {
    JsonDocument doc;
    deserializeJson(doc, payload, length);

    if (doc["state"].is<const char *>())
    {
      MqttCommandBackPower = strcmp(doc["state"], MQTT_STATE_ON) == 0;
      MqttCommandBackPowerReceived = true;
    }
    if (doc["brightness"].is<int>())
    {
      MqttCommandBackBrightness = doc["brightness"];
      MqttCommandBackBrightnessReceived = true;
    }
    if (doc["effect"].is<const char *>())
    {
      strncpy(MqttCommandBackPattern, doc["effect"], sizeof(MqttCommandBackPattern) - 1);
      MqttCommandBackPattern[sizeof(MqttCommandBackPattern) - 1] = '\0';
      MqttCommandBackPatternReceived = true;
    }
    if (doc["color"].is<JsonObject>())
    {
      MqttCommandBackColorPhase = backlights.hueToPhase(doc["color"]["h"]);
      MqttCommandBackColorPhaseReceived = true;
    }
    doc.clear();
  }
  if (strcmp(command[0], "use_twelve_hours") == 0 && strcmp(command[1], "set") == 0)
  {
    JsonDocument doc;
    deserializeJson(doc, payload, length);

    if (doc["state"].is<const char *>())
    {
      MqttCommandUseTwelveHours = strcmp(doc["state"], MQTT_STATE_ON) == 0;
      MqttCommandUseTwelveHoursReceived = true;
    }

    doc.clear();
  }
  if (strcmp(command[0], "summer_time") == 0 && commandNumber >= 2 && command[1] &&
      strcmp(command[1], "set") == 0)
  {
    JsonDocument doc;
    deserializeJson(doc, payload, length);

    if (doc["state"].is<const char *>())
    {
      MqttCommandSummerTime = strcmp(doc["state"], MQTT_STATE_ON) == 0;
      MqttCommandSummerTimeReceived = true;
    }

    doc.clear();
  }
  if (strcmp(command[0], "blank_zero_hours") == 0 && strcmp(command[1], "set") == 0)
  {
    JsonDocument doc;
    deserializeJson(doc, payload, length);

    if (doc["state"].is<const char *>())
    {
      MqttCommandBlankZeroHours = strcmp(doc["state"], MQTT_STATE_ON) == 0;
      MqttCommandBlankZeroHoursReceived = true;
    }

    doc.clear();
  }
  if (strcmp(command[0], "pulse_bpm") == 0 && strcmp(command[1], "set") == 0)
  {
    JsonDocument doc;
    deserializeJson(doc, payload, length);

    if (doc["state"].is<uint8_t>())
    {
      MqttCommandPulseBpm = doc["state"];
      MqttCommandPulseBpmReceived = true;
    }

    doc.clear();
  }
  if (strcmp(command[0], "breath_bpm") == 0 && strcmp(command[1], "set") == 0)
  {
    JsonDocument doc;
    deserializeJson(doc, payload, length);

    if (doc["state"].is<uint8_t>())
    {
      MqttCommandBreathBpm = doc["state"];
      MqttCommandBreathBpmReceived = true;
    }

    doc.clear();
  }
  if (strcmp(command[0], "rainbow_duration") == 0 && strcmp(command[1], "set") == 0)
  {
    JsonDocument doc;
    deserializeJson(doc, payload, length);

    if (doc["state"].is<float>())
    {
      MqttCommandRainbowSec = doc["state"];
      MqttCommandRainbowSecReceived = true;
    }
    doc.clear();
  }

  // Add new countdown commands handling
  if (strcmp(command[0], "countdown") == 0) {
      if (strcmp(command[1], "start") == 0) {
          uint32_t duration = atoi(message);
          if (duration > 0) {  // Only start if we have a valid duration
              MqttCommandCountdownDuration = duration * 60;   // Convert from minutes to seconds
              MqttCommandCountdownStart = true;
              MqttCommandCountdownStartReceived = true;
              Serial.print("Starting countdown with duration: ");
              Serial.println(duration);
          }
      }
      else if (strcmp(command[1], "stop") == 0) {
          MqttCommandCountdownStop = true;
          MqttCommandCountdownStopReceived = true;
      }
  }

  // saunaBox topics are not under IPSTUBE/; match the full topic (and the
  // old "temperature"/"humidity" first-token form from the previous splitter).
  if (strcmp(topic, "saunaBox/temperature") == 0 ||
      strcmp(command[0], "temperature") == 0 ||
      (commandNumber >= 2 && command[1] && strcmp(command[0], "saunaBox") == 0 &&
       strcmp(command[1], "temperature") == 0)) {
      MqttCommandTemperature = atof(message);
      MqttCommandTemperatureReceived = true;
  } else if (strcmp(topic, "saunaBox/humidity") == 0 ||
             strcmp(command[0], "humidity") == 0 ||
             (commandNumber >= 2 && command[1] && strcmp(command[0], "saunaBox") == 0 &&
              strcmp(command[1], "humidity") == 0)) {
      MqttCommandHumidity = atof(message);
      MqttCommandHumidityReceived = true;
  }


  // Mode handling
  if (strcmp(command[0], "mode") == 0 && strcmp(command[1], "set") == 0) {
      strncpy(MqttCommandMode, message, sizeof(MqttCommandMode) - 1);
      MqttCommandMode[sizeof(MqttCommandMode) - 1] = '\0'; // Ensure null-termination
      MqttCommandModeReceived = true;
  }

  if (strcmp(command[0], "alternate") == 0 && strcmp(command[1], "set") == 0) {
      JsonDocument doc;
      deserializeJson(doc, payload, length);

      if (doc["state"].is<const char *>()) {
          MqttCommandAlternate = strcmp(doc["state"], MQTT_STATE_ON) == 0;
          MqttCommandAlternateReceived = true;
      }
      doc.clear();
  }

  if (strcmp(topic, "saunaBox/power_status") == 0) {
      MqttCommandSaunaPower = strcmp(message, "ON") == 0;
      MqttCommandSaunaPowerReceived = true;
  }

  if (strcmp(topic, "saunaBox/temperature_status") == 0) {
      MqttTemperatureSensorOnline = (strcmp(message, "ONLINE") == 0);
  } else if (strcmp(topic, "saunaBox/humidity_status") == 0) {
      MqttHumiditySensorOnline = (strcmp(message, "ONLINE") == 0);
  }

  // When reconnecting, publish online status
  if (strcmp(topic, MQTT_LWT_TOPIC) == 0) {
      MQTTclient.publish(MQTT_LWT_TOPIC, MQTT_LWT_ONLINE, true);
  }
#endif
}

void MqttPublishCountdownRemaining()
{
#ifdef MQTT_ENABLED
  if (!MQTTclient.connected())
  {
    return;
  }
  if (MqttStatusCountdownRemaining == LastSentCountdownRemaining)
  {
    return;
  }
  char countdownBuffer[10];
  uint32_t minutes = MqttStatusCountdownRemaining / 60;
  uint32_t seconds = MqttStatusCountdownRemaining % 60;
  snprintf(countdownBuffer, sizeof(countdownBuffer), "%02d:%02d", minutes, seconds);
  const char *countdownTopic = concat2(MQTT_CLIENT, "/countdown/remaining");
  MQTTclient.publish(countdownTopic, countdownBuffer, false);
  LastSentCountdownRemaining = MqttStatusCountdownRemaining;
#ifdef DEBUG_OUTPUT
  Serial.print("TX MQTT: ");
  Serial.print(countdownTopic);
  Serial.print(" ");
  Serial.println(countdownBuffer);
#endif
#endif
}

void MqttLoopFrequently()
{
#ifdef MQTT_ENABLED
  MQTTclient.loop();
  checkMqtt();
  MqttPublishCountdownRemaining();
  MqttReportBackOnChange();
#endif
}

void MqttLoopInFreeTime()
{
#ifdef MQTT_ENABLED
  MqttPeriodicReportBack();
#endif
}

void MqttReportBattery()
{
  char message[5];
  snprintf(message, sizeof(message), "%d", MqttStatusBattery);
  sendToBroker("report/battery", message);
}

void MqttReportStatus()
{
  if (LastSentStatus != MqttStatusState)
  {
    char message[5];
    snprintf(message, sizeof(message), "%d", MqttStatusState);
    sendToBroker("report/setpoint", message);
    LastSentStatus = MqttStatusState;
  }
}

void MqttReportTemperature()
{
#ifdef ONE_WIRE_BUS_PIN
  if (fTemperature > -30)
  { // transmit data to MQTT only if data is valid
    sendToBroker("report/temperature", sTemperatureTxt);
  }
#endif
}

void MqttReportPowerState()
{
  if (MqttStatusPower != LastSentPowerState)
  {
    sendToBroker("report/powerState", MqttStatusPower == 0 ? MQTT_STATE_OFF : MQTT_STATE_ON);

    LastSentPowerState = MqttStatusPower;
  }
}

void MqttReportWiFiSignal()
{
  char signal[5];
  int SignalLevel = WiFi.RSSI();
  // ignore deviations smaller than 3 dBm
  if (abs(SignalLevel - LastSentSignalLevel) > 2)
  {
    snprintf(signal, sizeof(signal), "%d", SignalLevel);
    sendToBroker("report/signal", signal); // Reports the signal strength
    LastSentSignalLevel = SignalLevel;
  }
}

void MqttReportNotification(String message)
{
  int i;
  byte NotificationChecksum = 0;
  for (i = 0; i < message.length(); i++)
  {
    NotificationChecksum += byte(message[i]);
  }
  // send only different notification, do not re-send same notifications!
  if (NotificationChecksum != LastNotificationChecksum)
  {
    // string to char array
    char msg2[message.length() + 1];
    strncpy(msg2, message.c_str(), sizeof(msg2) - 1);
    sendToBroker("report/notification", msg2);
    LastNotificationChecksum = NotificationChecksum;
  }
}

void MqttReportGraphic(bool force)
{
  if (force || MqttStatusGraphic != LastSentGraphic)
  {
    char graphic[3]; // Increased size to accommodate null terminator
    snprintf(graphic, sizeof(graphic), "%i", MqttStatusGraphic);
    sendToBroker("graphic", graphic); // Reports the signal strength

    LastSentGraphic = MqttStatusGraphic;
  }
}

void MqttReportBackEverything(bool force)
{
  if (MQTTclient.connected())
  {
#ifndef MQTT_HOME_ASSISTANT
    MqttReportPowerState();
    MqttReportStatus();
    // MqttReportBattery();
    MqttReportWiFiSignal();
    MqttReportTemperature();
#endif

#ifdef MQTT_HOME_ASSISTANT
    MqttReportState(force);
#endif

    lastTimeSent = millis();
  }
}

void MqttReportDiscovery()
{
#ifdef MQTT_HOME_ASSISTANT_DISCOVERY
  JsonDocument discovery;
  char uid[64];
  char topic[160];
  char state_topic[96];
  char cmd_topic[112];
  char json_buffer[2048];

  auto fillCommon = [&](const char *unique_suffix, const char *name) {
    discovery["availability"][0]["topic"] = MQTT_LWT_TOPIC;
    discovery["availability"][0]["payload_available"] = MQTT_LWT_ONLINE;
    discovery["availability"][0]["payload_not_available"] = MQTT_LWT_OFFLINE;
    discovery["device"]["identifiers"][0] = MQTT_CLIENT;
    discovery["device"]["manufacturer"] = MQTT_HOME_ASSISTANT_DISCOVERY_DEVICE_MANUFACTURER;
    discovery["device"]["model"] = MQTT_HOME_ASSISTANT_DISCOVERY_DEVICE_MODEL;
    discovery["device"]["name"] = MQTT_CLIENT;
    discovery["device"]["sw_version"] = MQTT_HOME_ASSISTANT_DISCOVERY_SW_VERSION;
    discovery["device"]["hw_version"] = MQTT_HOME_ASSISTANT_DISCOVERY_HW_VERSION;
    discovery["device"]["connections"][0][0] = "mac";
    discovery["device"]["connections"][0][1] = WiFi.macAddress();
    snprintf(uid, sizeof(uid), "%s%s", MQTT_CLIENT, unique_suffix);
    discovery["unique_id"] = uid;
    discovery["object_id"] = uid;
    discovery["name"] = name;
  };

  auto publishDiscovery = [&](const char *component, const char *object_id) {
    snprintf(topic, sizeof(topic), "homeassistant/%s/%s/%s/config", component, MQTT_CLIENT, object_id);
    serializeJson(discovery, json_buffer, sizeof(json_buffer));
    MQTTclient.publish(topic, json_buffer, true);
    MQTTclient.loop();
#ifdef DEBUG_OUTPUT
    Serial.print("TX MQTT: ");
    Serial.print(topic);
    Serial.print(" ");
    Serial.println(json_buffer);
#endif
  };

  static const char *kObsoleteDiscovery[] = {
      "homeassistant/light/IPSTUBE_main/light/config",
      "homeassistant/light/IPSTUBE_back/light/config",
      "homeassistant/switch/IPSTUBE_use_twelve_hours/switch/config",
      "homeassistant/switch/IPSTUBE_blank_zero_hours/switch/config",
      "homeassistant/switch/IPSTUBE_countdown/switch/config",
      "homeassistant/switch/IPSTUBE_alternate/switch/config",
      "homeassistant/switch/IPSTUBE_countdown_start/switch/config",
      "homeassistant/switch/IPSTUBE_countdown_stop/switch/config",
      "homeassistant/number/IPSTUBE_pulse_bpm/number/config",
      "homeassistant/number/IPSTUBE_breath_bpm/number/config",
      "homeassistant/number/IPSTUBE_rainbow_duration/number/config",
      "homeassistant/number/IPSTUBE_countdown/number/config",
      "homeassistant/select/IPSTUBE_mode_set/select/config",
  };
  for (size_t i = 0; i < sizeof(kObsoleteDiscovery) / sizeof(kObsoleteDiscovery[0]); i++)
  {
    MQTTclient.publish(kObsoleteDiscovery[i], "", true);
  }

  discovery.clear();
  fillCommon("_main", "Main");
  discovery["schema"] = "json";
  discovery["state_topic"] = concat2(MQTT_CLIENT, "/main");
  discovery["json_attributes_topic"] = concat2(MQTT_CLIENT, "/main");
  discovery["command_topic"] = concat2(MQTT_CLIENT, "/main/set");
  discovery["brightness"] = true;
  discovery["brightness_scale"] = 255;
  discovery["effect"] = true;
  for (uint8_t i = 1; i <= tfts.NumberOfClockFaces; i++)
  {
    discovery["effect_list"][i - 1] = tfts.clockFaceToName(i);
  }
  publishDiscovery("light", "main");

  discovery.clear();
  fillCommon("_back", "Back");
  discovery["schema"] = "json";
  discovery["state_topic"] = concat2(MQTT_CLIENT, "/back");
  discovery["json_attributes_topic"] = concat2(MQTT_CLIENT, "/back");
  discovery["command_topic"] = concat2(MQTT_CLIENT, "/back/set");
  discovery["brightness"] = true;
  discovery["brightness_scale"] = 7;
  discovery["effect"] = true;
  for (int i = 0; i < Backlights::num_patterns; i++)
  {
    discovery["effect_list"][i] = backlights.patterns_str[i];
  }
  discovery["supported_color_modes"][0] = "hs";
  publishDiscovery("light", "back");

  struct SwitchEnt { const char *suffix; const char *oid; const char *name; const char *path; };
  const SwitchEnt switchList[] = {
      {"_use_twelve_hours", "use_twelve_hours", "Use Twelve Hours", "/use_twelve_hours"},
      {"_blank_zero_hours", "blank_zero_hours", "Blank Zero Hours", "/blank_zero_hours"},
      {"_summer_time", "summer_time", "Summer Time", "/summer_time"},
      {"_countdown", "countdown", "Countdown", "/countdown"},
      {"_alternate", "alternate", "Alternate Mode", "/alternate"},
  };
  for (size_t i = 0; i < sizeof(switchList) / sizeof(switchList[0]); i++)
  {
    const SwitchEnt &sw = switchList[i];
    discovery.clear();
    fillCommon(sw.suffix, sw.name);
    discovery["entity_category"] = "config";
    snprintf(state_topic, sizeof(state_topic), "%s%s", MQTT_CLIENT, sw.path);
    snprintf(cmd_topic, sizeof(cmd_topic), "%s%s/set", MQTT_CLIENT, sw.path);
    discovery["state_topic"] = state_topic;
    discovery["json_attributes_topic"] = state_topic;
    discovery["command_topic"] = cmd_topic;
    discovery["value_template"] = "{{ value_json.state }}";
    discovery["state_on"] = "ON";
    discovery["state_off"] = "OFF";
    discovery["payload_on"] = "{\"state\":\"ON\"}";
    discovery["payload_off"] = "{\"state\":\"OFF\"}";
    publishDiscovery("switch", sw.oid);
  }

  struct NumEnt { const char *suffix; const char *oid; const char *name; const char *path; float minv; float maxv; float step; };
  const NumEnt numList[] = {
      {"_pulse_bpm", "pulse_bpm", "Pulse, bpm", "/pulse_bpm", 20, 120, 1},
      {"_breath_bpm", "breath_bpm", "Breath, bpm", "/breath_bpm", 5, 60, 1},
      {"_rainbow_duration", "rainbow_duration", "Rainbow, sec", "/rainbow_duration", 0.2f, 10, 0.1f},
  };
  for (size_t i = 0; i < sizeof(numList) / sizeof(numList[0]); i++)
  {
    const NumEnt &nm = numList[i];
    discovery.clear();
    fillCommon(nm.suffix, nm.name);
    discovery["entity_category"] = "config";
    snprintf(state_topic, sizeof(state_topic), "%s%s", MQTT_CLIENT, nm.path);
    snprintf(cmd_topic, sizeof(cmd_topic), "%s%s/set", MQTT_CLIENT, nm.path);
    discovery["state_topic"] = state_topic;
    discovery["json_attributes_topic"] = state_topic;
    discovery["command_topic"] = cmd_topic;
    discovery["command_template"] = "{\"state\":{{value}}}";
    discovery["step"] = nm.step;
    discovery["min"] = nm.minv;
    discovery["max"] = nm.maxv;
    discovery["mode"] = "slider";
    discovery["value_template"] = "{{ value_json.state }}";
    publishDiscovery("number", nm.oid);
  }

  discovery.clear();
  fillCommon("_mode", "Mode");
  discovery["state_topic"] = concat2(MQTT_CLIENT, "/mode");
  discovery["command_topic"] = concat2(MQTT_CLIENT, "/mode/set");
  discovery["value_template"] = "{{ value_json.state }}";
  discovery["options"][0] = "clock";
  discovery["options"][1] = "sensor_display";
  discovery["options"][2] = "countdown";
  publishDiscovery("select", "mode");

  discovery.clear();
  fillCommon("_countdown_start", "Countdown Start");
  discovery["command_topic"] = concat2(MQTT_CLIENT, "/countdown/start");
  discovery["min"] = 1;
  discovery["max"] = 90;
  discovery["step"] = 1;
  discovery["mode"] = "box";
  discovery["optimistic"] = true;
  discovery["unit_of_measurement"] = "min";
  publishDiscovery("number", "countdown_start");

  discovery.clear();
  fillCommon("_countdown_stop", "Countdown Stop");
  discovery["command_topic"] = concat2(MQTT_CLIENT, "/countdown/stop");
  discovery["payload_press"] = "ON";
  discovery["entity_category"] = "config";
  publishDiscovery("button", "countdown_stop");

  discovery.clear();
  fillCommon("_countdown_remaining", "Countdown Remaining");
  discovery["state_topic"] = concat2(MQTT_CLIENT, "/countdown/remaining");
  discovery["icon"] = "mdi:timer";
  publishDiscovery("sensor", "countdown_remaining");

  discovery.clear();
  fillCommon("_countdown_finished", "Countdown Finished");
  discovery["state_topic"] = concat2(MQTT_CLIENT, "/countdown/finished");
  discovery["payload_on"] = "Countdown has finished";
  discovery["payload_off"] = "OFF";
  publishDiscovery("binary_sensor", "countdown_finished");
#endif
}

void MqttReportBackOnChange()
{
  if (MQTTclient.connected())
  {
#ifndef MQTT_HOME_ASSISTANT
    MqttReportPowerState();
    MqttReportStatus();
#endif
#ifdef MQTT_HOME_ASSISTANT_DISCOVERY
    if (!discoveryReported)
    {
      MqttReportDiscovery();
      discoveryReported = true;
    }
#endif
#ifdef MQTT_HOME_ASSISTANT
    MqttReportState(false);
#endif
  }
}

void MqttPeriodicReportBack()
{
  if (((millis() - lastTimeSent) > (MQTT_REPORT_STATUS_EVERY_SEC * 1000)) && MQTTclient.connected())
  {
#ifdef MQTT_HOME_ASSISTANT_DISCOVERY
    if (!discoveryReported)
    {
      MqttReportDiscovery();
      discoveryReported = true;
    }
#endif
    // Publish LWT status
    MQTTclient.publish(MQTT_LWT_TOPIC, MQTT_LWT_ONLINE, true);
    
    MqttReportBackEverything(true);
    lastTimeSent = millis();
  }
}

void setMqttCommandMode(const char* mode) {
    strncpy(MqttCommandMode, mode, sizeof(MqttCommandMode) - 1);
    MqttCommandMode[sizeof(MqttCommandMode) - 1] = '\0'; // Ensure null-termination
}

void MqttSendCountdownFinished() {
    if (MQTTclient.connected()) {
        const char* topic = concat2(MQTT_CLIENT, "/countdown/finished");
        MQTTclient.publish(topic, "Countdown has finished", false);
#ifdef DEBUG_OUTPUT
        Serial.print("Sent MQTT message: ");
        Serial.print(topic);
        Serial.println(" - Countdown has finished");
#endif
    } else {
        Serial.println("MQTT not connected, cannot send countdown finished message.");
    }
}