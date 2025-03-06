/*
 * Author: Aljaz Ogrin
 * Project: Alternative firmware for EleksTube IPS clock
 * Original location: https://github.com/aly-fly/EleksTubeHAX
 * Hardware: ESP32
 * Based on: https://github.com/SmittyHalibut/EleksTubeHAX
 */

#include <stdint.h>
#include "GLOBAL_DEFINES.h"
#include "Buttons.h"
#include "Backlights.h"
#include "TFTs.h"
#include "Clock.h"
#include "Menu.h"
#include "StoredConfig.h"
#include "WiFi_WPS.h"
#include "Mqtt_client_ips.h"
#include "TempSensor_inc.h"
#ifdef HARDWARE_NovelLife_SE_CLOCK // NovelLife_SE Clone XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// #include "Gestures.h"
// TODO put into class
#include <Wire.h>
#include <SparkFun_APDS9960.h>
#endif // NovelLife_SE Clone XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
#include "main.h"

// Constants

// Global Variables
#ifdef HARDWARE_NovelLife_SE_CLOCK // NovelLife_SE Clone XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// TODO put into class
SparkFun_APDS9960 apds = SparkFun_APDS9960();
// interupt signal for gesture sensor
int volatile isr_flag = 0;
#endif // NovelLife_SE Clone XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

Backlights backlights;
Buttons buttons;
TFTs tfts;
Clock uclock;
Menu menu;
StoredConfig stored_config;

#ifdef DIMMING
bool isDimmingNeeded = false;
uint8_t hour_old = 255;
#endif
bool DstNeedsUpdate = false;
uint8_t yesterday = 0;

uint32_t lastMqttCommandExecuted = (uint32_t)-1;

unsigned long countdownFinishTime = 0;
bool countdownFinished = false;
bool countdownHandled = false;

bool alternateMode = true; // Turn on alternate mode switching by default
unsigned long lastModeSwitch = 0;
const unsigned long MODE_SWITCH_INTERVAL = 20000; // 20 seconds in milliseconds

Mode currentMode = SENSOR_DISPLAY; // Default mode

void setupMenu(void);
#ifdef DIMMING
bool isNightTime(uint8_t current_hour);
void checkDimmingNeeded(void);
#endif
void UpdateDstEveryNight(void);
#ifdef HARDWARE_NovelLife_SE_CLOCK // NovelLife_SE Clone XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
void GestureStart();
void HandleGestureInterupt(void);   // only for NovelLife SE
void GestureInterruptRoutine(void); // only for NovelLife SE
void HandleGesture(void);           // only for NovelLife SE
#endif                              // NovelLife_SE Clone XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

void setup()
{
  Serial.begin(115200);
  delay(1000); // Waiting for serial monitor to catch up.
  Serial.println("");
  Serial.println(FIRMWARE_VERSION);
  Serial.println("In setup().");

  stored_config.begin();
  stored_config.load();

  backlights.begin(&stored_config.config.backlights);
  buttons.begin();
  menu.begin();

  // Setup the displays (TFTs) initaly and show bootup message(s)
  tfts.begin();
  tfts.fillScreen(TFT_BLACK);
  tfts.setTextColor(TFT_WHITE, TFT_BLACK);
  tfts.setCursor(0, 0, 2); // Font 2. 16 pixel high
  tfts.println("Starting Setup...");

#ifdef HARDWARE_NovelLife_SE_CLOCK // NovelLife_SE Clone XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
  // Init the Gesture sensor
  tfts.setTextColor(TFT_ORANGE, TFT_BLACK);
  tfts.print("Gest start...");
  Serial.print("Gesture Sensor start...");
  GestureStart(); // TODO put into class
  tfts.println("Done!");
  Serial.println("Done!");
  tfts.setTextColor(TFT_WHITE, TFT_BLACK);
#endif

  // Setup WiFi connection. Must be done before setting up Clock.
  // This is done outside Clock so the network can be used for other things.
  tfts.setTextColor(TFT_DARKGREEN, TFT_BLACK);
  tfts.println("WiFi start...");
  Serial.println("WiFi start...");
  WifiBegin();
  tfts.setTextColor(TFT_WHITE, TFT_BLACK);

  // wait a bit (5x100ms = 0.5 sec) before querying NTP
  for (uint8_t ndx = 0; ndx < 5; ndx++)
  {
    tfts.print(">");
    delay(100);
  }
  tfts.println("");

  // Setup the clock.  It needs WiFi to be established already.
  tfts.setTextColor(TFT_MAGENTA, TFT_BLACK);
  tfts.print("Clock start...");
  Serial.print("Clock start...");
  uclock.begin(&stored_config.config.uclock);
  tfts.println("Done!");
  Serial.println("Done!");
  tfts.setTextColor(TFT_WHITE, TFT_BLACK);

  // Setup MQTT
  tfts.setTextColor(TFT_YELLOW, TFT_BLACK);
  tfts.print("MQTT start...");
  Serial.print("MQTT start...");
  MqttStart();
  tfts.println("Done!");
  Serial.println("Done!");
  tfts.setTextColor(TFT_WHITE, TFT_BLACK);

#ifdef GEOLOCATION_ENABLED
  tfts.setTextColor(TFT_NAVY, TFT_BLACK);
  tfts.println("GeoLoc query...");
  Serial.println("GeoLoc query...");
  if (GetGeoLocationTimeZoneOffset())
  {
    tfts.print("TZ: ");
    Serial.print("TZ: ");
    tfts.println(GeoLocTZoffset);
    Serial.println(GeoLocTZoffset);
    uclock.setTimeZoneOffset(GeoLocTZoffset * 3600);
    Serial.println();
    Serial.print("Saving config! Triggerd by timezone change...");
    stored_config.save();
    tfts.println("Done!");
    Serial.println("Done!");
    tfts.setTextColor(TFT_WHITE, TFT_BLACK);
  }
  else
  {
    tfts.setTextColor(TFT_RED, TFT_BLACK);
    tfts.println("GeoLoc FAILED");
    Serial.println("GeoLoc failed!");
    tfts.setTextColor(TFT_WHITE, TFT_BLACK);
  }
#endif

  if (uclock.getActiveGraphicIdx() > tfts.NumberOfClockFaces)
  {
    uclock.setActiveGraphicIdx(tfts.NumberOfClockFaces);
    Serial.println("Last selected index of clock face is larger than currently available number of image sets.");
  }
  if (uclock.getActiveGraphicIdx() < 1)
  {
    uclock.setActiveGraphicIdx(1);
    Serial.println("Last selected index of clock face is less than 1.");
  }
  tfts.current_graphic = uclock.getActiveGraphicIdx();

  tfts.setTextColor(TFT_WHITE, TFT_BLACK);
  tfts.println("Done with Setup!");
  Serial.println("Done with Setup!");

  // Leave boot up messages on screen for a few seconds (10x200ms = 2 sec)
  for (uint8_t ndx = 0; ndx < 10; ndx++)
  {
    tfts.print(">");
    delay(200);
  }

  // Start up the clock displays.
  tfts.fillScreen(TFT_BLACK);
  uclock.loop();
  updateDisplay(TFTs::force); // Draw all the clock digits
  Serial.println("Setup finished.");

  // Set alternate mode to true by default
  alternateMode = true;
}

void loop()
{
  uint32_t millis_at_top = millis();
  // Do all the maintenance work
  WifiReconnect(); // if not connected attempt to reconnect

  MqttLoopFrequently();

  bool MqttCommandReceived =
      MqttCommandPowerReceived ||
      MqttCommandMainPowerReceived ||
      MqttCommandBackPowerReceived ||
      MqttCommandStateReceived ||
      MqttCommandBrightnessReceived ||
      MqttCommandMainBrightnessReceived ||
      MqttCommandBackBrightnessReceived ||
      MqttCommandPatternReceived ||
      MqttCommandBackPatternReceived ||
      MqttCommandBackColorPhaseReceived ||
      MqttCommandGraphicReceived ||
      MqttCommandMainGraphicReceived ||
      MqttCommandUseTwelveHoursReceived ||
      MqttCommandBlankZeroHoursReceived ||
      MqttCommandPulseBpmReceived ||
      MqttCommandBreathBpmReceived ||
      MqttCommandRainbowSecReceived ||
      MqttCommandCountdownStartReceived ||
      MqttCommandCountdownStopReceived ||
      MqttCommandModeReceived ||
      MqttCommandAlternateReceived ||
      MqttCommandSaunaPowerReceived;

  if (MqttCommandPowerReceived)
  {
    MqttCommandPowerReceived = false;
    if (MqttCommandPower)
    {
#ifndef HARDWARE_SI_HAI_CLOCK
      if (!tfts.isEnabled())
      {
        tfts.reinit(); // reinit (original EleksTube HW: after a few hours in OFF state the displays do not wake up properly)
        updateDisplay(TFTs::force);
      }
#endif
      tfts.enableAllDisplays();
      backlights.PowerOn();
    }
    else
    {
      tfts.disableAllDisplays();
      backlights.PowerOff();
    }
  }

  if (MqttCommandMainPowerReceived)
  {
    MqttCommandMainPowerReceived = false;
    if (MqttCommandMainPower)
    {
#ifndef HARDWARE_SI_HAI_CLOCK
      if (!tfts.isEnabled())
      {
        tfts.reinit(); // reinit (original EleksTube HW: after a few hours in OFF state the displays do not wake up properly)
        updateDisplay(TFTs::force);
      }
#endif
      tfts.enableAllDisplays();
    }
    else
    {
      tfts.disableAllDisplays();
    }
  }

  if (MqttCommandBackPowerReceived)
  {
    MqttCommandBackPowerReceived = false;
    if (MqttCommandBackPower)
    {
      backlights.PowerOn();
    }
    else
    {
      backlights.PowerOff();
    }
  }

  if (MqttCommandStateReceived)
  {
    MqttCommandStateReceived = false;
    randomSeed(millis());
    uint8_t idx;
    if (MqttCommandState >= 90)
    {
      idx = random(1, tfts.NumberOfClockFaces + 1);
    }
    else
    {
      idx = (MqttCommandState / 5) - 1;
    } // 10..40 -> graphic 1..6
    Serial.print("Graphic change request from MQTT; command: ");
    Serial.print(MqttCommandState);
    Serial.print(", index: ");
    Serial.println(idx);
    uclock.setClockGraphicsIdx(idx);
    tfts.current_graphic = uclock.getActiveGraphicIdx();
    updateDisplay(TFTs::force); // redraw everything
  }

  if (MqttCommandMainBrightnessReceived)
  {
    MqttCommandMainBrightnessReceived = false;
    tfts.dimming = MqttCommandMainBrightness;
    tfts.ProcessUpdatedDimming();
    updateDisplay(TFTs::force);
  }

  if (MqttCommandBackBrightnessReceived)
  {
    MqttCommandBackBrightnessReceived = false;
    backlights.setIntensity(uint8_t(MqttCommandBackBrightness));
  }

  if (MqttCommandPatternReceived)
  {
    MqttCommandPatternReceived = false;

    for (int8_t i = 0; i < Backlights::num_patterns; i++)
    {
      Serial.print("New pattern ");
      Serial.print(MqttCommandPattern);
      Serial.print(", check pattern ");
      Serial.println(Backlights::patterns_str[i]);
      if (strcmp(MqttCommandPattern, (Backlights::patterns_str[i]).c_str()) == 0)
      {
        backlights.setPattern(Backlights::patterns(i));
        break;
      }
    }
  }

  if (MqttCommandBackPatternReceived)
  {
    MqttCommandBackPatternReceived = false;
    for (int8_t i = 0; i < Backlights::num_patterns; i++)
    {
      Serial.print("new pattern ");
      Serial.print(MqttCommandBackPattern);
      Serial.print(", check pattern ");
      Serial.println(Backlights::patterns_str[i]);
      if (strcmp(MqttCommandBackPattern, (Backlights::patterns_str[i]).c_str()) == 0)
      {
        backlights.setPattern(Backlights::patterns(i));
        break;
      }
    }
  }

  if (MqttCommandBackColorPhaseReceived)
  {
    MqttCommandBackColorPhaseReceived = false;

    backlights.setColorPhase(MqttCommandBackColorPhase);
  }

  if (MqttCommandGraphicReceived)
  {
    MqttCommandGraphicReceived = false;

    uclock.setClockGraphicsIdx(MqttCommandGraphic);
    tfts.current_graphic = uclock.getActiveGraphicIdx();
    updateDisplay(TFTs::force); // redraw everything
  }

  if (MqttCommandMainGraphicReceived)
  {
    MqttCommandMainGraphicReceived = false;
    uclock.setClockGraphicsIdx(MqttCommandMainGraphic);
    tfts.current_graphic = uclock.getActiveGraphicIdx();
    updateDisplay(TFTs::force); // redraw everything
  }

  if (MqttCommandUseTwelveHoursReceived)
  {
    MqttCommandUseTwelveHoursReceived = false;
    uclock.setTwelveHour(MqttCommandUseTwelveHours);
  }

  if (MqttCommandBlankZeroHoursReceived)
  {
    MqttCommandBlankZeroHoursReceived = false;
    uclock.setBlankHoursZero(MqttCommandBlankZeroHours);
  }

  if (MqttCommandPulseBpmReceived)
  {
    MqttCommandPulseBpmReceived = false;
    backlights.setPulseRate(MqttCommandPulseBpm);
  }

  if (MqttCommandBreathBpmReceived)
  {
    MqttCommandBreathBpmReceived = false;
    backlights.setBreathRate(MqttCommandBreathBpm);
  }

  if (MqttCommandRainbowSecReceived)
  {
    MqttCommandRainbowSecReceived = false;
    backlights.setRainbowDuration(MqttCommandRainbowSec);
  }

  if (MqttCommandCountdownStartReceived)
  {
    MqttCommandCountdownStartReceived = false;
    uclock.startCountdown(MqttCommandCountdownDuration);
    countdownHandled = false; // Reset the flag
    currentMode = COUNTDOWN;
    
    // Stop any flashing background effect by setting it to constant
    backlights.setPattern(Backlights::dark);
  }
  if (MqttCommandCountdownStopReceived)
  {
    uclock.stopCountdown();
    MqttCommandCountdownStopReceived = false;
  }

  if (MqttCommandModeReceived) {
    if (strcmp(MqttCommandMode, "clock") == 0) {
        currentMode = CLOCK;
    } else if (strcmp(MqttCommandMode, "countdown") == 0) {
        currentMode = COUNTDOWN;
    } else if (strcmp(MqttCommandMode, "sensor_display") == 0) {
        currentMode = SENSOR_DISPLAY;
    }
    MqttCommandModeReceived = false;
    updateDisplay(TFTs::force);
  }

  if (MqttCommandAlternateReceived) {
    MqttCommandAlternateReceived = false;
    alternateMode = MqttCommandAlternate;
  }

  if (MqttCommandTemperatureReceived) {
    MqttCommandTemperatureReceived = false;
    // Handle the received temperature value
    //updateDisplay(TFTs::show_t::yes);
  }

  if (MqttCommandHumidityReceived) {
    MqttCommandHumidityReceived = false;
    // Handle the received humidity value
    //updateDisplay(TFTs::show_t::yes);
  }

  if (MqttCommandSaunaPowerReceived) {
    MqttCommandSaunaPowerReceived = false;
    if (MqttCommandSaunaPower) {
        // Sauna is ON - set orange effect
        uclock.setClockGraphicsIdx(tfts.nameToClockFace("Hui Orange"));
    } else {
        // Sauna is OFF - set blue effect
        uclock.setClockGraphicsIdx(tfts.nameToClockFace("Hui Blue"));
    }
    tfts.current_graphic = uclock.getActiveGraphicIdx();
    updateDisplay(TFTs::force);
  }

  MqttStatusPower = tfts.isEnabled();
  MqttStatusMainPower = tfts.isEnabled();
  MqttStatusBackPower = backlights.getPower();
  MqttStatusState = (uclock.getActiveGraphicIdx() + 1) * 5; // 10
  MqttStatusBrightness = backlights.getIntensity();
  MqttStatusMainBrightness = tfts.dimming;
  MqttStatusBackBrightness = backlights.getIntensity();
  strcpy(MqttStatusPattern, backlights.getPatternStr().c_str());
  strcpy(MqttStatusBackPattern, backlights.getPatternStr().c_str());
  backlights.getPatternStr().toCharArray(MqttStatusBackPattern, backlights.getPatternStr().length() + 1);
  MqttStatusBackColorPhase = backlights.getColorPhase();
  MqttStatusGraphic = uclock.getActiveGraphicIdx();
  MqttStatusMainGraphic = uclock.getActiveGraphicIdx();
  MqttStatusUseTwelveHours = uclock.getTwelveHour();
  MqttStatusBlankZeroHours = uclock.getBlankHoursZero();
  MqttStatusPulseBpm = backlights.getPulseRate();
  MqttStatusBreathBpm = backlights.getBreathRate();
  MqttStatusRainbowSec = backlights.getRainbowDuration();
  MqttStatusCountdownRunning = uclock.isCountdownRunning();
  MqttStatusCountdownRemaining = uclock.getRemainingSeconds();
  MqttStatusAlternate = alternateMode;

  if (MqttCommandReceived)
  {
    lastMqttCommandExecuted = millis();
    MqttReportBackEverything(true);
  }

  if (lastMqttCommandExecuted != -1)
  {
    if (((millis() - lastMqttCommandExecuted) > (MQTT_SAVE_PREFERENCES_AFTER_SEC * 1000)) && menu.getState() == Menu::idle)
    {
      lastMqttCommandExecuted = -1;

      Serial.print("Saving config...");
      stored_config.save();
      Serial.println(" Done.");
    }
  }

  buttons.loop();

#ifdef HARDWARE_NovelLife_SE_CLOCK // NovelLife_SE Clone XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
  HandleGestureInterupt();
#endif // NovelLife_SE Clone XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

  // Power button: If in menu, exit menu. Else turn off displays and backlight.
#ifndef ONE_BUTTON_ONLY_MENU
  if (buttons.power.isUpEdge() && (menu.getState() == Menu::idle))
  {
#ifdef DEBUG_OUTPUT
    Serial.println("Power button pressed.");
#endif
    tfts.chip_select.setAll();
    tfts.fillScreen(TFT_BLACK);
    tfts.toggleAllDisplays();
    if (tfts.isEnabled())
    {
#ifndef HARDWARE_SI_HAI_CLOCK
      tfts.reinit(); // reinit (original EleksTube HW: after a few hours in OFF state the displays do not wake up properly)
#endif
      tfts.chip_select.setAll();
      tfts.fillScreen(TFT_BLACK);
      updateDisplay(TFTs::force);
    }
    backlights.togglePower();
  }
#endif

  menu.loop(buttons); // Must be called after buttons.loop()
  backlights.loop();
  uclock.loop();

#ifdef DIMMING
  checkDimmingNeeded(); // night or day time brightness change
#endif

  // Other display updates or logic
  if (alternateMode) {
    unsigned long currentTime = millis();
    if (currentTime - lastModeSwitch >= MODE_SWITCH_INTERVAL) {
        lastModeSwitch = currentTime;

        // Check if countdown is running or countdown effect is still active
        if (uclock.isCountdownRunning() || countdownFinished) {
            // Alternate between countdown and sensor display
            if (currentMode == COUNTDOWN) {
                // Do not switch to SENSOR_DISPLAY while countdown is finished and effect is active
                if (!countdownFinished && uclock.getRemainingSeconds() >= 60) {
                    currentMode = SENSOR_DISPLAY;
                }
            } else {
                currentMode = COUNTDOWN;
            }
        } else {
            // Alternate between clock and sensor display
            if (currentMode == CLOCK) {
                currentMode = SENSOR_DISPLAY;
            } else {
                currentMode = CLOCK;
            }
        }

        updateDisplay(TFTs::force);
    }
  }

  updateDisplay(TFTs::show_t::yes);

  UpdateDstEveryNight();

  // Menu
  if (menu.stateChanged() && tfts.isEnabled())
  {
    Menu::states menu_state = menu.getState();
    int8_t menu_change = menu.getChange();

    if (menu_state == Menu::idle)
    {
      // We just changed into idle, so force a redraw of all clock digits and save the config.
      updateDisplay(TFTs::force); // redraw all the clock digits
      Serial.println();
      Serial.print("Saving config! Triggered from leaving menu...");
      stored_config.save();
      Serial.println(" Done.");
    }
    else
    {
      // Backlight Pattern
      if (menu_state == Menu::backlight_pattern)
      {
        if (menu_change != 0)
        {
          backlights.setNextPattern(menu_change);
        }
        setupMenu();
        tfts.println("Pattern:");
        tfts.println(backlights.getPatternStr());
      }
      // Backlight Color
      else if (menu_state == Menu::pattern_color)
      {
        if (menu_change != 0)
        {
          backlights.adjustColorPhase(menu_change * 16);
        }
        setupMenu();
        tfts.println("Color:");
        tfts.printf("%06X\n", backlights.getColor());
      }
      // Backlight Intensity
      else if (menu_state == Menu::backlight_intensity)
      {
        if (menu_change != 0)
        {
          backlights.adjustIntensity(menu_change);
        }
        setupMenu();
        tfts.println("Intensity:");
        tfts.println(backlights.getIntensity());
      }
      // 12 Hour or 24 Hour mode?
      else if (menu_state == Menu::twelve_hour)
      {
        if (menu_change != 0)
        {
          uclock.toggleTwelveHour();
          tfts.setDigit(HOURS_TENS, uclock.getHoursTens(), TFTs::force);
          tfts.setDigit(HOURS_ONES, uclock.getHoursOnes(), TFTs::force);
        }
        setupMenu();
        tfts.println("Hour format");
        tfts.println(uclock.getTwelveHour() ? "12 hour" : "24 hour");
      }
      // Blank leading zeros on the hours?
      else if (menu_state == Menu::blank_hours_zero)
      {
        if (menu_change != 0)
        {
          uclock.toggleBlankHoursZero();
          tfts.setDigit(HOURS_TENS, uclock.getHoursTens(), TFTs::force);
        }
        setupMenu();
        tfts.println("Blank zero?");
        tfts.println(uclock.getBlankHoursZero() ? "yes" : "no");
      }
      // UTC Offset, hours
      else if (menu_state == Menu::utc_offset_hour)
      {
        time_t currOffset = uclock.getTimeZoneOffset();

        if (menu_change != 0)
        {
          // calculate the new offset
          time_t newOffsetAdjustmentValue = menu_change * 3600;
          time_t newOffset = currOffset + newOffsetAdjustmentValue;

          // check if the new offset is within the allowed range of -12 to +12 hours
          // If the minutes part of the offset is 0, we want to change from +12 to -12 or vice versa (without changing the shown time on the displays)
          // If the minutes part is not 0: We want to wrap around to the other side and change the minutes part (i.e. from 11:45 directly to -11:15)
          bool offsetWrapAround = false;
          if (newOffset > 43200)
          { // we just "passed" +12 hours -> set to -12 hours
            newOffset = -43200;
            offsetWrapAround = true;
          }
          if (newOffset < -43200 && !offsetWrapAround)
          { // we just passed -12 hours -> set to +12 hours
            newOffset = 43200;
          }

          uclock.setTimeZoneOffset(newOffset); // set the new offset
          uclock.loop();                       // update the clock time and redraw the changed digits -> will "flicker" the menu for a short time, but without, menu is not redrawn correctly
#ifdef DIMMING
          checkDimmingNeeded(); // check if we need dimming for the night, because timezone was changed
#endif
          currOffset = uclock.getTimeZoneOffset(); // get the new offset as current offset for the menu
        }
        setupMenu();
        tfts.println("UTC Offset");
        tfts.println(" +/- Hour");
        char offsetStr[11];
        int8_t offset_hour = currOffset / 3600;
        int8_t offset_min = (currOffset % 3600) / 60;
        if (offset_min <= 0 && offset_hour <= 0)
        { // negative timezone value -> Make them positive and print a minus in front
          offset_min = -offset_min;
          offset_hour = -offset_hour;
          snprintf(offsetStr, sizeof(offsetStr), "-%d:%02d", offset_hour, offset_min);
        }
        else
        {
          if (offset_min >= 0 && offset_hour >= 0)
          { // postive timezone value for hours and minutes -> show a plus in front
            snprintf(offsetStr, sizeof(offsetStr), "+%d:%02d", offset_hour, offset_min);
          }
        }
        if (offset_min == 0 && offset_hour == 0)
        { // we don't want a sign in front of the 0:00 case
          snprintf(offsetStr, sizeof(offsetStr), "%d:%02d", offset_hour, offset_min);
        }
        tfts.println(offsetStr);
      } // END UTC Offset, hours
      // BEGIN UTC Offset, 15 minutes
      else if (menu_state == Menu::utc_offset_15m)
      {
        time_t currOffset = uclock.getTimeZoneOffset();

        if (menu_change != 0)
        {
          time_t newOffsetAdjustmentValue = menu_change * 900; // calculate the new offset
          time_t newOffset = currOffset + newOffsetAdjustmentValue;

          // check if the new offset is within the allowed range of -12 to +12 hours
          // same behaviour as for the +/-1 hour offset, but with 15 minutes steps
          bool offsetWrapAround = false;
          if (newOffset > 43200)
          { // we just "passed" +12 hours -> set to -12 hours
            newOffset = -43200;
            offsetWrapAround = true;
          }
          if (newOffset < -43200 && !offsetWrapAround)
          { // we just passed -12 hours -> set to +12 hours
            newOffset = 43200;
          }

          uclock.setTimeZoneOffset(newOffset); // set the new offset
          uclock.loop();                       // update the clock time and redraw the changed digits -> will "flicker" the menu for a short time, but without, menu is not redrawn correctly
#ifdef DIMMING
          checkDimmingNeeded(); // check if we need dimming for the night, because timezone was changed
#endif
          currOffset = uclock.getTimeZoneOffset(); // get the new offset as current offset for the menu
        }
        setupMenu();
        tfts.println("UTC Offset");
        tfts.println(" +/- 15m");
        char offsetStr[11];
        int8_t offset_hour = currOffset / 3600;
        int8_t offset_min = (currOffset % 3600) / 60;
        if (offset_min <= 0 && offset_hour <= 0)
        { // negative timezone value -> Make them positive and print a minus in front
          offset_min = -offset_min;
          offset_hour = -offset_hour;
          snprintf(offsetStr, sizeof(offsetStr), "-%d:%02d", offset_hour, offset_min);
        }
        else
        {
          if (offset_min >= 0 && offset_hour >= 0)
          { // postive timezone value for hours and minutes -> show a plus in front
            snprintf(offsetStr, sizeof(offsetStr), "+%d:%02d", offset_hour, offset_min);
          }
        }
        if (offset_min == 0 && offset_hour == 0)
        { // we don't want a sign in front of the 0:00 case so overwrite the string
          snprintf(offsetStr, sizeof(offsetStr), "%d:%02d", offset_hour, offset_min);
        }
        tfts.println(offsetStr);
      } // END UTC Offset, 15 minutes
      // select clock face
      else if (menu_state == Menu::selected_graphic)
      {
        if (menu_change != 0)
        {
          uclock.adjustClockGraphicsIdx(menu_change);

          if (tfts.current_graphic != uclock.getActiveGraphicIdx())
          {
            tfts.current_graphic = uclock.getActiveGraphicIdx();
            updateDisplay(TFTs::force); // redraw all the clock digits
          }
        }
        setupMenu();
        tfts.println("Selected");
        tfts.println("graphic:");
        tfts.printf("    %d\n", uclock.getActiveGraphicIdx());
      }
#ifdef WIFI_USE_WPS //  WPS code
      // connect to WiFi using wps pushbutton mode
      else if (menu_state == Menu::start_wps)
      {
        if (menu_change != 0)
        { // button was pressed
          if (menu_change < 0)
          { // left button
            Serial.println("WiFi WPS start request");
            tfts.clear();
            tfts.fillScreen(TFT_BLACK);
            tfts.setTextColor(TFT_WHITE, TFT_BLACK);
            tfts.setCursor(0, 0, 4); // Font 4. 26 pixel high
            WiFiStartWps();
          }
        }
        setupMenu();
        tfts.println("Connect to WiFi?");
        tfts.println("Left=WPS");
      }
#endif
    }
  } // if (menu.stateChanged())

  // Countdown has finished, start the background breath effect
  if (currentMode == COUNTDOWN && !uclock.isCountdownRunning() && !countdownHandled) {
    if (backlights.getCurrentPattern() != Backlights::breath) {
        backlights.setPattern(Backlights::breath);
    }

    // Set the countdown finished flag and record the finish time
    countdownFinished = true;
    countdownFinishTime = millis();
    countdownHandled = true;

    // Send countdown finished message via mqtt
    MqttSendCountdownFinished();
  }

  // Check if the countdown has finished and if BACKLIGHT_PULSE_DURATION_MS have passed
  // This is the duration how long the backlight breath effect is kept on after the countdown has finished
  if (countdownFinished && (millis() - countdownFinishTime >= BACKLIGHT_PULSE_DURATION_MS)) {
    // Turn off the breath effect
    backlights.setPattern(Backlights::dark);
    countdownFinished = false; // Reset the flag
  }

  uint32_t time_in_loop = millis() - millis_at_top;
  if (time_in_loop < 20)
  {
    // we have free time, spend it for loading next image into buffer
    tfts.LoadNextImage();

    // we still have extra time
    time_in_loop = millis() - millis_at_top;
    if (time_in_loop < 20)
    {
      MqttLoopInFreeTime();
      PeriodicReadTemperature();
      if (bTemperatureUpdated)
      {
        tfts.setDigit(HOURS_ONES, uclock.getHoursOnes(), TFTs::force); // show latest clock digit and temperature readout together
        bTemperatureUpdated = false;
      }

      // run once a day (= 744 times per month which is below the limit of 5k for free account)
      if (DstNeedsUpdate)
      { // Daylight savings time changes at 3 in the morning
        if (GetGeoLocationTimeZoneOffset())
        {
          uclock.setTimeZoneOffset(GeoLocTZoffset * 3600);
          DstNeedsUpdate = false; // done for this night; retry if not sucessfull
        }
      }
      // Sleep for up to 20ms, less if we've spent time doing stuff above.
      time_in_loop = millis() - millis_at_top;
      if (time_in_loop < 20)
      {
        delay(20 - time_in_loop);
      }
    }
  }
#ifdef DEBUG_OUTPUT
  if (time_in_loop <= 1)
    Serial.print(".");
  else
  {
    Serial.print("time spent in loop (ms): ");
    Serial.println(time_in_loop);
  }
#endif
}

#ifdef HARDWARE_NovelLife_SE_CLOCK // NovelLife_SE Clone XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
void GestureStart()
{
  // for gesture sensor APDS9660 - Set interrupt pin on ESP32 as input
  pinMode(GESTURE_SENSOR_INPUT_PIN, INPUT);

  // Initialize interrupt service routine for interupt from APDS-9960 sensor
  attachInterrupt(digitalPinToInterrupt(GESTURE_SENSOR_INPUT_PIN), GestureInterruptRoutine, FALLING);

  // Initialize gesture sensor APDS-9960 (configure I2C and initial values)
  if (apds.init())
  {
    Serial.println(F("APDS-9960 initialization complete"));

    // Set Gain to 1x, bacause the cheap chinese fake APDS sensor can't handle more (also remember to extend ID check in Sparkfun libary to 0x3B!)
    apds.setGestureGain(GGAIN_1X);

    // Start running the APDS-9960 gesture sensor engine
    if (apds.enableGestureSensor(true))
    {
      Serial.println(F("Gesture sensor is now running"));
    }
    else
    {
      Serial.println(F("Something went wrong during gesture sensor enablimg in the APDS-9960 library!"));
    }
  }
  else
  {
    Serial.println(F("Something went wrong during APDS-9960 init!"));
  }
}

// Handle Interrupt from gesture sensor and simulate a short button press of the corresponding button, if a gesture is detected
void HandleGestureInterupt()
{
  if (isr_flag == 1)
  {
    detachInterrupt(digitalPinToInterrupt(GESTURE_SENSOR_INPUT_PIN));
    HandleGesture();
    isr_flag = 0;
    attachInterrupt(digitalPinToInterrupt(GESTURE_SENSOR_INPUT_PIN), GestureInterruptRoutine, FALLING);
  }
  return;
}

// mark, that the Interrupt of the gesture sensor was signaled
void GestureInterruptRoutine()
{
  isr_flag = 1;
  return;
}

// check which gesture was detected
void HandleGesture()
{
  // Serial.println("->main::HandleGesture()");
  if (apds.isGestureAvailable())
  {
    switch (apds.readGesture())
    {
    case DIR_UP:
      buttons.left.setUpEdgeState();
      Serial.println("Gesture detected! LEFT");
      break;
    case DIR_DOWN:
      buttons.right.setUpEdgeState();
      Serial.println("Gesture detected! RIGHT");
      break;
    case DIR_LEFT:
      buttons.power.setUpEdgeState();
      Serial.println("Gesture detected! DOWN");
      break;
    case DIR_RIGHT:
      buttons.mode.setUpEdgeState();
      Serial.println("Gesture detected! UP");
      break;
    case DIR_NEAR:
      buttons.mode.setUpEdgeState();
      Serial.println("Gesture detected! NEAR");
      break;
    case DIR_FAR:
      buttons.power.setUpEdgeState();
      Serial.println("Gesture detected! FAR");
      break;
    default:
      Serial.println("Movement detected but NO gesture detected!");
    }
  }
  return;
}
#endif // NovelLife_SE Clone XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

void setupMenu()
{                                  // Prepare drawing of the menu texts
  tfts.chip_select.setHoursTens(); // use most left display
  tfts.setTextColor(TFT_WHITE, TFT_BLACK);
  tfts.fillRect(0, 120, 135, 120, TFT_BLACK); // use lower half of the display, fill with black
  tfts.setCursor(0, 124, 4);                  // use font 4 - 26 pixel high - for the menu text
}

#ifdef DIMMING
bool isNightTime(uint8_t current_hour)
{ // check the actual hour is in the defined "night time"
  if (DAY_TIME < NIGHT_TIME)
  { // "Night" spans across midnight so it is split between two days
    return (current_hour < DAY_TIME) || (current_hour >= NIGHT_TIME);
  }
  else
  { // "Night" starts after midnight, entirely contained within the current day
    return (current_hour >= NIGHT_TIME) && (current_hour < DAY_TIME);
  }
}

void checkDimmingNeeded()
{                                             // dim the display in the defined night time
  uint8_t current_hour = uclock.getHour24();  // for internal calcs we always use 24h format
  isDimmingNeeded = current_hour != hour_old; // check, if the hour has changed since last loop (from time passing by or from timezone change)
  if (isDimmingNeeded)
  {
    Serial.print("Current hour = ");
    Serial.print(current_hour);
    Serial.print(", Night Time Start = ");
    Serial.print(NIGHT_TIME);
    Serial.print(", Day Time Start = ");
    Serial.println(DAY_TIME);
    if (isNightTime(current_hour))
    { // check if it is in the defined night time
      Serial.println("Set to night time mode (dimmed)!");
      tfts.dimming = TFT_DIMMED_INTENSITY;
      tfts.ProcessUpdatedDimming();
      backlights.setDimming(true);
    }
    else
    {
      Serial.println("Setting daytime mode (max brightness)");
      tfts.dimming = 255; // 0..255
      tfts.ProcessUpdatedDimming();
      backlights.setDimming(false);
    }
    updateDisplay(TFTs::force); // redraw all the clock digits -> software dimming will be done here
    hour_old = current_hour;
  }
}
#endif // DIMMING

void UpdateDstEveryNight()
{
  uint8_t currentDay = uclock.getDay();
  // This `DstNeedsUpdate` is True between 3:00:05 and 3:00:59. Has almost one minute of time slot to fetch updates, incl. eventual retries.
  DstNeedsUpdate = (currentDay != yesterday) && (uclock.getHour24() == 3) && (uclock.getMinute() == 0) && (uclock.getSecond() > 5);
  if (DstNeedsUpdate)
  {
    Serial.print("DST needs update...");

    // Update day after geoloc was sucesfully updated. Otherwise this will immediatelly disable the failed update retry.
    yesterday = currentDay;
  }
}

void updateDisplay(TFTs::show_t show) {
    if (currentMode == SENSOR_DISPLAY) {
        updateSensorDisplay(show);
    } else if (currentMode == COUNTDOWN) {
        updateCountdownDisplay(show);
    } else {
        updateClockDisplay(show);
    }
}

void updateClockDisplay(TFTs::show_t show)
{
    // refresh starting on seconds
    tfts.setDigit(SECONDS_ONES, uclock.getSecondsOnes(), show);
    tfts.setDigit(SECONDS_TENS, uclock.getSecondsTens(), show);
    tfts.setDigit(MINUTES_ONES, uclock.getMinutesOnes(), show);
    tfts.setDigit(MINUTES_TENS, uclock.getMinutesTens(), show);
    tfts.setDigit(HOURS_ONES, uclock.getHoursOnes(), show);
    tfts.setDigit(HOURS_TENS, uclock.getHoursTens(), show);
}

void updateCountdownDisplay(TFTs::show_t show) {
    static bool colonVisible = true;
    static unsigned long lastToggleTime = 0;
    unsigned long currentTime = millis();

    // Only blink the colon if countdown is still running
    if (!countdownFinished) {
        // Calculate the time elapsed since the last toggle
        unsigned long elapsedTime = currentTime - lastToggleTime;

        // Blink pattern: visible for 800ms, off for 200ms
        if (colonVisible && elapsedTime >= 800) {
            colonVisible = false;
            lastToggleTime = currentTime;
        } else if (!colonVisible && elapsedTime >= 200) {
            colonVisible = true;
            lastToggleTime = currentTime;
        }
    } else {
        // Keep colon visible when countdown is finished
        colonVisible = true;
    }

    if (uclock.getRemainingSeconds() < 3600) { // Less than 1 hour
        tfts.setDigit(HOURS_TENS, 0, show, TFTs::HOURGLASS);

        // Use 4 displays for mm:ss and 1 for the colon
        tfts.setDigit(HOURS_ONES, uclock.getCountdownMinutesTens(), show);
        tfts.setDigit(MINUTES_TENS, uclock.getCountdownMinutesOnes(), show);

        // Show colon based on colonVisible state
        if (colonVisible) {
            tfts.setDigit(MINUTES_ONES, 0, show, TFTs::COLON);
        } else {
            tfts.setDigit(MINUTES_ONES, TFTs::blanked, show);
        }

        tfts.setDigit(SECONDS_TENS, uclock.getCountdownSecondsTens(), show);
        tfts.setDigit(SECONDS_ONES, uclock.getCountdownSecondsOnes(), show);
    } else {
        // Use all 6 displays for hh:mm:ss
        tfts.setDigit(HOURS_TENS, uclock.getCountdownHoursTens(), show);
        tfts.setDigit(HOURS_ONES, uclock.getCountdownHoursOnes(), show);
        tfts.setDigit(MINUTES_TENS, uclock.getCountdownMinutesTens(), show);
        tfts.setDigit(MINUTES_ONES, uclock.getCountdownMinutesOnes(), show);
        tfts.setDigit(SECONDS_TENS, uclock.getCountdownSecondsTens(), show);
        tfts.setDigit(SECONDS_ONES, uclock.getCountdownSecondsOnes(), show);
    }
}

void updateSensorDisplay(TFTs::show_t show) {
    // Display temperature as two integer digits on displays #5 and #4
    if (MqttTemperatureSensorOnline) {
        int temperature = static_cast<int>(MqttCommandTemperature);
        int temperatureTens = temperature / 10;
        if (temperatureTens == 0) {
            tfts.setDigit(HOURS_TENS, TFTs::blanked, show);
        } else {
            tfts.setDigit(HOURS_TENS, temperatureTens, show);
        }
        tfts.setDigit(HOURS_ONES, temperature % 10, show);
    } else {
        // Blank temperature displays if sensor is offline
        tfts.setDigit(HOURS_TENS, TFTs::blanked, show);
        tfts.setDigit(HOURS_ONES, TFTs::blanked, show);
    }

    // Display "°C" image on display #3
    tfts.setDigit(MINUTES_TENS, 0, show, TFTs::CELSIUS);

    // Display humidity as two integer digits on displays #2 and #1
    if (MqttHumiditySensorOnline) {
        int humidity = static_cast<int>(MqttCommandHumidity);
        int humidityTens = humidity / 10;
        if (humidityTens == 0) {
            tfts.setDigit(MINUTES_ONES, TFTs::blanked, show);
        } else {
            tfts.setDigit(MINUTES_ONES, humidityTens, show);
        }
        tfts.setDigit(SECONDS_TENS, humidity % 10, show);
    } else {
        // Blank humidity displays if sensor is offline
        tfts.setDigit(MINUTES_ONES, TFTs::blanked, show);
        tfts.setDigit(SECONDS_TENS, TFTs::blanked, show);
    }

    // Display "%" image on display #0
    tfts.setDigit(SECONDS_ONES, 0, show, TFTs::PERCENT);
}

const char* modeToString(Mode mode) {
    switch (mode) {
        case CLOCK: return "clock";
        case COUNTDOWN: return "countdown";
        case SENSOR_DISPLAY: return "sensor_display";
        default: return "unknown";
    }
}
Mode getCurrentMode() {
    return currentMode; // Return the current mode
}
