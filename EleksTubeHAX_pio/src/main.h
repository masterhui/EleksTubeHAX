#ifndef MAIN_H_
#define MAIN_H_

// Enum declaration
enum Mode {
    CLOCK,
    COUNTDOWN,
    SENSOR_DISPLAY
};

// Function declarations
void updateClockDisplay(TFTs::show_t show = TFTs::yes);
void setupMenu();
void EveryFullHour(bool loopUpdate = false);
Mode getCurrentMode(); // Declaration of the getter function
const char* modeToString(Mode mode);
void updateDisplay(TFTs::show_t show);
void updateClockDisplay(TFTs::show_t show);
void updateCountdownDisplay(TFTs::show_t show);
void updateSensorDisplay(TFTs::show_t show);
void setCurrentMode(Mode newMode);

#endif /* MAIN_H_ */
