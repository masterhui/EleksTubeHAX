#include "Menu.h"
#include "Clock.h"

// Big ol' state machine: menu and buttons as the state, buttons as the transition triggers.

#ifndef ONE_BUTTON_ONLY_MENU
void Menu::loop(Buttons &buttons)
{
  Button::state left_state = buttons.left.getState();   // decrement value
  Button::state right_state = buttons.right.getState(); // increment value
  Button::state power_state = buttons.power.getState(); // exit menu
  Button::state mode_state = buttons.mode.getState();   // next menu

  // Reset the change value in every case.  We don't always change the state though.
  change = 0;
  state_changed = false;

  // Early out for idle state, which will be most of the time.
  if (state == idle && left_state == Button::idle && right_state == Button::idle && mode_state == Button::idle)
  {
    // Everything is idle.
    return;
  }

  // Go idle if the user hasn't pressed a button in a long time.
  if (state != idle && millis() - millis_last_button_press > idle_timeout_ms)
  {
    // Go idle.
    state = idle;
    state_changed = true;
    return;
  }

  // Menu is idle. A button is pressed, go into the menu, but don't act on the button press. It just wakes up the menu.
  if (state == idle && (left_state == Button::up_edge || right_state == Button::up_edge || mode_state == Button::up_edge))
  {
    state = states(1); // Start at the beginning of the menu.

    millis_last_button_press = millis();
    state_changed = true;
    return;
  }

  // Go to the next menu option
  if (state != idle && mode_state == Button::up_edge)
  {
    uint8_t new_state = (uint8_t(state) + 1) % num_states;
    if (new_state == 0)
    {
      new_state = 1; // Skip over idle when incrementing through the menu.
    }
    state = states(new_state);

    millis_last_button_press = millis();
    state_changed = true;
    return;
  }

  // Exit with a power button.
  if (state != idle && (power_state == Button::up_edge))
  {
    state = idle;
    state_changed = true;
    return;
  }

  // In a menu, and a left (negative change value) or right button (positive change value) has been pressed!
  if (state != idle && (left_state == Button::up_edge || right_state == Button::up_edge))
  {
    // Pressing both left and right at the same time cancels out?  Sure, why not...
    if (left_state == Button::up_edge)
    {
      change--;
    }
    if (right_state == Button::up_edge)
    {
      change++;
    }

    millis_last_button_press = millis();
    state_changed = true;
    return;
  }

  // Map button states to ButtonPress enum
  if (left_state == Button::up_edge) {
    button_press = BUTTON_DOWN;
  } else if (right_state == Button::up_edge) {
    button_press = BUTTON_UP;
  } else if (mode_state == Button::up_edge) {
    button_press = BUTTON_SELECT;
  } else if (power_state == Button::up_edge) {
    button_press = BUTTON_BACK;
  } else if (power_state == Button::up_long_edge) {
    button_press = BUTTON_START;
  } else {
    button_press = BUTTON_NONE;
  }

  // Some other button state, but it doesn't trigger any change in state.  There are LOTS of states that will
  // get here, but I think they're all "just do nothing."  If there's an explicit state we want to handle,
  // add an if() block above.
}
#endif

#ifdef ONE_BUTTON_ONLY_MENU
void Menu::loop(Buttons &buttons)
{
  Button::state mode_state = buttons.mode.getState(); // next menu

  // Reset the change value in every case.  We don't always change the state though.
  change = 0;
  state_changed = false;

  // Early out for idle state, which will be most of the time.
  if (state == idle && mode_state == Button::idle)
  {
    // Everything is idle. Do nothing.
    return;
  }

  // Go idle if the user hasn't pressed a button in a long time.
  if (state != idle && millis() - millis_last_button_press > idle_timeout_ms)
  {
    // Go idle.
    state = idle;
    state_changed = true;
    return;
  }

  // Menu is idle. A button is pressed, go into the menu, but don't act on the button press. It just wakes up the menu.
  if (state == idle && (mode_state == Button::up_edge))
  {
    state = states(1); // Start at the beginning of the menu.

    millis_last_button_press = millis();
    state_changed = true;
    return;
  }

  // In a menu, and button long pressed! -> simulate right button press
  /// Must be done BEFORE the next menu option
  if (state != idle && (mode_state == Button::up_long_edge))
  {
    change++;

    millis_last_button_press = millis();
    state_changed = true;
    return;
  }

  // Go to the next menu option
  if (state != idle && mode_state == Button::up_edge)
  {
    uint8_t new_state = (uint8_t(state) + 1) % num_states;
    if (new_state == 0)
    {
      new_state = 1; // Skip over idle when incrementing through the menu.
    }
    state = states(new_state);

    millis_last_button_press = millis();
    state_changed = true;
    return;
  }

  // Map button states to ButtonPress enum
  if (mode_state == Button::up_edge) {
    button_press = BUTTON_SELECT;
  } else if (mode_state == Button::up_long_edge) {
    button_press = BUTTON_START;
  } else {
    button_press = BUTTON_NONE;
  }

  // Some other button state, but it doesn't trigger any change in state.  There are LOTS of states that will
  // get here, but I think they're all "just do nothing."  If there's an explicit state we want to handle,
  // add an if() block above.
}
#endif

#ifndef WIFI_USE_WPS
const String Menu::state_str[Menu::num_states] = {
    "idle",
    "backlight_pattern",
    "pattern_color",
    "backlight_intensity",
    "twelve_hour",
    "blank_hours_zero",
    "utc_offset_hour",
    "utc_offset_15m",
    "selected_graphic",
    "countdown_timer"};
#else
const String Menu::state_str[Menu::num_states] = {
    "idle",
    "backlight_pattern",
    "pattern_color",
    "backlight_intensity",
    "twelve_hour",
    "blank_hours_zero",
    "utc_offset_hour",
    "utc_offset_15m",
    "selected_graphic",
    "start_wps",
    "countdown_timer"};
#endif

void Menu::handleCountdownTimer() {
    static uint32_t countdown_hours = 0;
    static uint32_t countdown_minutes = 0;
    static uint32_t countdown_seconds = 0;
    static uint8_t setting_position = 0; // 0=hours, 1=minutes, 2=seconds
    
    switch (button_press) {
        case BUTTON_SELECT:
            setting_position = (setting_position + 1) % 3;
            break;
            
        case BUTTON_UP:
            switch (setting_position) {
                case 0: // Hours
                    countdown_hours = (countdown_hours + 1) % 100;
                    break;
                case 1: // Minutes
                    countdown_minutes = (countdown_minutes + 1) % 60;
                    break;
                case 2: // Seconds
                    countdown_seconds = (countdown_seconds + 1) % 60;
                    break;
            }
            break;
            
        case BUTTON_DOWN:
            switch (setting_position) {
                case 0: // Hours
                    countdown_hours = countdown_hours > 0 ? countdown_hours - 1 : 99;
                    break;
                case 1: // Minutes
                    countdown_minutes = countdown_minutes > 0 ? countdown_minutes - 1 : 59;
                    break;
                case 2: // Seconds
                    countdown_seconds = countdown_seconds > 0 ? countdown_seconds - 1 : 59;
                    break;
            }
            break;
            
        case BUTTON_BACK:
            state = idle;
            return;
            
        case BUTTON_START:
            uint32_t total_seconds = countdown_hours * 3600 + 
                                   countdown_minutes * 60 + 
                                   countdown_seconds;
            if (total_seconds > 0) {
                uclock.startCountdown(total_seconds);
                state = idle;
                return;
            }
            break;
    }
    
    // Update display
    tfts.setDigit(HOURS_TENS, countdown_hours / 10, TFTs::show_t::yes);
    tfts.setDigit(HOURS_ONES, countdown_hours % 10, TFTs::show_t::yes);
    tfts.setDigit(MINUTES_TENS, countdown_minutes / 10, TFTs::show_t::yes);
    tfts.setDigit(MINUTES_ONES, countdown_minutes % 10, TFTs::show_t::yes);
    tfts.setDigit(SECONDS_TENS, countdown_seconds / 10, TFTs::show_t::yes);
    tfts.setDigit(SECONDS_ONES, countdown_seconds % 10, TFTs::show_t::yes);
    
    // Blink the current setting position
    if (millis() % 1000 > 500) {
        switch (setting_position) {
            case 0: // Hours
                tfts.setDigit(HOURS_TENS, 0xFF, TFTs::show_t::no);
                tfts.setDigit(HOURS_ONES, 0xFF, TFTs::show_t::no);
                break;
            case 1: // Minutes
                tfts.setDigit(MINUTES_TENS, 0xFF, TFTs::show_t::no);
                tfts.setDigit(MINUTES_ONES, 0xFF, TFTs::show_t::no);
                break;
            case 2: // Seconds
                tfts.setDigit(SECONDS_TENS, 0xFF, TFTs::show_t::no);
                tfts.setDigit(SECONDS_ONES, 0xFF, TFTs::show_t::no);
                break;
        }
    }
}