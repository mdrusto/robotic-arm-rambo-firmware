/* -*- c++ -*- */
/**
 * @file
 */

/**
 * @mainpage Reprap 3D printer firmware based on Sprinter and grbl.
 *
 * @section intro_sec Introduction
 *
 * This firmware is a mashup between Sprinter and grbl.
 * https://github.com/kliment/Sprinter
 * https://github.com/simen/grbl/tree
 *
 * It has preliminary support for Matthew Roberts advance algorithm
 * http://reprap.org/pipermail/reprap-dev/2011-May/003323.html
 *
 * Prusa Research s.r.o. https://www.prusa3d.cz
 *
 * @section copyright_sec Copyright
 *
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @section notes_sec Notes
 *
 * * Do not create static objects in global functions.
 *   Otherwise constructor guard against concurrent calls is generated costing
 *   about 8B RAM and 14B flash.
 *
 *
 */

#include "Configuration.h"
#include "Marlin.h"
#include "config.h"

#include "macros.h"

#ifdef ENABLE_AUTO_BED_LEVELING
#include "vector_3.h"
  #ifdef AUTO_BED_LEVELING_GRID
    #include "qr_solve.h"
  #endif
#endif // ENABLE_AUTO_BED_LEVELING

#ifdef MESH_BED_LEVELING
  #include "mesh_bed_leveling.h"
  #include "mesh_bed_calibration.h"
#endif

#include "printers.h"

#include "menu.h"
#include "ultralcd.h"
#include "backlight.h"

#include "planner.h"
#include "host.h"
#include "stepper.h"
#include "temperature.h"
#include "fancheck.h"
#include "motion_control.h"
#include "cardreader.h"
#include "ConfigurationStore.h"
#include "language.h"
#include "math.h"
#include "util.h"
#include "Timer.h"
#include "power_panic.h"
#include "Prusa_farm.h"

#include <avr/wdt.h>
#include <util/atomic.h>
#include <avr/pgmspace.h>

#include "Tcodes.h"
#include "Dcodes.h"
#include "SpoolJoin.h"
#include "stopwatch.h"

#ifndef LA_NOCOMPAT
#include "la10compat.h"
#endif

#include "spi.h"

#include "Filament_sensor.h"

#ifdef TMC2130
#include "tmc2130.h"
#endif //TMC2130

#ifdef XFLASH
#include "xflash.h"
#include "optiboot_xflash.h"
#endif //XFLASH

#include "xflash_dump.h"

#ifdef BLINKM
#include "BlinkM.h"
#include "Wire.h"
#endif

#if NUM_SERVOS > 0
#include "Servo.h"
#endif

#if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
#include <SPI.h>
#endif

#include "mmu2.h"

#define VERSION_STRING  "1.0.2"

#include "sound.h"

#include "cmdqueue.h"

//filament types 
#define FILAMENT_DEFAULT 0
#define FILAMENT_FLEX 1
#define FILAMENT_PVA 2
#define FILAMENT_UNDEFINED 255

//Stepper Movement Variables

//===========================================================================
//=============================imported variables============================
//===========================================================================

//===========================================================================
//=============================public variables=============================
//===========================================================================
#ifdef SDSUPPORT
CardReader card;
#endif

//used for PINDA temp calibration and pause print
#define DEFAULT_RETRACTION    1
#define DEFAULT_RETRACTION_MM 4 //MM

float default_retraction = DEFAULT_RETRACTION;


//Although this flag and many others like this could be represented with a struct/bitfield for each axis (more readable and efficient code), the implementation
//would not be standard across all platforms. That being said, the code will continue to use bitmasks for independent axis.
//Moreover, according to C/C++ standard, the ordering of bits is platform/compiler dependent and the compiler is allowed to align the bits arbitrarily,
//thus bit operations like shifting and masking may stop working and will be very hard to fix.
uint8_t axis_relative_modes = 0;

int feedmultiply=100; //100->1 200->2
int extrudemultiply=100; //100->1 200->2

bool homing_flag = false;
bool did_pause_print;

LongTimer safetyTimer;
static LongTimer crashDetTimer;

//unsigned long load_filament_time;

bool mesh_bed_leveling_flag = false;

uint32_t total_filament_used; // unit mm/100 or 10um
HeatingStatus heating_status;
int fan_edge_counter[2];
int fan_speed[2];


float extruder_multiplier[EXTRUDERS] = {1.0
  #if EXTRUDERS > 1
    , 1.0
    #if EXTRUDERS > 2
      , 1.0
    #endif
  #endif
};

float current_position[NUM_AXIS] = { 0.0, 0.0, 0.0, 0.0 };
//shortcuts for more readable code
#define _x current_position[X_AXIS]
#define _y current_position[Y_AXIS]
#define _z current_position[Z_AXIS]
#define _e current_position[E_AXIS]

float min_pos[3] = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS };
float max_pos[3] = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS };
bool axis_known_position[3] = {false, false, false};

static float pause_position[3] = { X_PAUSE_POS, Y_PAUSE_POS, Z_PAUSE_LIFT };

uint8_t fanSpeed = 0;
uint8_t newFanSpeed = 0;

#ifdef FWRETRACT
  bool retracted[EXTRUDERS]={false
    #if EXTRUDERS > 1
    , false
     #if EXTRUDERS > 2
      , false
     #endif
  #endif
  };
  bool retracted_swap[EXTRUDERS]={false
    #if EXTRUDERS > 1
    , false
     #if EXTRUDERS > 2
      , false
     #endif
  #endif
  };

  float retract_length_swap = RETRACT_LENGTH_SWAP;
  float retract_recover_length_swap = RETRACT_RECOVER_LENGTH_SWAP;
#endif

#ifndef PINDA_THERMISTOR
static bool temp_compensation_retracted = false;
#endif // !PINDA_THERMISTOR

  #ifdef PS_DEFAULT_OFF
    bool powersupply = false;
  #else
	  bool powersupply = true;
  #endif

static bool cancel_heatup = false;

int8_t busy_state = NOT_BUSY;
static long prev_busy_signal_ms = -1;
static uint8_t host_keepalive_interval = HOST_KEEPALIVE_INTERVAL;

const char errormagic[] PROGMEM = "Error:";
const char echomagic[] PROGMEM = "echo:";

float saved_start_position[NUM_AXIS] = {SAVED_START_POSITION_UNSET, 0, 0, 0};

uint16_t saved_segment_idx = 0;
bool isPartialBackupAvailable;

// storing estimated time to end of print counted by slicer
uint8_t print_percent_done_normal = PRINT_PERCENT_DONE_INIT;
uint8_t print_percent_done_silent = PRINT_PERCENT_DONE_INIT;
uint16_t print_time_remaining_normal = PRINT_TIME_REMAINING_INIT; //estimated remaining print time in minutes
uint16_t print_time_remaining_silent = PRINT_TIME_REMAINING_INIT; //estimated remaining print time in minutes
uint16_t print_time_to_change_normal = PRINT_TIME_REMAINING_INIT; //estimated remaining time to next change in minutes
uint16_t print_time_to_change_silent = PRINT_TIME_REMAINING_INIT; //estimated remaining time to next change in minutes

uint32_t IP_address = 0;

//===========================================================================
//=============================Private Variables=============================
//===========================================================================
#define MSG_BED_LEVELING_FAILED_TIMEOUT 30

const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};
float destination[NUM_AXIS] = {  0.0, 0.0, 0.0, 0.0};

// For tracing an arc
static float offset[2] = {0.0, 0.0};

// Current feedrate
float feedrate = 1500.0;

static const int8_t sensitive_pins[] PROGMEM = SENSITIVE_PINS; // Sensitive pin list for M42

//static float tt = 0;
//static float bt = 0;

//Inactivity shutdown variables
static LongTimer previous_millis_cmd;
static uint32_t max_inactive_time = 0;
static uint32_t stepper_inactive_time = DEFAULT_STEPPER_DEACTIVE_TIME*1000l;
static uint32_t safetytimer_inactive_time = DEFAULT_SAFETYTIMER_TIME_MINS*60*1000ul;

ShortTimer usb_timer;

bool Stopped=false;
bool processing_tcode; // Helper variable to block certain functions while T-code is being processed

//Insert variables if CHDK is defined
#ifdef CHDK
static uint32_t chdkHigh = 0;
static bool chdkActive = false;
#endif

//! @name RAM save/restore printing
//! @{
bool saved_printing = false; //!< Print is paused and saved in RAM
uint32_t saved_sdpos = 0; //!< SD card position, or line number in case of USB printing
uint8_t saved_printing_type = PowerPanic::PRINT_TYPE_NONE;
float saved_pos[NUM_AXIS] = { X_COORD_INVALID, 0, 0, 0 };
uint16_t saved_feedrate2 = 0; //!< Default feedrate (truncated from float)
static int saved_feedmultiply2 = 0;
uint16_t saved_extruder_temperature; //!< Active extruder temperature
uint8_t saved_bed_temperature; //!< Bed temperature
bool saved_extruder_relative_mode;
uint8_t saved_fan_speed = 0; //!< Print fan speed
//! @}
static bool uvlo_auto_recovery_ready = false;

static int saved_feedmultiply_mm = 100;

class AutoReportFeatures {
    union {
          struct {
            uint8_t temp : 1; //Temperature flag
            uint8_t fans : 1; //Fans flag
            uint8_t pos: 1;   //Position flag
            uint8_t ar4 : 1;  //Unused
            uint8_t ar5 : 1;  //Unused
            uint8_t ar6 : 1;  //Unused
            uint8_t ar7 : 1;  //Unused
          } __attribute__((packed)) bits;
          uint8_t byte;
        } arFunctionsActive;
    uint8_t auto_report_period;
public:
    LongTimer auto_report_timer;
    AutoReportFeatures():auto_report_period(0){ 
#if defined(AUTO_REPORT)
        arFunctionsActive.byte = 0xff; 
#else
        arFunctionsActive.byte = 0;
#endif //AUTO_REPORT
    }
    
    inline bool Temp()const { return arFunctionsActive.bits.temp != 0; }
    inline void SetTemp(uint8_t v){ arFunctionsActive.bits.temp = v; }

    inline bool Fans()const { return arFunctionsActive.bits.fans != 0; }
    inline void SetFans(uint8_t v){ arFunctionsActive.bits.fans = v; }

    inline bool Pos()const { return arFunctionsActive.bits.pos != 0; }
    inline void SetPos(uint8_t v){ arFunctionsActive.bits.pos = v; }
    
    inline void SetMask(uint8_t mask){ arFunctionsActive.byte = mask; }
    
    /// sets the autoreporting timer's period
    /// setting it to zero stops the timer
    void SetPeriod(uint8_t p){
        auto_report_period = p;
        if (auto_report_period != 0){
          auto_report_timer.start();
        } else{
          auto_report_timer.stop();
        }
    }
    
    inline void TimerStart() { auto_report_timer.start(); }
    inline bool TimerRunning()const { return auto_report_timer.running(); }
    inline bool TimerExpired() { return auto_report_timer.expired(auto_report_period * 1000ul); }
};

AutoReportFeatures autoReportFeatures;

//===========================================================================
//=============================Routines======================================
//===========================================================================

static void print_time_remaining_init();
static void gcode_M105();

#ifndef PINDA_THERMISTOR
static void temp_compensation_start();
static void temp_compensation_apply();
#endif

static uint16_t gcode_in_progress = 0;
static uint16_t mcode_in_progress = 0;

void serial_echopair_P(const char *s_P, float v)
    { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char *s_P, double v)
    { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char *s_P, unsigned long v)
    { serialprintPGM(s_P); SERIAL_ECHO(v); }

void serialprintPGM(const char *str) {
    while(uint8_t ch = pgm_read_byte(str)) {
        MYSERIAL.write((char)ch);
        ++str;
    }
}

void serialprintlnPGM(const char *str) {
    serialprintPGM(str);
    MYSERIAL.println();
}

#ifdef SDSUPPORT
  #include "SdFatUtil.h"
  int freeMemory() { return SdFatUtil::FreeRam(); }
#else
  extern "C" {
    extern unsigned int __bss_end;
    extern unsigned int __heap_start;
    extern void *__brkval;

    int freeMemory() {
      int free_memory;

      if ((int)__brkval == 0)
        free_memory = ((int)&free_memory) - ((int)&__bss_end);
      else
        free_memory = ((int)&free_memory) - ((int)__brkval);

      return free_memory;
    }
  }
#endif //!SDSUPPORT

void setup_killpin()
{
  #if defined(KILL_PIN) && KILL_PIN > -1
    SET_INPUT(KILL_PIN);
    WRITE(KILL_PIN,HIGH);
  #endif
}

// Set home pin
void setup_homepin(void)
{
#if defined(HOME_PIN) && HOME_PIN > -1
   SET_INPUT(HOME_PIN);
   WRITE(HOME_PIN,HIGH);
#endif
}

void setup_photpin()
{
  #if defined(PHOTOGRAPH_PIN) && PHOTOGRAPH_PIN > -1
    SET_OUTPUT(PHOTOGRAPH_PIN);
    WRITE(PHOTOGRAPH_PIN, LOW);
  #endif
}

void setup_powerhold()
{
  #if defined(SUICIDE_PIN) && SUICIDE_PIN > -1
    SET_OUTPUT(SUICIDE_PIN);
    WRITE(SUICIDE_PIN, HIGH);
  #endif
  #if defined(PS_ON_PIN) && PS_ON_PIN > -1
    SET_OUTPUT(PS_ON_PIN);
	#if defined(PS_DEFAULT_OFF)
	  WRITE(PS_ON_PIN, PS_ON_ASLEEP);
    #else
	  WRITE(PS_ON_PIN, PS_ON_AWAKE);
	#endif
  #endif
}

void suicide()
{
  #if defined(SUICIDE_PIN) && SUICIDE_PIN > -1
    SET_OUTPUT(SUICIDE_PIN);
    WRITE(SUICIDE_PIN, LOW);
  #endif
}

void servo_init()
{
  #if (NUM_SERVOS >= 1) && defined(SERVO0_PIN) && (SERVO0_PIN > -1)
    servos[0].attach(SERVO0_PIN);
  #endif
  #if (NUM_SERVOS >= 2) && defined(SERVO1_PIN) && (SERVO1_PIN > -1)
    servos[1].attach(SERVO1_PIN);
  #endif
  #if (NUM_SERVOS >= 3) && defined(SERVO2_PIN) && (SERVO2_PIN > -1)
    servos[2].attach(SERVO2_PIN);
  #endif
  #if (NUM_SERVOS >= 4) && defined(SERVO3_PIN) && (SERVO3_PIN > -1)
    servos[3].attach(SERVO3_PIN);
  #endif
  #if (NUM_SERVOS >= 5)
    #error "TODO: enter initalisation code for more servos"
  #endif
}

bool __attribute__((noinline)) printJobOngoing() {
    return (IS_SD_PRINTING || usb_timer.running() || print_job_timer.isRunning());
}

bool printingIsPaused() {
  return did_pause_print || print_job_timer.isPaused();
}

bool __attribute__((noinline)) printer_active() {
    return printJobOngoing()
        || printingIsPaused()
        || saved_printing
        || (lcd_commands_type != LcdCommands::Idle)
        || MMU2::mmu2.MMU_PRINT_SAVED()
        || homing_flag
        || mesh_bed_leveling_flag;
}

#ifdef DEBUG_PRINTER_STATES
//! @brief debug printer states
//!
//! This outputs a lot over serial and should be only used
//! - when debugging LCD menus
//! - or other functions related to these states
//! To reduce the output feel free to comment out the lines you
//! aren't troubleshooting.

void debug_printer_states()
{
    printf_P(PSTR("DBG:printJobOngoing() = %d\n"), (int)printJobOngoing());
    printf_P(PSTR("DBG:IS_SD_PRINTING = %d\n"), (int)IS_SD_PRINTING);
    printf_P(PSTR("DBG:printer_recovering() = %d\n"), (int)printer_recovering());
    printf_P(PSTR("DBG:heating_status = %d\n"), (int)heating_status);
    printf_P(PSTR("DBG:GetPrinterState() = %d\n"), (int)GetPrinterState());
    printf_P(PSTR("DBG:babystep_allowed_strict() = %d\n"), (int)babystep_allowed_strict());
    printf_P(PSTR("DBG:lcd_commands_type = %d\n"), (int)lcd_commands_type);
    printf_P(PSTR("DBG:farm_mode = %d\n"), (int)farm_mode);
    printf_P(PSTR("DBG:moves_planned() = %d\n"), (int)moves_planned());
    printf_P(PSTR("DBG:Stopped = %d\n"), (int)Stopped);
    printf_P(PSTR("DBG:usb_timer.running() = %d\n"), (int)usb_timer.running());
    printf_P(PSTR("DBG:M79_timer_get_status() = %d\n"), (int)M79_timer_get_status());
    printf_P(PSTR("DBG:print_job_timer.isRunning() = %d\n"), (int)print_job_timer.isRunning());
    printf_P(PSTR("DBG:printingIsPaused() = %d\n"), (int)printingIsPaused());
    printf_P(PSTR("DBG:did_pause_print = %d\n"), (int)did_pause_print);
    printf_P(PSTR("DBG:print_job_timer.isPaused() = %d\n"), (int)print_job_timer.isPaused());
    printf_P(PSTR("DBG:saved_printing = %d\n"), (int)saved_printing);
    printf_P(PSTR("DBG:saved_printing_type = %d\n"), (int)saved_printing_type);
    printf_P(PSTR("DBG:homing_flag = %d\n"), (int)homing_flag);
    printf_P(PSTR("DBG:mesh_bed_leveling_flag = %d\n"), (int)mesh_bed_leveling_flag);
    printf_P(PSTR("DBG:get_temp_error() = %d\n"), (int)get_temp_error());
    printf_P(PSTR("DBG:card.mounted = %d\n"), (int)card.mounted);
    printf_P(PSTR("DBG:card.isFileOpen() = %d\n"), (int)card.isFileOpen());
    printf_P(PSTR("DBG:fan_check_error = %d\n"), (int)fan_check_error);
    printf_P(PSTR("DBG:processing_tcode = %d\n"), (int)processing_tcode);
    printf_P(PSTR("DBG:nextSheet = %d\n"), (int)eeprom_next_initialized_sheet(eeprom_read_byte(&(EEPROM_Sheets_base->active_sheet))));
    printf_P(PSTR("DBG:eFilamentAction = %d\n"), (int)eFilamentAction);
    printf_P(PSTR("DBG:MMU2::mmu2.Enabled() = %d\n"), (int)MMU2::mmu2.Enabled());
    printf_P(PSTR("DBG:MMU2::mmu2.MMU_PRINT_SAVED() = %d\n"), (int)MMU2::mmu2.MMU_PRINT_SAVED());
    printf_P(PSTR("DBG:MMU2::mmu2.FindaDetectsFilament() = %d\n"), (int)MMU2::mmu2.FindaDetectsFilament());
    printf_P(PSTR("DBG:fsensor.getFilamentPresent() = %d\n"), (int)fsensor.getFilamentPresent());
    printf_P(PSTR("DBG:MMU CUTTER ENABLED = %d\n"), (int)eeprom_read_byte((uint8_t*)EEPROM_MMU_CUTTER_ENABLED));
    printf_P(PSTR("DBG:fsensor.isEnabled() = %d\n"), (int)fsensor.isEnabled());
    printf_P(PSTR("DBG:fsensor.getAutoLoadEnabled() = %d\n"), (int)fsensor.getAutoLoadEnabled());
    printf_P(PSTR("DBG:custom_message_type = %d\n"), (int)custom_message_type);
    printf_P(PSTR("DBG:uvlo_auto_recovery_ready = %d\n"), (int)uvlo_auto_recovery_ready);
    SERIAL_ECHOLN("");
}
#endif //End DEBUG_PRINTER_STATES

// Block LCD menus when
bool __attribute__((noinline)) printer_recovering() {
    return (eeprom_read_byte((uint8_t*)EEPROM_UVLO) != PowerPanic::NO_PENDING_RECOVERY);
}

// Currently only used in one place, allowed to be inlined
bool check_fsensor() {
    return printJobOngoing()
        && mcode_in_progress != 600
        && !saved_printing
        && !mesh_bed_leveling_flag
        && !homing_flag
        && e_active();
}

bool __attribute__((noinline)) babystep_allowed() {
    return ( !homing_flag
        && !mesh_bed_leveling_flag
        && !printingIsPaused()
        && ((lcd_commands_type == LcdCommands::Layer1Cal && CHECK_ALL_HEATERS)
            || printJobOngoing()
            || lcd_commands_type == LcdCommands::Idle
        )
    );
}

bool __attribute__((noinline)) babystep_allowed_strict() {
    return ( babystep_allowed() && current_position[Z_AXIS] < Z_HEIGHT_HIDE_LIVE_ADJUST_MENU);
}

bool fans_check_enabled = true;

#ifdef TMC2130

void crashdet_stop_and_save_print()
{
	stop_and_save_print_to_ram(10, -default_retraction); //XY - no change, Z 10mm up, E -1mm retract
}

void crashdet_restore_print_and_continue()
{
  restore_print_from_ram_and_continue(default_retraction); //XYZ = orig, E +1mm unretract
//babystep_apply();
}

void crashdet_fmt_error(char* buf, uint8_t mask)
{
    if(mask & X_AXIS_MASK) *buf++ = axis_codes[X_AXIS];
    if(mask & Y_AXIS_MASK) *buf++ = axis_codes[Y_AXIS];
    *buf++ = ' ';
    strcpy_P(buf, _T(MSG_CRASH_DETECTED));
}

void crashdet_detected(uint8_t mask)
{
	st_synchronize();
	static uint8_t crashDet_counter = 0;
	static uint8_t crashDet_axes = 0;
	bool automatic_recovery_after_crash = true;
	char msg[LCD_WIDTH+1] = "";

    if (crashDetTimer.expired(CRASHDET_TIMER * 1000ul)) {
        crashDet_counter = 0;
    }
    if(++crashDet_counter >= CRASHDET_COUNTER_MAX) {
        automatic_recovery_after_crash = false;
    }
    crashDetTimer.start();
    crashDet_axes |= mask;

	if (mask & X_AXIS_MASK) {
		eeprom_increment_byte((uint8_t*)EEPROM_CRASH_COUNT_X);
		eeprom_increment_word((uint16_t*)EEPROM_CRASH_COUNT_X_TOT);
	}
	if (mask & Y_AXIS_MASK) {
		eeprom_increment_byte((uint8_t*)EEPROM_CRASH_COUNT_Y);
		eeprom_increment_word((uint16_t*)EEPROM_CRASH_COUNT_Y_TOT);
	}

    lcd_update_enable(true);
    lcd_update(2);
    // prepare the status message with the _current_ axes status
    crashdet_fmt_error(msg, mask);
    lcd_setstatus(msg);

	//gcode_G28(true, true, false); //home X and Y

	if (automatic_recovery_after_crash) {
		enquecommand_P(PSTR("CRASH_RECOVER"));
	}else{
		setTargetHotend(0);

        // notify the user of *all* the axes previously affected, not just the last one
        lcd_update_enable(false);
        lcd_clear();
        crashdet_fmt_error(msg, crashDet_axes);
        crashDet_axes = 0;
        lcd_print(msg);

        // ask whether to resume printing
        lcd_puts_at_P(0, 1, _T(MSG_RESUME_PRINT));
        lcd_putc('?');
        uint8_t yesno = lcd_show_yes_no_and_wait(false, LCD_LEFT_BUTTON_CHOICE);
		if (yesno == LCD_LEFT_BUTTON_CHOICE)
		{
			enquecommand_P(PSTR("CRASH_RECOVER"));
		}
		else // LCD_MIDDLE_BUTTON_CHOICE
		{
			enquecommand_P(PSTR("CRASH_CANCEL"));
		}
	}
}

void crashdet_recover()
{
	if (!printingIsPaused()) crashdet_restore_print_and_continue();
	crashdet_use_eeprom_setting();
}

/// Crash detection cancels the print
void crashdet_cancel() {
    // Restore crash detection
    crashdet_use_eeprom_setting();

    // Abort the print
    print_stop();
}

#endif //TMC2130

void failstats_reset_print()
{
	eeprom_update_byte_notify((uint8_t *)EEPROM_CRASH_COUNT_X, 0);
	eeprom_update_byte_notify((uint8_t *)EEPROM_CRASH_COUNT_Y, 0);
	eeprom_update_byte_notify((uint8_t *)EEPROM_FERROR_COUNT, 0);
	eeprom_update_byte_notify((uint8_t *)EEPROM_POWER_COUNT, 0);
	eeprom_update_byte_notify((uint8_t *)EEPROM_MMU_FAIL, 0);
	eeprom_update_byte_notify((uint8_t *)EEPROM_MMU_LOAD_FAIL, 0);
}

void watchdogEarlyDisable(void) {
    // Regardless if the watchdog support is enabled or not, disable the watchdog very early
    // after the program starts since there's no danger in doing this.
    // The reason for this is because old bootloaders might not handle the watchdog timer at all,
    // leaving it enabled when jumping to the program. This could cause another watchdog reset
    // during setup() if not handled properly. So to avoid any issue of this kind, stop the
    // watchdog timer manually.
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      wdt_reset();
      MCUSR &= ~_BV(WDRF);
      wdt_disable();
    }
}

void softReset(void) {
    cli();
#ifdef WATCHDOG
    // If the watchdog support is enabled, use that for resetting. The timeout value is customized
    // for each board since the miniRambo ships with a bootloader which doesn't properly handle the
    // WDT. In order to avoid bootlooping, the watchdog is set to a value large enough for the
    // usual timeout of the bootloader to pass.
    wdt_enable(WATCHDOG_SOFT_RESET_VALUE);
#else
    #warning WATCHDOG not defined. See the following comment for more details about the implications
    // In case the watchdog is not enabled, the reset is acomplished by jumping to the bootloader
    // vector manually. This however is somewhat dangerous since the peripherals don't get reset
    // by this operation. Considering this is not going to be used in any production firmware,
    // it can be left as is and just be cautious with it. The only way to accomplish a peripheral
    // reset is by an external reset, by a watchdog reset or by a power cycle. All of these options
    // can't be accomplished just from software. One way to minimize the dangers of this is by
    // setting all dangerous pins to INPUT before jumping to the bootloader, but that still doesn't
    // reset other peripherals such as UART, timers, INT, PCINT, etc...
    asm volatile("jmp 0x3E000");
#endif
    while(1);
}


#ifdef MESH_BED_LEVELING
   enum MeshLevelingState { MeshReport, MeshStart, MeshNext, MeshSet };
#endif


static void factory_reset_stats(){
    eeprom_update_dword_notify((uint32_t *)EEPROM_TOTALTIME, 0);
    eeprom_update_dword_notify((uint32_t *)EEPROM_FILAMENTUSED, 0);

    failstats_reset_print();

    eeprom_update_word_notify((uint16_t *)EEPROM_CRASH_COUNT_X_TOT, 0);
    eeprom_update_word_notify((uint16_t *)EEPROM_CRASH_COUNT_Y_TOT, 0);
    eeprom_update_word_notify((uint16_t *)EEPROM_FERROR_COUNT_TOT, 0);
    eeprom_update_word_notify((uint16_t *)EEPROM_POWER_COUNT_TOT, 0);

    eeprom_update_word_notify((uint16_t *)EEPROM_MMU_FAIL_TOT, 0);
    eeprom_update_word_notify((uint16_t *)EEPROM_MMU_LOAD_FAIL_TOT, 0);
    eeprom_update_dword_notify((uint32_t *)EEPROM_MMU_MATERIAL_CHANGES, 0);
}

// Factory reset function
// This function is used to erase parts or whole EEPROM memory which is used for storing calibration and and so on.
// Level input parameter sets depth of reset
static void factory_reset(char level)
{
	//lcd_clear();
	//Sound_MakeCustom(100,0,false);
	switch (level) {

	case 0: // Level 0: Language reset
		//lang_reset();
		break;

	case 1: //Level 1: Reset statistics
		factory_reset_stats();
		//lcd_menu_statistics();
		break;

	case 2: // Level 2: Prepare for shipping
		factory_reset_stats();
		// FALLTHRU

	case 3: // Level 3: Preparation after being serviced
		// Force language selection at the next boot up.
		//lang_reset();

		// Force the wizard in "Follow calibration flow" mode at the next boot up
		//calibration_status_clear(CALIBRATION_FORCE_PREP);
		eeprom_write_byte((uint8_t*)EEPROM_WIZARD_ACTIVE, 2);
		//farm_disable();

		break;

	case 4:
		//menu_progressbar_init(EEPROM_TOP, PSTR("ERASING all data"));
		// Erase EEPROM
		for (uint16_t i = 0; i < EEPROM_TOP; i++) {
			eeprom_update_byte((uint8_t*)i, 0xFF);
			//menu_progressbar_update(i);
		}
		//menu_progressbar_finish();
		softReset();
		break;
	default:
		break;
	}
}

static FILE _uartout;
#define uartout (&_uartout)

int uart_putchar(char c, FILE *)
{
	MYSERIAL.write(c);
	return 0;
}

void factory_reset() 
{
	KEEPALIVE_STATE(PAUSED_FOR_USER);
	if (!READ(BTN_ENC))
	{
		_delay_ms(1000);
		if (!READ(BTN_ENC))
		{
			//lcd_clear();

			//lcd_puts_P(PSTR("Factory RESET"));

			SET_OUTPUT(BEEPER);
			if(eSoundMode!=e_SOUND_MODE_SILENT)
				WRITE(BEEPER, HIGH);

			while (!READ(BTN_ENC));

			WRITE(BEEPER, LOW);

			_delay_ms(2000);

			char level = reset_menu();
			factory_reset(level);

			switch (level) {
			case 0:
			case 1:
			case 2:
			case 3:
			case 4: _delay_ms(0); break;
			}

		}
	}
	KEEPALIVE_STATE(IN_HANDLER);
}

uint8_t check_printer_version()
{
	uint8_t version_changed = 0;
	uint16_t printer_type = eeprom_init_default_word((uint16_t*)EEPROM_PRINTER_TYPE, PRINTER_TYPE);
	uint16_t motherboard = eeprom_init_default_word((uint16_t*)EEPROM_BOARD_TYPE, MOTHERBOARD);

	if (printer_type != PRINTER_TYPE) version_changed |= 0b10;
	if (motherboard != MOTHERBOARD) version_changed |= 0b01;
	return version_changed;
}

#ifdef BOOTAPP
#include "bootapp.h" //bootloader support
#endif //BOOTAPP

static void fw_crash_init()
{
#ifdef XFLASH_DUMP
    dump_crash_reason crash_reason;
    if(xfdump_check_state(&crash_reason))
    {
        // always signal to the host that a dump is available for retrieval
        puts_P(_N("//action:dump_available"));

#ifdef EMERGENCY_DUMP
        if(crash_reason != dump_crash_reason::manual &&
           eeprom_read_byte((uint8_t*)EEPROM_FW_CRASH_FLAG) != 0xFF)
        {
            lcd_show_fullscreen_message_and_wait_P(
                    _n("FW crash detected! "
                       "You can continue printing. "
                       "Debug data available for analysis. "
                       "Contact support to submit details."));
        }
#endif
#endif
    }
    // prevent crash prompts to reappear once acknowledged
    eeprom_update_byte_notify((uint8_t*)EEPROM_FW_CRASH_FLAG, 0xFF);
}

#define KILL_PENDING_FLAG 0x42

static void fw_kill_init() {
    if (eeprom_read_byte((uint8_t*)EEPROM_KILL_PENDING_FLAG) == KILL_PENDING_FLAG) {
        // clear pending message event
        eeprom_write_byte_notify((uint8_t*)EEPROM_KILL_PENDING_FLAG, EEPROM_EMPTY_VALUE);

        // display the kill message
        PGM_P kill_msg = (PGM_P)eeprom_read_word((uint16_t*)EEPROM_KILL_MESSAGE);
        lcd_show_fullscreen_message_and_wait_P(kill_msg);
    }
}


static void xflash_err_msg()
{
    puts_P(_n("XFLASH not responding."));
    lcd_show_fullscreen_message_and_wait_P(_n("External SPI flash\nXFLASH is not res-\nponding. Language\nswitch unavailable."));
}

// "Setup" function is called by the Arduino framework on startup.
// Before startup, the Timers-functions (PWM)/Analog RW and HardwareSerial provided by the Arduino-code 
// are initialized by the main() routine provided by the Arduino framework.
void setup()
{
  watchdogEarlyDisable();
  
	timer2_init(); // enables functional millis

	//ultralcd_init();

	spi_init();

	//lcd_splash();
  //Sound_Init();                                // also guarantee "SET_OUTPUT(BEEPER)"

  selectedSerialPort = eeprom_init_default_byte((uint8_t *)EEPROM_SECOND_SERIAL_ACTIVE, 0);
	MYSERIAL.begin(BAUDRATE);
	fdev_setup_stream(uartout, uart_putchar, NULL, _FDEV_SETUP_WRITE); //setup uart out stream
	stdout = uartout;

  bool xflash_success = xflash_init();
	uint8_t optiboot_status = 1;
	if (xflash_success)
	{
		optiboot_status = optiboot_xflash_enter();
#if (LANG_MODE != 0) //secondary language support
        update_sec_lang_from_external_flash();
#endif //(LANG_MODE != 0)
	}

	setup_killpin();
	setup_powerhold();

	if ((optiboot_status != 0) || (selectedSerialPort != 0))
		SERIAL_PROTOCOLLNPGM("start");
	SERIAL_ECHO_START;

	puts_P(PSTR("Matt custom firmware"));

	// Check startup - does nothing if bootloader sets MCUSR to 0
	byte mcu = MCUSR;
	if (mcu & 1) puts_P(MSG_POWERUP);
	if (mcu & 2) puts_P(MSG_EXTERNAL_RESET);
	if (mcu & 4) puts_P(MSG_BROWNOUT_RESET);
	if (mcu & 8) puts_P(MSG_WATCHDOG_RESET);
	if (mcu & 32) puts_P(MSG_SOFTWARE_RESET);
	MCUSR = 0;

	SERIAL_ECHO_START;
	SERIAL_ECHORPGM(_n(" Free Memory: "));////MSG_FREE_MEMORY
	SERIAL_ECHO(freeMemory());
	SERIAL_ECHORPGM(_n("  PlannerBufferBytes: "));////MSG_PLANNER_BUFFER_BYTES
	SERIAL_ECHOLN((int)sizeof(block_t)*BLOCK_BUFFER_SIZE);
	//lcd_update_enable(false); // why do we need this?? - andre
	// loads data from EEPROM if available else uses defaults (and resets step acceleration rate)
	
	uint8_t hw_changed = check_printer_version();
	if (!(hw_changed & 0b10)) { //if printer version wasn't changed, check for eeprom version and retrieve settings from eeprom in case that version wasn't changed
	} 
	else { //printer version was changed so use default settings 
		Config_ResetDefault();
	}

    // Initialize pwm/temperature loops
    //soft_pwm_init();
    //temp_mgr_init();

	plan_init();  // Initialize planner;

	factory_reset();

	tmc2130_mode = TMC2130_MODE_NORMAL;

  tmc2130_sg_stop_on_crash = eeprom_init_default_byte((uint8_t*)EEPROM_CRASH_DET, farm_mode ? false : true);

	/*if (tmc2130_sg_stop_on_crash) {
    puts_P(_N("CrashDetect ENABLED!"));
	} else {
	    puts_P(_N("CrashDetect DISABLED"));
	}*/

#ifdef TMC2130_LINEARITY_CORRECTION
#ifdef TMC2130_LINEARITY_CORRECTION_XYZ
	tmc2130_wave_fac[X_AXIS] = eeprom_read_byte((uint8_t*)EEPROM_TMC2130_WAVE_X_FAC);
	tmc2130_wave_fac[Y_AXIS] = eeprom_read_byte((uint8_t*)EEPROM_TMC2130_WAVE_Y_FAC);
	tmc2130_wave_fac[Z_AXIS] = eeprom_read_byte((uint8_t*)EEPROM_TMC2130_WAVE_Z_FAC);
#endif //TMC2130_LINEARITY_CORRECTION_XYZ
	tmc2130_wave_fac[E_AXIS] = eeprom_read_byte((uint8_t*)EEPROM_TMC2130_WAVE_E_FAC);
	if (tmc2130_wave_fac[X_AXIS] == 0xff) tmc2130_wave_fac[X_AXIS] = 0;
	if (tmc2130_wave_fac[Y_AXIS] == 0xff) tmc2130_wave_fac[Y_AXIS] = 0;
	if (tmc2130_wave_fac[Z_AXIS] == 0xff) tmc2130_wave_fac[Z_AXIS] = 0;
	if (tmc2130_wave_fac[E_AXIS] == 0xff) tmc2130_wave_fac[E_AXIS] = 0;
#endif //TMC2130_LINEARITY_CORRECTION

#ifdef TMC2130_VARIABLE_RESOLUTION
	tmc2130_mres[X_AXIS] = tmc2130_usteps2mres(cs.axis_ustep_resolution[X_AXIS]);
	tmc2130_mres[Y_AXIS] = tmc2130_usteps2mres(cs.axis_ustep_resolution[Y_AXIS]);
	tmc2130_mres[Z_AXIS] = tmc2130_usteps2mres(cs.axis_ustep_resolution[Z_AXIS]);
	tmc2130_mres[E_AXIS] = tmc2130_usteps2mres(cs.axis_ustep_resolution[E_AXIS]);
#else //TMC2130_VARIABLE_RESOLUTION
	tmc2130_mres[X_AXIS] = tmc2130_usteps2mres(TMC2130_USTEPS_XY);
	tmc2130_mres[Y_AXIS] = tmc2130_usteps2mres(TMC2130_USTEPS_XY);
	tmc2130_mres[Z_AXIS] = tmc2130_usteps2mres(TMC2130_USTEPS_Z);
	tmc2130_mres[E_AXIS] = tmc2130_usteps2mres(TMC2130_USTEPS_E);
#endif //TMC2130_VARIABLE_RESOLUTION

	st_init();    // Initialize stepper, this enables interrupts!
  
	update_mode_profile();
	tmc2130_init(TMCInitParams(false, FarmOrUserECool() ));
    
	setup_photpin();

	// Reset the machine correction matrix.
	// It does not make sense to load the correction matrix until the machine is homed.
	world2machine_reset();

  // Initialize current_position accounting for software endstops to
  // avoid unexpected initial shifts on the first move
  clamp_to_software_endstops(current_position);
  plan_set_position_curposXYZE();

  // Show the xflash error message now that serial, lcd and encoder are available
  if (!xflash_success)
      xflash_err_msg();

  // report kill() events
  fw_kill_init();

	setup_homepin();

#if defined(Z_AXIS_ALWAYS_ON)
    enable_z();
#endif

  eeprom_init();
  // In the future, somewhere here would one compare the current firmware version against the firmware version stored in the EEPROM.
  // If they differ, an update procedure may need to be performed. At the end of this block, the current firmware version
  // is being written into the EEPROM, so the update procedure will be triggered only once.

	eeprom_init_default_byte((uint8_t*)EEPROM_TEMP_CAL_ACTIVE, 0);

	if (eeprom_read_byte((uint8_t*)EEPROM_CALIBRATION_STATUS_PINDA) == 255) {
		//eeprom_write_byte_notify((uint8_t*)EEPROM_CALIBRATION_STATUS_PINDA, 0);
		eeprom_update_byte_notify((uint8_t*)EEPROM_CALIBRATION_STATUS_PINDA, 1);
		int16_t z_shift = 0;
		for (uint8_t i = 0; i < 5; i++) {
			eeprom_update_word_notify((uint16_t*)EEPROM_PROBE_TEMP_SHIFT + i, z_shift);
		}
		eeprom_update_byte_notify((uint8_t*)EEPROM_TEMP_CAL_ACTIVE, 0);
	}
	eeprom_init_default_byte((uint8_t*)EEPROM_UVLO, PowerPanic::NO_PENDING_RECOVERY);
	eeprom_init_default_byte((uint8_t*)EEPROM_UVLO_Z_LIFTED, 0);
	eeprom_init_default_byte((uint8_t*)EEPROM_SD_SORT, 0);

	//mbl_mode_init();
	mbl_settings_init();
	eeprom_init_default_byte((uint8_t*)EEPROM_MMU_STEALTH, 1);

#if (!defined(DEBUG_DISABLE_FANCHECK) && defined(FANCHECK) && defined(TACH_1) && (TACH_1 >-1))
	setup_fan_interrupt();
#endif //DEBUG_DISABLE_FANCHECK

#ifndef DEBUG_DISABLE_STARTMSGS
  KEEPALIVE_STATE(PAUSED_FOR_USER);


  KEEPALIVE_STATE(IN_PROCESS);
#endif //DEBUG_DISABLE_STARTMSGS
    //lcd_update_enable(true);
    //lcd_clear();
    //lcd_update(2);

#ifdef TMC2130
    tmc2130_home_origin[X_AXIS] = eeprom_init_default_byte((uint8_t*)EEPROM_TMC2130_HOME_X_ORIGIN, 0);
	tmc2130_home_bsteps[X_AXIS] = eeprom_init_default_byte((uint8_t*)EEPROM_TMC2130_HOME_X_BSTEPS, 48);
	tmc2130_home_fsteps[X_AXIS] = eeprom_init_default_byte((uint8_t*)EEPROM_TMC2130_HOME_X_FSTEPS, 48);

	tmc2130_home_origin[Y_AXIS] = eeprom_init_default_byte((uint8_t*)EEPROM_TMC2130_HOME_Y_ORIGIN, 0);
	tmc2130_home_bsteps[Y_AXIS] = eeprom_init_default_byte((uint8_t*)EEPROM_TMC2130_HOME_Y_BSTEPS, 48);
	tmc2130_home_fsteps[Y_AXIS] = eeprom_init_default_byte((uint8_t*)EEPROM_TMC2130_HOME_Y_FSTEPS, 48);

	tmc2130_home_enabled = eeprom_init_default_byte((uint8_t*)EEPROM_TMC2130_HOME_ENABLED, 0);
#endif //TMC2130

  // report crash failures
  fw_crash_init();

  fCheckModeInit();
  KEEPALIVE_STATE(NOT_BUSY);
#ifdef WATCHDOG
  wdt_enable(WDTO_4S);
#ifdef EMERGENCY_HANDLERS
  WDTCSR |= (1 << WDIE);
#endif //EMERGENCY_HANDLERS
#endif //WATCHDOG
}

static inline void crash_and_burn(dump_crash_reason reason)
{
    WRITE(BEEPER, HIGH);
    eeprom_update_byte_notify((uint8_t*)EEPROM_FW_CRASH_FLAG, (uint8_t)reason);
#ifdef EMERGENCY_DUMP
    xfdump_full_dump_and_reset(reason);
#elif defined(EMERGENCY_SERIAL_DUMP)
    if(emergency_serial_dump)
        serial_dump_and_reset(reason);
#endif
    softReset();
}

#ifdef EMERGENCY_HANDLERS
#ifdef WATCHDOG
ISR(WDT_vect)
{
    crash_and_burn(dump_crash_reason::watchdog);
}
#endif

ISR(BADISR_vect)
{
    crash_and_burn(dump_crash_reason::bad_isr);
}
#endif //EMERGENCY_HANDLERS

void stack_error() {
    crash_and_burn(dump_crash_reason::stack_error);
}


/**
 * Output autoreport values according to features requested in M155
 */
#if defined(AUTO_REPORT)
void host_autoreport()
{
    if (autoReportFeatures.TimerExpired())
    {
        if(autoReportFeatures.Temp()){
            gcode_M105();
        }
        if(autoReportFeatures.Pos()){
            gcode_M114();
        }
#if defined(AUTO_REPORT) && (defined(FANCHECK) && (((defined(TACH_0) && (TACH_0 >-1)) || (defined(TACH_1) && (TACH_1 > -1)))))
        if(autoReportFeatures.Fans()){
            gcode_M123();
        }
#endif //AUTO_REPORT and (FANCHECK and TACH_0 or TACH_1)
        autoReportFeatures.TimerStart();
    }
}
#endif //AUTO_REPORT


/**
* Output a "busy" message at regular intervals
* while the machine is not accepting commands.
*/
void host_keepalive() {
#ifndef HOST_KEEPALIVE_FEATURE
  return;
#endif //HOST_KEEPALIVE_FEATURE
  //if (farm_mode) return;
  long ms = _millis();

  if (host_keepalive_interval && busy_state != NOT_BUSY) {
    if ((ms - prev_busy_signal_ms) < (long)(1000L * host_keepalive_interval)) return;
     switch (busy_state) {
      case IN_HANDLER:
      case IN_PROCESS:
        SERIAL_ECHO_START;
        SERIAL_ECHOLNPGM("busy: processing");
        break;
      case PAUSED_FOR_USER:
        SERIAL_ECHO_START;
        SERIAL_ECHOLNPGM("busy: paused for user");
        break;
      case PAUSED_FOR_INPUT:
        SERIAL_ECHO_START;
        SERIAL_ECHOLNPGM("busy: paused for input");
        break;
      default:
	break;
    }
  }
  prev_busy_signal_ms = ms;
}


// The loop() function is called in an endless loop by the Arduino framework from the default main() routine.
// Before loop(), the setup() function is called by the main() routine.
void loop()
{
    // Reset a previously aborted command, we can now start processing motion again
    planner_aborted = false;

    if(Stopped) {
        // Currently Stopped (possibly due to an error) and not accepting new serial commands.
        // Signal to the host that we're currently busy waiting for supervision.
        KEEPALIVE_STATE(PAUSED_FOR_USER);
    } else {
        // Printer is available for processing, reset state
        KEEPALIVE_STATE(NOT_BUSY);
    }

	if (printingIsPaused() && saved_printing_type == PowerPanic::PRINT_TYPE_HOST) { //keep believing that usb is being printed. Prevents accessing dangerous menus while pausing.
		usb_timer.start();
	}
	else if (usb_timer.expired(USB_TIMER_TIMEOUT)) { //just need to check if it expired. Nothing else is needed to be done.
        SetPrinterState(PrinterState::HostPrintingFinished); //set printer state to show LCD menu after finished SD print
	}
    
    {

        get_command();

  if(buflen)
  {
    cmdbuffer_front_already_processed = false;
    
    process_commands();

    if (! cmdbuffer_front_already_processed && buflen)
    {
      // ptr points to the start of the block currently being processed.
      // The first character in the block is the block type.      
      char *ptr = cmdbuffer + bufindr;
      if (*ptr == CMDBUFFER_CURRENT_TYPE_SDCARD) {
        // To support power panic, move the length of the command on the SD card to a planner buffer.
        union {
          struct {
              char lo;
              char hi;
          } lohi;
          uint16_t value;
        } sdlen;
        sdlen.value = 0;
        {
          // This block locks the interrupts globally for 3.25 us,
          // which corresponds to a maximum repeat frequency of 307.69 kHz.
          // This blocking is safe in the context of a 10kHz stepper driver interrupt
          // or a 115200 Bd serial line receive interrupt, which will not trigger faster than 12kHz.
          cli();
          // Reset the command to something, which will be ignored by the power panic routine,
          // so this buffer length will not be counted twice.
          *ptr ++ = CMDBUFFER_CURRENT_TYPE_TO_BE_REMOVED;
          // Extract the current buffer length.
          sdlen.lohi.lo = *ptr ++;
          sdlen.lohi.hi = *ptr;
          // and pass it to the planner queue.
          planner_add_sd_length(sdlen.value);
          sei();
        }
	  }
	  else if((*ptr == CMDBUFFER_CURRENT_TYPE_USB_WITH_LINENR) && !IS_SD_PRINTING){ 
		  
		  cli();
          *ptr ++ = CMDBUFFER_CURRENT_TYPE_TO_BE_REMOVED;
          // and one for each command to previous block in the planner queue.
          planner_add_sd_length(1);
          sei();
	  }
      // Now it is safe to release the already processed command block. If interrupted by the power panic now,
      // this block's SD card length will not be counted twice as its command type has been replaced 
      // by CMDBUFFER_CURRENT_TYPE_TO_BE_REMOVED.
      cmdqueue_pop_front();
    }
	host_keepalive();
  }
}
  //check heater every n milliseconds
  //manage_heater();
  manage_inactivity(printingIsPaused());
  checkHitEndstops();
  //lcd_update(0);
#ifdef TMC2130
	tmc2130_check_overtemp();
	if (tmc2130_sg_crash)
	{
		uint8_t crash = tmc2130_sg_crash;
		tmc2130_sg_crash = 0;
//		crashdet_stop_and_save_print();
		switch (crash)
		{
		case 1: enquecommand_P((PSTR("CRASH_DETECTEDX"))); break;
		case 2: enquecommand_P((PSTR("CRASH_DETECTEDY"))); break;
		case 3: enquecommand_P((PSTR("CRASH_DETECTEDXY"))); break;
		}
	}
#endif //TMC2130
	//MMU2::mmu2.mmu_loop();
}

#define DEFINE_PGM_READ_ANY(type, reader)       \
    static inline type pgm_read_any(const type *p)  \
    { return pgm_read_##reader##_near(p); }

DEFINE_PGM_READ_ANY(float,       float);
DEFINE_PGM_READ_ANY(signed char, byte);

#define XYZ_CONSTS_FROM_CONFIG(type, array, CONFIG) \
static const PROGMEM type array##_P[3] =        \
    { X_##CONFIG, Y_##CONFIG, Z_##CONFIG };     \
static inline type array(uint8_t axis)              \
    { return pgm_read_any(&array##_P[axis]); }  \
type array##_ext(uint8_t axis)                      \
    { return pgm_read_any(&array##_P[axis]); }

XYZ_CONSTS_FROM_CONFIG(float, base_min_pos,    MIN_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_max_pos,    MAX_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_home_pos,   HOME_POS);
XYZ_CONSTS_FROM_CONFIG(float, max_length,      MAX_LENGTH);
XYZ_CONSTS_FROM_CONFIG(float, home_retract_mm, HOME_RETRACT_MM);
XYZ_CONSTS_FROM_CONFIG(signed char, home_dir,  HOME_DIR);

static void axis_is_at_home(uint8_t axis) {
  current_position[axis] = base_home_pos(axis) + cs.add_homing[axis];
  min_pos[axis] =          base_min_pos(axis) + cs.add_homing[axis];
  max_pos[axis] =          base_max_pos(axis) + cs.add_homing[axis];
}

bool check_commands() {
	bool end_command_found = false;
	
		while (buflen)
		{
		if ((code_seen_P(MSG_M84)) || (code_seen_P(PSTR("M 84")))) end_command_found = true;
		if (!cmdbuffer_front_already_processed)
			 cmdqueue_pop_front();
		cmdbuffer_front_already_processed = false;
		}
	return end_command_found;
	
}

/// @brief Safely move Z-axis by distance delta (mm)
/// @param delta travel distance in mm
/// @returns The actual travel distance in mm. Endstop may limit the requested move.
float raise_z(float delta)
{
    float travel_z = current_position[Z_AXIS];

    // Prepare to move Z axis
    current_position[Z_AXIS] += delta;

#if defined(Z_MIN_PIN) && (Z_MIN_PIN > -1) && !defined(DEBUG_DISABLE_ZMINLIMIT)
    bool z_min_endstop = (READ(Z_MIN_PIN) != Z_MIN_ENDSTOP_INVERTING);
#else
    bool z_min_endstop = false;
#endif

    if (axis_known_position[Z_AXIS] || z_min_endstop)
    {
        // current position is known or very low, it's safe to raise Z
        clamp_to_software_endstops(current_position);
        plan_buffer_line_curposXYZE(max_feedrate[Z_AXIS]);
        st_synchronize();

        // Get the final travel distance
        travel_z = current_position[Z_AXIS] - travel_z;
    } else {
        // ensure Z is powered in normal mode to overcome initial load
        enable_z();
        st_synchronize();

        // rely on crashguard to limit damage
        bool z_endstop_enabled = enable_z_endstop(true);
#ifdef TMC2130
        tmc2130_home_enter(Z_AXIS_MASK);
#endif //TMC2130
        plan_buffer_line_curposXYZE(homing_feedrate[Z_AXIS] / 60);
        st_synchronize();

        // Get the final travel distance
        travel_z = st_get_position_mm(Z_AXIS) - travel_z;
#ifdef TMC2130
        if (endstop_z_hit_on_purpose())
        {
            // not necessarily exact, but will avoid further vertical moves
            current_position[Z_AXIS] = max_pos[Z_AXIS];
            plan_set_position_curposXYZE();
        }
        tmc2130_home_exit();
#endif //TMC2130
        enable_z_endstop(z_endstop_enabled);
    }

    return travel_z;
}

// raise_z_above: slowly raise Z to the requested height
//
// contrarily to a simple move, this function will carefully plan a move
// when the current Z position is unknown. In such cases, stallguard is
// enabled and will prevent prolonged pushing against the Z tops
void raise_z_above(float target)
{
    if (current_position[Z_AXIS] >= target)
        return;

    // Use absolute value in case the current position is unknown
    raise_z(fabs(current_position[Z_AXIS] - target));
}


#ifdef TMC2130
bool calibrate_z_auto()
{
	//lcd_display_message_fullscreen_P(_T(MSG_CALIBRATE_Z_AUTO));
	lcd_clear();
	lcd_puts_at_P(0, 1, _T(MSG_CALIBRATE_Z_AUTO));
	bool endstops_enabled = enable_endstops(true);
	int axis_up_dir = -home_dir(Z_AXIS);
	tmc2130_home_enter(Z_AXIS_MASK);
	current_position[Z_AXIS] = 0;
	plan_set_position_curposXYZE();
	set_destination_to_current();
	destination[Z_AXIS] += (1.1 * max_length(Z_AXIS) * axis_up_dir);
	feedrate = homing_feedrate[Z_AXIS];
	plan_buffer_line_destinationXYZE(feedrate / 60);
	st_synchronize();
	//	current_position[axis] = 0;
	//	plan_set_position_curposXYZE();
	tmc2130_home_exit();
	enable_endstops(false);
	current_position[Z_AXIS] = 0;
	plan_set_position_curposXYZE();
	set_destination_to_current();
	destination[Z_AXIS] += 10 * axis_up_dir; //10mm up
	feedrate = homing_feedrate[Z_AXIS] / 2;
	plan_buffer_line_destinationXYZE(feedrate / 60);
	st_synchronize();
	enable_endstops(endstops_enabled);
	if (PRINTER_TYPE == PRINTER_MK3) {
		current_position[Z_AXIS] = Z_MAX_POS + 2.0;
	}
	else {
		current_position[Z_AXIS] = Z_MAX_POS + 9.0;
	}
	plan_set_position_curposXYZE();
	return true;
}
#endif //TMC2130

#ifdef TMC2130
void check_Z_crash(void)
{
	if (!READ(Z_TMC2130_DIAG)) { //Z crash
		FORCE_HIGH_POWER_END;
		current_position[Z_AXIS] = 0;
		plan_set_position_curposXYZE();
		current_position[Z_AXIS] += MESH_HOME_Z_SEARCH;
		plan_buffer_line_curposXYZE(max_feedrate[Z_AXIS]);
		st_synchronize();
		kill(_T(MSG_BED_LEVELING_FAILED_POINT_LOW));
	}
}
#endif //TMC2130

float __attribute__((noinline)) get_feedrate_mm_s(const float feedrate_mm_min) {
  return feedrate_mm_min / 60.f;
}

#ifdef TMC2130
void homeaxis(uint8_t axis, uint8_t cnt, uint8_t* pstep)
#else
void homeaxis(uint8_t axis, uint8_t cnt)
#endif //TMC2130
{
	bool endstops_enabled  = enable_endstops(true); //RP: endstops should be allways enabled durring homing
#define HOMEAXIS_DO(LETTER) \
((LETTER##_MIN_PIN > -1 && LETTER##_HOME_DIR==-1) || (LETTER##_MAX_PIN > -1 && LETTER##_HOME_DIR==1))
    if ((axis==X_AXIS)?HOMEAXIS_DO(X):(axis==Y_AXIS)?HOMEAXIS_DO(Y):0)
	{
        int axis_home_dir = home_dir(axis);
        feedrate = homing_feedrate[axis];
        float feedrate_mm_s = get_feedrate_mm_s(feedrate);

#ifdef TMC2130
    	tmc2130_home_enter(X_AXIS_MASK << axis);
#endif //TMC2130


        // Move away a bit, so that the print head does not touch the end position,
        // and the following movement to endstop has a chance to achieve the required velocity
        // for the stall guard to work.
        current_position[axis] = 0;
        plan_set_position_curposXYZE();
		set_destination_to_current();
//        destination[axis] = 11.f;
        destination[axis] = -3.f * axis_home_dir;
        plan_buffer_line_destinationXYZE(feedrate_mm_s);
        st_synchronize();
        // Move away from the possible collision with opposite endstop with the collision detection disabled.
        endstops_hit_on_purpose();
        enable_endstops(false);
        current_position[axis] = 0;
        plan_set_position_curposXYZE();
        destination[axis] = 1. * axis_home_dir;
        plan_buffer_line_destinationXYZE(feedrate_mm_s);
        st_synchronize();
        // Now continue to move up to the left end stop with the collision detection enabled.
        enable_endstops(true);
        destination[axis] = 1.1 * axis_home_dir * max_length(axis);
        plan_buffer_line_destinationXYZE(feedrate_mm_s);
        st_synchronize();
		for (uint8_t i = 0; i < cnt; i++)
		{
			// Move away from the collision to a known distance from the left end stop with the collision detection disabled.
			endstops_hit_on_purpose();
			enable_endstops(false);
			current_position[axis] = 0;
			plan_set_position_curposXYZE();
			destination[axis] = -10.f * axis_home_dir;
			plan_buffer_line_destinationXYZE(feedrate_mm_s);
			st_synchronize();
			endstops_hit_on_purpose();
			// Now move left up to the collision, this time with a repeatable velocity.
			enable_endstops(true);
			destination[axis] = 11.f * axis_home_dir;
#ifdef TMC2130
			feedrate = homing_feedrate[axis];
#else //TMC2130
			feedrate = homing_feedrate[axis] / 2;
      feedrate_mm_s = get_feedrate_mm_s(feedrate);
#endif //TMC2130
			plan_buffer_line_destinationXYZE(feedrate_mm_s);
			st_synchronize();
#ifdef TMC2130
			uint16_t mscnt = tmc2130_rd_MSCNT(axis);
			if (pstep) pstep[i] = mscnt >> 4;
			printf_P(PSTR("%3d step=%2d mscnt=%4d\n"), i, mscnt >> 4, mscnt);
#endif //TMC2130
		}
		endstops_hit_on_purpose();
		enable_endstops(false);

#ifdef TMC2130
		uint8_t orig = tmc2130_home_origin[axis];
		uint8_t back = tmc2130_home_bsteps[axis];
		if (tmc2130_home_enabled && (orig <= 63))
		{
			tmc2130_goto_step(axis, orig, 2, 1000, tmc2130_get_res(axis));
			if (back > 0)
				tmc2130_do_steps(axis, back, -axis_home_dir, 1000);
		}
		else
			tmc2130_do_steps(axis, 8, -axis_home_dir, 1000);
		tmc2130_home_exit();
#endif //TMC2130

        axis_is_at_home(axis);
        axis_known_position[axis] = true;
        // Move from minimum
#ifdef TMC2130
        float dist = - axis_home_dir * 0.01f * tmc2130_home_fsteps[axis];
#else //TMC2130
        float dist = - axis_home_dir * 0.01f * 64;
#endif //TMC2130
        current_position[axis] -= dist;
        plan_set_position_curposXYZE();
        current_position[axis] += dist;
        destination[axis] = current_position[axis];
        plan_buffer_line_destinationXYZE(0.5f*feedrate_mm_s);
        st_synchronize();

   		feedrate = 0.0;
    }
    else if ((axis==Z_AXIS)?HOMEAXIS_DO(Z):0)
	{
#ifdef TMC2130
		FORCE_HIGH_POWER_START;
#endif	
        int axis_home_dir = home_dir(axis);
        current_position[axis] = 0;
        plan_set_position_curposXYZE();
        destination[axis] = 1.5 * max_length(axis) * axis_home_dir;
        feedrate = homing_feedrate[axis];
        float feedrate_mm_s = get_feedrate_mm_s(feedrate);
        plan_buffer_line_destinationXYZE(feedrate_mm_s);
        st_synchronize();
#ifdef TMC2130
        check_Z_crash();
#endif //TMC2130
        current_position[axis] = 0;
        plan_set_position_curposXYZE();
        destination[axis] = -home_retract_mm(axis) * axis_home_dir;
        plan_buffer_line_destinationXYZE(feedrate_mm_s);
        st_synchronize();
        destination[axis] = 2*home_retract_mm(axis) * axis_home_dir;
        feedrate = homing_feedrate[axis] / 2;
        feedrate_mm_s = get_feedrate_mm_s(feedrate);
        plan_buffer_line_destinationXYZE(feedrate_mm_s);
        st_synchronize();
#ifdef TMC2130
        check_Z_crash();
#endif //TMC2130
        axis_is_at_home(axis);
        destination[axis] = current_position[axis];
        feedrate = 0.0;
        endstops_hit_on_purpose();
        axis_known_position[axis] = true;
#ifdef TMC2130
		FORCE_HIGH_POWER_END;
#endif	
    }
    enable_endstops(endstops_enabled);
}

/**/
void home_xy()
{
    set_destination_to_current();
    homeaxis(X_AXIS);
    homeaxis(Y_AXIS);
    plan_set_position_curposXYZE();
    endstops_hit_on_purpose();
}

void refresh_cmd_timeout(void)
{
  previous_millis_cmd.start();
}

#ifdef TMC2130

void change_power_mode_live(uint8_t mode)
{
  // Wait for the planner queue to drain and for the stepper timer routine to reach an idle state.
		st_synchronize();
		cli();
		tmc2130_mode = mode;
		update_mode_profile();
		tmc2130_init(TMCInitParams(FarmOrUserECool()));
    // We may have missed a stepper timer interrupt due to the time spent in the tmc2130_init() routine.
    // Be safe than sorry, reset the stepper timer before re-enabling interrupts.
    st_reset_timer();
		sei();
}

void force_high_power_mode(bool start_high_power_section) {
#ifdef PSU_Delta
	if (start_high_power_section == true) enable_force_z();
#endif //PSU_Delta
	uint8_t silent;
	silent = eeprom_read_byte((uint8_t*)EEPROM_SILENT);
	if (silent == 1 || tmc2130_mode == TMC2130_MODE_SILENT) {
		//we are in silent mode, set to normal mode to enable crash detection
    change_power_mode_live((start_high_power_section == true) ? TMC2130_MODE_NORMAL : TMC2130_MODE_SILENT);
	}
}
#endif //TMC2130

void gcode_M105()
{
#if defined(TEMP_0_PIN) && TEMP_0_PIN > -1
    SERIAL_PROTOCOLPGM("T:");
    SERIAL_PROTOCOL_F(degHotend(active_extruder),1);
    SERIAL_PROTOCOLPGM(" /");
    SERIAL_PROTOCOL_F(degTargetHotend(active_extruder),1);
#if defined(TEMP_BED_PIN) && TEMP_BED_PIN > -1
    SERIAL_PROTOCOLPGM(" B:");
    SERIAL_PROTOCOL_F(degBed(),1);
    SERIAL_PROTOCOLPGM(" /");
    SERIAL_PROTOCOL_F(degTargetBed(),1);
#endif //TEMP_BED_PIN
    SERIAL_PROTOCOLPGM(" T0:");
    SERIAL_PROTOCOL_F(degHotend(active_extruder),1);
    SERIAL_PROTOCOLPGM(" /");
    SERIAL_PROTOCOL_F(degTargetHotend(active_extruder),1);
#else
    SERIAL_ERROR_START;
    SERIAL_ERRORLNRPGM(_n("No thermistors - no temperature"));////MSG_ERR_NO_THERMISTORS
#endif

    SERIAL_PROTOCOLPGM(" @:");
#ifdef EXTRUDER_WATTS
    SERIAL_PROTOCOL((EXTRUDER_WATTS * getHeaterPower(active_extruder))/127);
    SERIAL_PROTOCOLPGM("W");
#else
    SERIAL_PROTOCOL(getHeaterPower(active_extruder));
#endif

    SERIAL_PROTOCOLPGM(" B@:");
#ifdef BED_WATTS
    SERIAL_PROTOCOL((BED_WATTS * getHeaterPower(-1))/127);
    SERIAL_PROTOCOLPGM("W");
#else
    SERIAL_PROTOCOL(getHeaterPower(-1));
#endif

#ifdef PINDA_THERMISTOR
    SERIAL_PROTOCOLPGM(" P:");
    SERIAL_PROTOCOL_F(current_temperature_pinda,1);
#endif //PINDA_THERMISTOR

#ifdef AMBIENT_THERMISTOR
    SERIAL_PROTOCOLPGM(" A:");
    SERIAL_PROTOCOL_F(current_temperature_ambient,1);
#endif //AMBIENT_THERMISTOR

    SERIAL_PROTOCOLLN();
}

void gcode_M114()
{
	SERIAL_PROTOCOLPGM("X:");
	SERIAL_PROTOCOL(current_position[X_AXIS]);
	SERIAL_PROTOCOLPGM(" Y:");
	SERIAL_PROTOCOL(current_position[Y_AXIS]);
	SERIAL_PROTOCOLPGM(" Z:");
	SERIAL_PROTOCOL(current_position[Z_AXIS]);
	SERIAL_PROTOCOLPGM(" E:");
	SERIAL_PROTOCOL(current_position[E_AXIS]);

	SERIAL_PROTOCOLRPGM(_n(" Count X: "));////MSG_COUNT_X
	SERIAL_PROTOCOL(float(st_get_position(X_AXIS)) / cs.axis_steps_per_mm[X_AXIS]);
	SERIAL_PROTOCOLPGM(" Y:");
	SERIAL_PROTOCOL(float(st_get_position(Y_AXIS)) / cs.axis_steps_per_mm[Y_AXIS]);
	SERIAL_PROTOCOLPGM(" Z:");
	SERIAL_PROTOCOL(float(st_get_position(Z_AXIS)) / cs.axis_steps_per_mm[Z_AXIS]);
	SERIAL_PROTOCOLPGM(" E:");
	SERIAL_PROTOCOLLN(float(st_get_position(E_AXIS)) / cs.axis_steps_per_mm[E_AXIS]);
}

#if (defined(FANCHECK) && (((defined(TACH_0) && (TACH_0 >-1)) || (defined(TACH_1) && (TACH_1 > -1)))))
void gcode_M123()
{
  printf_P(_N("E0:%d RPM PRN1:%d RPM E0@:%u PRN1@:%u\n"), 60*fan_speed[active_extruder], 60*fan_speed[1], newFanSpeed, fanSpeed);
}
#endif //FANCHECK and TACH_0 or TACH_1

//! Detection of faulty RAMBo 1.1b boards equipped with bigger capacitors
//! at the TACH_1 pin, which causes bad detection of print fan speed.
//! Warning: This function is not to be used by ordinary users, it is here only for automated testing purposes,
//!   it may even interfere with other functions of the printer! You have been warned!
//! The test idea is to measure the time necessary to charge the capacitor.
//! So the algorithm is as follows:
//! 1. Set TACH_1 pin to INPUT mode and LOW
//! 2. Wait a few ms
//! 3. disable interrupts and measure the time until the TACH_1 pin reaches HIGH
//! Repeat 1.-3. several times
//! Good RAMBo's times are in the range of approx. 260-320 us
//! Bad RAMBo's times are approx. 260-1200 us
//! So basically we are interested in maximum time, the minima are mostly the same.
//! May be that's why the bad RAMBo's still produce some fan RPM reading, but not corresponding to reality
static void gcode_PRUSA_BadRAMBoFanTest(){
    //printf_P(PSTR("Enter fan pin test\n"));
#if !defined(DEBUG_DISABLE_FANCHECK) && defined(FANCHECK) && defined(TACH_1) && TACH_1 >-1
	fan_measuring = false; // prevent EXTINT7 breaking into the measurement
	unsigned long tach1max = 0;
	uint8_t tach1cntr = 0;
	for( /* nothing */; tach1cntr < 100; ++tach1cntr){
		//printf_P(PSTR("TACH_1: %d\n"), tach1cntr);
		SET_OUTPUT(TACH_1);
		WRITE(TACH_1, LOW);
		_delay(20); // the delay may be lower
		unsigned long tachMeasure = _micros();
		cli();
		SET_INPUT(TACH_1);
		// just wait brutally in an endless cycle until we reach HIGH
		// if this becomes a problem it may be improved to non-endless cycle
		while( READ(TACH_1) == 0 ) ;
		sei();
		tachMeasure = _micros() - tachMeasure;
		if( tach1max < tachMeasure )
		tach1max = tachMeasure;
		//printf_P(PSTR("TACH_1: %d: capacitor check time=%lu us\n"), (int)tach1cntr, tachMeasure);
	}	
	//printf_P(PSTR("TACH_1: max=%lu us\n"), tach1max);
	SERIAL_PROTOCOLPGM("RAMBo FAN ");
	if( tach1max > 500 ){
		// bad RAMBo
		SERIAL_PROTOCOLLNPGM("BAD");
	} else {
		SERIAL_PROTOCOLLNPGM("OK");
    }
	// cleanup after the test function
	SET_INPUT(TACH_1);
	WRITE(TACH_1, HIGH);
#endif
}


// G92 - Set current position to coordinates given
static void gcode_G92()
{
    bool codes[NUM_AXIS];
    float values[NUM_AXIS];

    // Check which axes need to be set
    for(uint8_t i = 0; i < NUM_AXIS; ++i)
    {
        codes[i] = code_seen(axis_codes[i]);
        if(codes[i])
            values[i] = code_value();
    }

    if((codes[E_AXIS] && values[E_AXIS] == 0) &&
       (!codes[X_AXIS] && !codes[Y_AXIS] && !codes[Z_AXIS]))
    {
        // As a special optimization, when _just_ clearing the E position
        // we schedule a flag asynchronously along with the next block to
        // reset the starting E position instead of stopping the planner
        current_position[E_AXIS] = 0;
        plan_reset_next_e();
    }
    else
    {
        // In any other case we're forced to synchronize
        st_synchronize();
        for(uint8_t i = 0; i < 3; ++i)
        {
            if(codes[i])
                current_position[i] = values[i] + cs.add_homing[i];
        }
        if(codes[E_AXIS])
            current_position[E_AXIS] = values[E_AXIS];

        // Set all at once
        plan_set_position_curposXYZE();
    }
}

#ifdef EXTENDED_CAPABILITIES_REPORT

static void cap_line(const char* name, bool ena = false) {
    printf_P(PSTR("Cap:%S:%c\n"), name, (char)ena + '0');
}

static void extended_capabilities_report()
{
    // AUTOREPORT_TEMP (M155)
    cap_line(PSTR("AUTOREPORT_TEMP"), ENABLED(AUTO_REPORT));
#if (defined(FANCHECK) && (((defined(TACH_0) && (TACH_0 >-1)) || (defined(TACH_1) && (TACH_1 > -1)))))
    // AUTOREPORT_FANS (M123)
    cap_line(PSTR("AUTOREPORT_FANS"), ENABLED(AUTO_REPORT));
#endif //FANCHECK and TACH_0 or TACH_1
    // AUTOREPORT_POSITION (M114)
    cap_line(PSTR("AUTOREPORT_POSITION"), ENABLED(AUTO_REPORT));
    // EXTENDED_M20 (support for L and T parameters)
    cap_line(PSTR("EXTENDED_M20"), 1);
    cap_line(PSTR("PRUSA_MMU2"), 1); //this will soon change to ENABLED(PRUSA_MMU2_SUPPORT)
}
#endif //EXTENDED_CAPABILITIES_REPORT

#ifdef BACKLASH_X
extern uint8_t st_backlash_x;
#endif //BACKLASH_X
#ifdef BACKLASH_Y
extern uint8_t st_backlash_y;
#endif //BACKLASH_Y

//! \ingroup marlin_main

//! @brief Parse and process commands
//!
//! look here for descriptions of G-codes: https://reprap.org/wiki/G-code
//!
//!
//! Implemented Codes 
//! -------------------
//!
//! * _This list is not updated. Current documentation is maintained inside the process_cmd function._ 
//!
//!@n PRUSA CODES
//!@n P F - Returns FW versions
//!@n P R - Returns revision of printer
//!
//!@n G0  -> G1
//!@n G1  - Coordinated Movement X Y Z E
//!@n G2  - CW ARC
//!@n G3  - CCW ARC
//!@n G4  - Dwell S<seconds> or P<milliseconds>
//!@n G10 - retract filament according to settings of M207
//!@n G11 - retract recover filament according to settings of M208
//!@n G28 - Home all Axes
//!@n G90 - Use Absolute Coordinates
//!@n G91 - Use Relative Coordinates
//!@n G92 - Set current position to coordinates given
//!
//!@n M Codes
//!@n M0   - Unconditional stop - Wait for user to press a button on the LCD
//!@n M1   - Same as M0
//!@n M17  - Enable/Power all stepper motors
//!@n M18  - Disable all stepper motors; same as M84
//!          syntax "M32 /path/filename#", or "M32 S<startpos bytes> !filename#"
//!          Call gcode file : "M32 P !filename#" and return to caller file after finishing (similar to #include).
//!          The '#' is necessary when calling from within sd files, as it stops buffer prereading
//!@n M42  - Change pin status via gcode Use M42 Px Sy to set pin x to value y, when omitting Px the onboard led will be used.
//!@n M73  - Show percent done and print time remaining
//!@n M80  - Turn on Power Supply
//!@n M81  - Turn off Power Supply
//!@n M82  - Set E codes absolute (default)
//!@n M83  - Set E codes relative while in Absolute Coordinates (G90) mode
//!@n M84  - Disable steppers until next move,
//!          or use S<seconds> to specify an inactivity timeout, after which the steppers will be disabled.  S0 to disable the timeout.
//!@n M85  - Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
//!@n M86  - Set safety timer expiration time with parameter S<seconds>; M86 S0 will disable safety timer
//!@n M92  - Set axis_steps_per_mm - same syntax as G92
//!@n M106 - Fan on
//!@n M107 - Fan off
//!        IF AUTOTEMP is enabled, S<mintemp> B<maxtemp> F<factor>. Exit autotemp by any M109 without F
//!@n M112 - Emergency stop
//!@n M113 - Get or set the timeout interval for Host Keepalive "busy" messages
//!@n M114 - Output current position to serial port
//!@n M115 - Capabilities string
//!@n M117 - display message
//!@n M118 - Serial print
//!@n M119 - Output Endstop status to serial port
//!@n M123 - Tachometer value
//!@n M126 - Solenoid Air Valve Open (BariCUDA support by jmil)
//!@n M127 - Solenoid Air Valve Closed (BariCUDA vent to atmospheric pressure by jmil)
//!@n M128 - EtoP Open (BariCUDA EtoP = electricity to air pressure transducer by jmil)
//!@n M129 - EtoP Closed (BariCUDA EtoP = electricity to air pressure transducer by jmil)
//!@n M150 - Set BlinkM Color Output R: Red<0-255> U(!): Green<0-255> B: Blue<0-255> over i2c, G for green does not work.
//!@n M155 - Automatically send temperatures, fan speeds, position
//!@n M190 - Sxxx Wait for bed current temp to reach target temp. Waits only when heating
//!          Rxxx Wait for bed current temp to reach target temp. Waits when heating and cooling
//!@n M200 D<millimeters>- set filament diameter and set E axis units to cubic millimeters (use S0 to set back to millimeters).
//!@n M201 - Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
//!@n M202 - Set max acceleration in units/s^2 for travel moves (M202 X1000 Y1000) Unused in Marlin!!
//!@n M203 - Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in mm/sec
//!@n M204 - Set default acceleration: S normal moves T filament only moves (M204 S3000 T7000) in mm/sec^2  also sets minimum segment time in ms (B20000) to prevent buffer under-runs and M20 minimum feedrate
//!@n M205 -  advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk, E=maximum E jerk
//!@n M206 - set additional homing offset
//!@n M207 - set retract length S[positive mm] F[feedrate mm/min] Z[additional zlift/hop], stays in mm regardless of M200 setting
//!@n M208 - set recover=unretract length S[positive mm surplus to the M207 S*] F[feedrate mm/sec]
//!@n M209 - S<1=true/0=false> enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
//!@n M214 - Set Arc Parameters (Use M500 to store in eeprom) P<MM_PER_ARC_SEGMENT> S<MIN_MM_PER_ARC_SEGMENT> R<MIN_ARC_SEGMENTS> F<ARC_SEGMENTS_PER_SEC>
//!@n M220 S<factor in percent>- set speed factor override percentage
//!@n M221 S<factor in percent>- set extrude factor override percentage
//!@n M226 P<pin number> S<pin state>- Wait until the specified pin reaches the state required
//!@n M240 - Trigger a camera to take a photograph
//!@n M250 - Set LCD contrast C<contrast value> (value 0..63)
//!@n M280 - set servo position absolute. P: servo index, S: angle or microseconds
//!@n M300 - Play beep sound S<frequency Hz> P<duration ms>
//!@n M301 - Set PID parameters P I and D
//!@n M302 - Allow cold extrudes, or set the minimum extrude S<temperature>.
//!@n M303 - PID relay autotune S<temperature> sets the target temperature. (default target temperature = 150C)
//!@n M304 - Set bed PID parameters P I and D
//!@n M310 - Thermal model settings
//!@n M400 - Finish all moves
//!@n M500 - stores parameters in EEPROM
//!@n M501 - reads parameters from EEPROM (if you need reset them after you changed them temporarily).
//!@n M502 - reverts to the default "factory settings".  You still need to store them in EEPROM afterwards if you want to.
//!@n M503 - print the current settings (from memory not from EEPROM)
//!@n M509 - force language selection on next restart
//!@n M540 - Use S[0|1] to enable or disable the stop SD card print on endstop hit (requires ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
//!@n M552 - Set IP address
//!@n M907 - Set digital trimpot motor current using axis codes.
//!@n M908 - Control digital trimpot directly.
//!@n M350 - Set microstepping mode.
//!@n M351 - Toggle MS1 MS2 pins directly.
//!
//!@n M928 - Start SD logging (M928 filename.g) - ended by M29
//!@n M999 - Restart after being stopped by error
//! <br><br>

/** @defgroup marlin_main Marlin main */

/** \ingroup GCodes */

//! _This is a list of currently implemented G Codes in Prusa firmware (dynamically generated from doxygen)._ 
/**
They are shown in order of appearance in the code.
There are reasons why some G Codes aren't in numerical order.
*/


void process_commands()
{
	if (!buflen) return; //empty command

#ifdef CMDBUFFER_DEBUG
  SERIAL_ECHOPGM("Processing a GCODE command: ");
  SERIAL_ECHO(cmdbuffer+bufindr+CMDHDRSIZE);
  SERIAL_ECHOLNPGM("");
  SERIAL_ECHOPGM("In cmdqueue: ");
  SERIAL_ECHO(buflen);
  SERIAL_ECHOLNPGM("");
#endif /* CMDBUFFER_DEBUG */
  
  unsigned long codenum; //throw away variable
#ifdef ENABLE_AUTO_BED_LEVELING
  float x_tmp, y_tmp, z_tmp, real_z;
#endif

  // PRUSA GCODES
  KEEPALIVE_STATE(IN_HANDLER);
    /*!
    ### Special internal commands
    These are used by internal functions to process certain actions in the right order. Some of these are also usable by the user.
    They are processed early as the commands are complex (strings).
    These are only available on the MK3(S) as these require TMC2130 drivers:
        - CRASH DETECTED
        - CRASH RECOVER
        - CRASH_CANCEL
        - TMC_SET_WAVE
        - TMC_SET_STEP
        - TMC_SET_CHOP
    */
	if (false) {} // allow chaining of optional next else if blocks
#ifdef TMC2130
	else if (strncmp_P(CMDBUFFER_CURRENT_STRING, PSTR("CRASH_"), 6) == 0)
	{

    // ### CRASH_DETECTED - TMC2130
    // ---------------------------------
	  if(code_seen_P(PSTR("CRASH_DETECTED")))
	  {
		  uint8_t mask = 0;
		  if (code_seen('X')) mask |= X_AXIS_MASK;
		  if (code_seen('Y')) mask |= Y_AXIS_MASK;
		  crashdet_detected(mask);
	  }

    // ### CRASH_RECOVER - TMC2130
    // ----------------------------------
	  else if(code_seen_P(PSTR("CRASH_RECOVER")))
		  crashdet_recover();

    // ### CRASH_CANCEL - TMC2130
    // ----------------------------------
	  else if(code_seen_P(PSTR("CRASH_CANCEL")))
		  crashdet_cancel();
	}
	else if (strncmp_P(CMDBUFFER_CURRENT_STRING, PSTR("TMC_"), 4) == 0)
	{
    
    // ### TMC_SET_WAVE_ 
    // --------------------
		if (strncmp_P(CMDBUFFER_CURRENT_STRING + 4, PSTR("SET_WAVE_"), 9) == 0)
		{
			uint8_t axis = *(CMDBUFFER_CURRENT_STRING + 13);
			axis = (axis == 'E')?3:(axis - 'X');
			if (axis < 4)
			{
				uint8_t fac = (uint8_t)strtol(CMDBUFFER_CURRENT_STRING + 14, NULL, 10);
				tmc2130_set_wave(axis, 247, fac);
			}
		}
    
    // ### TMC_SET_STEP_
    //  ------------------
		else if (strncmp_P(CMDBUFFER_CURRENT_STRING + 4, PSTR("SET_STEP_"), 9) == 0)
		{
			uint8_t axis = *(CMDBUFFER_CURRENT_STRING + 13);
			axis = (axis == 'E')?3:(axis - 'X');
			if (axis < 4)
			{
				uint8_t step = (uint8_t)strtol(CMDBUFFER_CURRENT_STRING + 14, NULL, 10);
				uint16_t res = tmc2130_get_res(axis);
				tmc2130_goto_step(axis, step & (4*res - 1), 2, 1000, res);
			}
		}

    // ### TMC_SET_CHOP_
    //  -------------------
		else if (strncmp_P(CMDBUFFER_CURRENT_STRING + 4, PSTR("SET_CHOP_"), 9) == 0)
		{
			uint8_t axis = *(CMDBUFFER_CURRENT_STRING + 13);
			axis = (axis == 'E')?3:(axis - 'X');
			if (axis < 4)
			{
				uint8_t chop0 = tmc2130_chopper_config[axis].toff;
				uint8_t chop1 = tmc2130_chopper_config[axis].hstr;
				uint8_t chop2 = tmc2130_chopper_config[axis].hend;
				uint8_t chop3 = tmc2130_chopper_config[axis].tbl;
				char* str_end = 0;
				if (CMDBUFFER_CURRENT_STRING[14])
				{
					chop0 = (uint8_t)strtol(CMDBUFFER_CURRENT_STRING + 14, &str_end, 10) & 15;
					if (str_end && *str_end)
					{
						chop1 = (uint8_t)strtol(str_end, &str_end, 10) & 7;
						if (str_end && *str_end)
						{
							chop2 = (uint8_t)strtol(str_end, &str_end, 10) & 15;
							if (str_end && *str_end)
								chop3 = (uint8_t)strtol(str_end, &str_end, 10) & 3;
						}
					}
				}
				tmc2130_chopper_config[axis].toff = chop0;
				tmc2130_chopper_config[axis].hstr = chop1 & 7;
				tmc2130_chopper_config[axis].hend = chop2 & 15;
				tmc2130_chopper_config[axis].tbl = chop3 & 3;
				tmc2130_setup_chopper(axis, tmc2130_mres[axis]);
				//printf_P(_N("TMC_SET_CHOP_%c %d %d %d %d\n"), "xyze"[axis], chop0, chop1, chop2, chop3);
			}
		}
	}
#ifdef BACKLASH_X
	else if (strncmp_P(CMDBUFFER_CURRENT_STRING, PSTR("BACKLASH_X"), 10) == 0)
	{
		uint8_t bl = (uint8_t)strtol(CMDBUFFER_CURRENT_STRING + 10, NULL, 10);
		st_backlash_x = bl;
		printf_P(_N("st_backlash_x = %d\n"), st_backlash_x);
	}
#endif //BACKLASH_X
#ifdef BACKLASH_Y
	else if (strncmp_P(CMDBUFFER_CURRENT_STRING, PSTR("BACKLASH_Y"), 10) == 0)
	{
		uint8_t bl = (uint8_t)strtol(CMDBUFFER_CURRENT_STRING + 10, NULL, 10);
		st_backlash_y = bl;
		printf_P(_N("st_backlash_y = %d\n"), st_backlash_y);
	}
#endif //BACKLASH_Y
#endif //TMC2130
  else if(strncmp_P(CMDBUFFER_CURRENT_STRING, PSTR("PRUSA"), 5) == 0) {
    /*!
    ---------------------------------------------------------------------------------
    ### PRUSA - Internal command set <a href="https://reprap.org/wiki/G-code#G98:_Activate_farm_mode">G98: Activate farm mode - Notes</a>
    
    Set of internal PRUSA commands
    #### Usage
         PRUSA [ PRN | FAN | thx | uvlo | MMURES | RESET | fv | M28 | SN | Fir | Rev | Lang | Lz | FR ]
    
    #### Parameters
      - `PRN` - Prints revision of the printer
      - `FAN` - Prints fan details
      - `thx` 
      - `uvlo` 
      - `MMURES` - Reset MMU
      - `RESET` - (Careful!)
      - `fv`  - ?
      - `M28` 
      - `SN` 
      - `Fir` - Prints firmware version
      - `Rev`- Prints filament size, elelectronics, nozzle type
      - `Lang` - Reset the language
      - `Lz` 
      - `FR` - Full factory reset
      - `nozzle set <diameter>` - set nozzle diameter (farm mode only), e.g. `PRUSA nozzle set 0.4`
      - `nozzle D<diameter>` - check the nozzle diameter (farm mode only), works like M862.1 P, e.g. `PRUSA nozzle D0.4`
      - `nozzle` - prints nozzle diameter (farm mode only), works like M862.1 P, e.g. `PRUSA nozzle`
    */

        if (farm_prusa_code_seen()) {}
        else if(code_seen_P(PSTR("FANPINTST"))) {
            gcode_PRUSA_BadRAMBoFanTest();
        } else if (code_seen_P(PSTR("FAN"))) { // PRUSA FAN
            printf_P(_N("E0:%d RPM\nPRN0:%d RPM\n"), 60*fan_speed[0], 60*fan_speed[1]);
        } else if (code_seen_P(PSTR("uvlo"))) { // PRUSA uvlo
            if (eeprom_read_byte((uint8_t*)EEPROM_UVLO_PRINT_TYPE) == PowerPanic::PRINT_TYPE_SD) {
                // M24 - Start SD print
                enquecommand_P(MSG_M24);

                // Print is recovered, clear the recovery flag
                eeprom_update_byte_notify((uint8_t*)EEPROM_UVLO, PowerPanic::NO_PENDING_RECOVERY);
                eeprom_update_byte_notify((uint8_t*)EEPROM_UVLO_Z_LIFTED, 0);
            } else if (eeprom_read_byte((uint8_t*)EEPROM_UVLO_PRINT_TYPE) == PowerPanic::PRINT_TYPE_HOST) {
                // For Host prints we need to start the timer so that the pause has any effect
                // this will allow g-codes to be processed while in the paused state
                // For SD prints, M24 starts the timer
                print_job_timer.start();
                usb_timer.start();

                // Park the extruder to the side and don't resume the print
                // we must assume that the host as not fully booted up at this point
                lcd_pause_print();
            }
        } else if (code_seen_P(PSTR("MMURES"))) { // PRUSA MMURES
            MMU2::mmu2.Reset(MMU2::MMU2::Software);
        } else if (code_seen_P(PSTR("RESET"))) { // PRUSA RESET
#if defined(XFLASH) && defined(BOOTAPP)
            boot_app_magic = 0;
#endif //defined(XFLASH) && defined(BOOTAPP)
            softReset();
        } else if (code_seen_P(PSTR("SN"))) { // PRUSA SN
            char SN[20];
            eeprom_read_block(SN, (uint8_t*)EEPROM_PRUSA_SN, 20);
            if (SN[19])
                puts_P(PSTR("SN invalid"));
            else
                puts(SN);
        } else if(code_seen_P(PSTR("Fir"))){ // PRUSA Fir

            SERIAL_PROTOCOLLNPGM(FW_VERSION_FULL);

    } else if(code_seen_P(PSTR("Rev"))){ // PRUSA Rev

      SERIAL_PROTOCOLLNPGM(FILAMENT_SIZE "-" ELECTRONICS "-" NOZZLE_TYPE );

    } else if(code_seen_P(PSTR("Lang"))) { // PRUSA Lang
        lang_reset();

    } else if(code_seen_P(PSTR("Lz"))) { // PRUSA Lz
      eeprom_update_word_notify(reinterpret_cast<uint16_t *>(&(EEPROM_Sheets_base->s[(eeprom_read_byte(&(EEPROM_Sheets_base->active_sheet)))].z_offset)),0);

    } else if(code_seen_P(PSTR("FR"))) { // PRUSA FR
        // Factory full reset
        factory_reset(0);
    } else if(code_seen_P(PSTR("MBL"))) { // PRUSA MBL
        // Change the MBL status without changing the logical Z position.
        if(code_seen('V')) {
            bool value = code_value_short();
            st_synchronize();
            if(value != mbl.active) {
                mbl.active = value;
                // Use plan_set_z_position to reset the physical values
                plan_set_z_position(current_position[Z_AXIS]);
            }
        }
    } else if (code_seen_P(PSTR("nozzle"))) { // PRUSA nozzle
            uint16_t nDiameter;
            if(code_seen('D')) {
                nDiameter=(uint16_t)(code_value()*1000.0+0.5); // [,um]
                nozzle_diameter_check(nDiameter);
            } else if(code_seen_P(PSTR("set")) && farm_mode) {
                strchr_pointer++;                  // skip 1st char (~ 's')
                strchr_pointer++;                  // skip 2nd char (~ 'e')
                nDiameter=(uint16_t)(code_value()*1000.0+0.5); // [,um]
                eeprom_update_byte_notify((uint8_t*)EEPROM_NOZZLE_DIAMETER,(uint8_t)ClNozzleDiameter::_Diameter_Undef); // for correct synchronization after farm-mode exiting
                eeprom_update_word_notify((uint16_t*)EEPROM_NOZZLE_DIAMETER_uM,nDiameter);
            } else SERIAL_PROTOCOLLN((float)eeprom_read_word((uint16_t*)EEPROM_NOZZLE_DIAMETER_uM)/1000.0);
    }
  }
  else if(*CMDBUFFER_CURRENT_STRING == 'G')
  {
	strchr_pointer = CMDBUFFER_CURRENT_STRING;
	gcode_in_progress = code_value_short();
//	printf_P(_N("BEGIN G-CODE=%u\n"), gcode_in_progress);
    switch (gcode_in_progress)
    {

    /*!
    ---------------------------------------------------------------------------------
	 # G Codes
	### G0, G1 - Coordinated movement X Y Z E <a href="https://reprap.org/wiki/G-code#G0_.26_G1:_Move">G0 & G1: Move</a> 
	In Prusa Firmware G0 and G1 are the same.
	#### Usage
	
	      G0 [ X | Y | Z | E | F | S ]
		  G1 [ X | Y | Z | E | F | S ]
	
	#### Parameters
	  - `X` - The position to move to on the X axis
	  - `Y` - The position to move to on the Y axis
	  - `Z` - The position to move to on the Z axis
	  - `E` - The amount to extrude between the starting point and ending point
	  - `F` - The feedrate per minute of the move between the starting point and ending point (if supplied)
	  
    */
    case 0: // G0 -> G1
    case 1: // G1
        {
        uint16_t start_segment_idx = restore_interrupted_gcode();
        get_coordinates(); // For X Y Z E F

        prepare_move(start_segment_idx);
        //ClearToSend();
      }
      break;

    /*!
	### G2, G3 - Controlled Arc Move <a href="https://reprap.org/wiki/G-code#G2_.26_G3:_Controlled_Arc_Move">G2 & G3: Controlled Arc Move</a>
	
    These commands don't propperly work with MBL enabled. The compensation only happens at the end of the move, so avoid long arcs.
    
	#### Usage
	
	      G2 [ X | Y | I | E | F ] (Clockwise Arc)
		  G3 [ X | Y | I | E | F ] (Counter-Clockwise Arc)
	
	#### Parameters
	  - `X` - The position to move to on the X axis
	  - `Y` - The position to move to on the Y axis
      - 'Z' - The position to move to on the Z axis
	  - `I` - The point in X space from the current X position to maintain a constant distance from
	  - `J` - The point in Y space from the current Y position to maintain a constant distance from
	  - `E` - The amount to extrude between the starting point and ending point
	  - `F` - The feedrate per minute of the move between the starting point and ending point (if supplied)
	
    */
    case 2:
    case 3:
    {
        uint16_t start_segment_idx = restore_interrupted_gcode();
#ifdef SF_ARC_FIX
        bool relative_mode_backup = relative_mode;
        relative_mode = true;
#endif
        get_coordinates(); // For X Y Z E F
#ifdef SF_ARC_FIX
        relative_mode=relative_mode_backup;
#endif

        offset[0] = code_seen('I') ? code_value() : 0.f;
        offset[1] = code_seen('J') ? code_value() : 0.f;
        
        prepare_arc_move((gcode_in_progress == 2), start_segment_idx);
    } break;

    /*!
	### G4 - Dwell <a href="https://reprap.org/wiki/G-code#G4:_Dwell">G4: Dwell</a>
	Pause the machine for a period of time.
	
	#### Usage
	
	    G4 [ P | S ]
	
	#### Parameters
	  - `P` - Time to wait, in milliseconds
	  - `S` - Time to wait, in seconds
	
    */
    case 4: 
      codenum = 0;
      if(code_seen('P')) codenum = code_value(); // milliseconds to wait
      if(code_seen('S')) codenum = code_value() * 1000; // seconds to wait
      if(codenum != 0)
      {
        if(custom_message_type != CustomMsg::M117)
        {
          LCD_MESSAGERPGM(_n("Sleep..."));////MSG_DWELL
        }
      }
      st_synchronize();
      codenum += _millis();  // keep track of when we started waiting
      previous_millis_cmd.start();
      while(_millis() < codenum) {
        manage_heater();
        manage_inactivity();
        lcd_update(0);
      }
      break;


    /*!
	### G21 - Sets Units to Millimters <a href="https://reprap.org/wiki/G-code#G21:_Set_Units_to_Millimeters">G21: Set Units to Millimeters</a>
	Units are in millimeters. Prusa doesn't support inches.
    */
    case 21: 
      break; //Doing nothing. This is just to prevent serial UNKOWN warnings.

        /*!
        ### G88 - Reserved <a href="https://reprap.org/wiki/G-code#G88:_Reserved">G88: Reserved</a>
        
        Currently has no effect. 
        */

        // Prusa3D specific: Don't know what it is for, it is in V2Calibration.gcode

		    case 88:
			      break;      

    /*!
	### G90 - Switch off relative mode <a href="https://reprap.org/wiki/G-code#G90:_Set_to_Absolute_Positioning">G90: Set to Absolute Positioning</a>
	All coordinates from now on are absolute relative to the origin of the machine. E axis is left intact.
    */
    case 90: {
		axis_relative_modes &= ~(X_AXIS_MASK | Y_AXIS_MASK | Z_AXIS_MASK);
    }
    break;

    /*!
	### G91 - Switch on relative mode <a href="https://reprap.org/wiki/G-code#G91:_Set_to_Relative_Positioning">G91: Set to Relative Positioning</a>
    All coordinates from now on are relative to the last position. E axis is left intact.
	*/
    case 91: {
		axis_relative_modes |= X_AXIS_MASK | Y_AXIS_MASK | Z_AXIS_MASK;
    }
    break;

    /*!
	### G92 - Set position <a href="https://reprap.org/wiki/G-code#G92:_Set_Position">G92: Set Position</a>
    
    It is used for setting the current position of each axis. The parameters are always absolute to the origin.
    If a parameter is omitted, that axis will not be affected.
    If `X`, `Y`, or `Z` axis are specified, the move afterwards might stutter because of Mesh Bed Leveling. `E` axis is not affected if the target position is 0 (`G92 E0`).
	A G92 without coordinates will reset all axes to zero on some firmware. This is not the case for Prusa-Firmware!
    
    #### Usage
	
	      G92 [ X | Y | Z | E ]
	
	#### Parameters
	  - `X` - new X axis position
	  - `Y` - new Y axis position
	  - `Z` - new Z axis position
	  - `E` - new extruder position
	
    */
    case 92: {
        gcode_G92();
    }
    
	default:
		printf_P(MSG_UNKNOWN_CODE, 'G', cmdbuffer + bufindr + CMDHDRSIZE);
    }
//	printf_P(_N("END G-CODE=%u\n"), gcode_in_progress);
	gcode_in_progress = 0;
  } // end if(code_seen('G'))
  /*!
  ### End of G-Codes
  */

  /*!
  ---------------------------------------------------------------------------------
  # M Commands
  
  */

  else  if(*CMDBUFFER_CURRENT_STRING == 'M')
  {
	  strchr_pointer = CMDBUFFER_CURRENT_STRING;

	  int index;
	  for (index = 1; *(strchr_pointer + index) == ' ' || *(strchr_pointer + index) == '\t'; index++);
	   
	 /*for (++strchr_pointer; *strchr_pointer == ' ' || *strchr_pointer == '\t'; ++strchr_pointer);*/
	  if (*(strchr_pointer+index) < '0' || *(strchr_pointer+index) > '9') {
		  printf_P(PSTR("Invalid M code: %s\n"), cmdbuffer + bufindr + CMDHDRSIZE);

	  } else
	  {
	  mcode_in_progress = code_value_short();
//	printf_P(_N("BEGIN M-CODE=%u\n"), mcode_in_progress);

    switch(mcode_in_progress)
    {

    /*!
    ### M0, M1 - Stop the printer <a href="https://reprap.org/wiki/G-code#M0:_Stop_or_Unconditional_stop">M0: Stop or Unconditional stop</a>
    #### Usage

      M0 [P<ms<] [S<sec>] [string]
      M1 [P<ms>] [S<sec>] [string]

    #### Parameters

    - `P<ms>`  - Expire time, in milliseconds
    - `S<sec>` - Expire time, in seconds
    - `string` - Must for M1 and optional for M0 message to display on the LCD
    */

    case 0:
    case 1: {
        const char *src = strchr_pointer + 2;
        codenum = 0;
        if (code_seen('P')) codenum = code_value_long(); // milliseconds to wait
        if (code_seen('S')) codenum = code_value_long() * 1000; // seconds to wait
        bool expiration_time_set = bool(codenum);

        while (*src == ' ') ++src;
        custom_message_type = CustomMsg::M0Wait;
        if (!expiration_time_set && *src != '\0') {
            lcd_setstatus(src);
        } else {
            // farmers want to abuse a bug from the previous firmware releases
            // - they need to see the filename on the status screen instead of "Wait for user..."
            // So we won't update the message in farm mode...
            if( ! farm_mode){
                LCD_MESSAGERPGM(_T(MSG_USERWAIT));
            } else {
                custom_message_type = CustomMsg::Status; // let the lcd display the name of the printed G-code file in farm mode
            }
        }
        st_synchronize();
        menu_set_block(MENU_BLOCK_STATUS_SCREEN_M0);
        previous_millis_cmd.start();
        if (expiration_time_set) {
            codenum += _millis();  // keep track of when we started waiting
            KEEPALIVE_STATE(PAUSED_FOR_USER);
            while(_millis() < codenum && !lcd_clicked()) {
                delay_keep_alive(0);
            }
            KEEPALIVE_STATE(IN_HANDLER);
        } else {
            marlin_wait_for_click();
        }
        menu_unset_block(MENU_BLOCK_STATUS_SCREEN_M0);
        if (IS_SD_PRINTING)
            custom_message_type = CustomMsg::Status;
        else
            LCD_MESSAGERPGM(MSG_WELCOME);
    }
    break;

    /*!
	### M17 - Enable all axes <a href="https://reprap.org/wiki/G-code#M17:_Enable.2FPower_all_stepper_motors">M17: Enable/Power all stepper motors</a>
    */

    case 17:
        LCD_MESSAGERPGM(_T(MSG_NO_MOVE));
        enable_x();
        enable_y();
        enable_z();
        enable_e0();
      break;

    /*!
	### M42 - Set pin state <a href="https://reprap.org/wiki/G-code#M42:_Switch_I.2FO_pin">M42: Switch I/O pin</a>
    #### Usage
    
        M42 [ P | S ]
        
    #### Parameters
    - `P` - Pin number.
    - `S` - Pin value. If the pin is analog, values are from 0 to 255. If the pin is digital, values are from 0 to 1.
    
    */
    case 42:
      if (code_seen('S'))
      {
        uint8_t pin_status = code_value_uint8();
        int8_t pin_number = LED_PIN;
        if (code_seen('P'))
          pin_number = code_value_uint8();
        for(int8_t i = 0; i < (int8_t)(sizeof(sensitive_pins)/sizeof(sensitive_pins[0])); i++)
        {
          if ((int8_t)pgm_read_byte(&sensitive_pins[i]) == pin_number)
          {
            pin_number = -1;
            break;
          }
        }
      #if defined(FAN_PIN) && FAN_PIN > -1
        if (pin_number == FAN_PIN)
          fanSpeed = pin_status;
      #endif
        if (pin_number > -1)
        {
          pinMode(pin_number, OUTPUT);
          digitalWrite(pin_number, pin_status);
          analogWrite(pin_number, pin_status);
        }
      }
     break;

    /*!
	### M46 - Show the assigned IP address <a href="https://reprap.org/wiki/G-code#M46:_Show_the_assigned_IP_address">M46: Show the assigned IP address.</a>
    */
    case 46:
    {
        // M46: Prusa3D: Show the assigned IP address.
        if (card.ToshibaFlashAir_isEnabled()) {
            uint8_t ip[4];
            if (card.ToshibaFlashAir_GetIP(ip)) {
                // SERIAL_PROTOCOLPGM("Toshiba FlashAir current IP: ");
                SERIAL_PROTOCOL(uint8_t(ip[0]));
                SERIAL_PROTOCOL('.');
                SERIAL_PROTOCOL(uint8_t(ip[1]));
                SERIAL_PROTOCOL('.');
                SERIAL_PROTOCOL(uint8_t(ip[2]));
                SERIAL_PROTOCOL('.');
                SERIAL_PROTOCOLLN(uint8_t(ip[3]));
            } else {
                SERIAL_PROTOCOLPGM("?Toshiba FlashAir GetIP failed\n");          
            }
        } else {
            SERIAL_PROTOCOLLNPGM("n/a");          
        }
        break;
    }

    /*!
	### M47 - Show end stops dialog on the display <a href="https://reprap.org/wiki/G-code#M47:_Show_end_stops_dialog_on_the_display">M47: Show end stops dialog on the display</a>
    */

    /*!
    ### M72 - Set/get Printer State <a href="https://reprap.org/wiki/G-code#M72:_Set.2FGet_Printer_State">M72: Set/get Printer State</a>
    Without any parameter get printer state
      - 0 = NotReady  Used by PrusaConnect
      - 1 = IsReady   Used by PrusaConnect
      - 2 = Idle
      - 3 = SD printing finished
      - 4 = Host printing finished
      - 5 = SD printing
      - 6 = Host printing

    #### Usage

        M72 [ S ]

    #### Parameters
        - `Snnn` - Set printer state 0 = not_ready, 1 = ready
    */
    case 72:
    {
        if(code_seen('S')){
            switch (code_value_uint8()){
            case 0:
                SetPrinterState(PrinterState::NotReady);
                break;
            case 1:
                SetPrinterState(PrinterState::IsReady);
                break;
            default:
                break;
            }
        } else {
            printf_P(_N("PrinterState: %d\n"),uint8_t(GetPrinterState()));
            break;
        }
        break;
    }

    /*!
    ### M78 - Show statistical information about the print jobs <a href="https://reprap.org/wiki/G-code#M78:_Show_statistical_information_about_the_print_jobs">M78: Show statistical information about the print jobs</a>
    */
    case 78:
    {
        // @todo useful for maintenance notifications
        SERIAL_ECHOPGM("STATS ");
        SERIAL_ECHO(eeprom_read_dword((uint32_t *)EEPROM_TOTALTIME));
        SERIAL_ECHOPGM(" min ");
        SERIAL_ECHO(eeprom_read_dword((uint32_t *)EEPROM_FILAMENTUSED));
        SERIAL_ECHOLNPGM(" cm.");
        break;
    }

    /*!
    ### M79 - Start host timer <a href="https://reprap.org/wiki/G-code#M79:_Start_host_timer">M79: Start host timer</a>
    Start the printer-host enable keep-alive timer. While the timer has not expired, the printer will enable host specific features.
    #### Usage

        M79 [ S ]
    #### Parameters
       - `S` - Quoted string containing two characters e.g. "PL"
    */
    case 79:
        M79_timer_restart();

        if (code_seen('S'))
        {
            unquoted_string str = unquoted_string(strchr_pointer);
            if (str.WasFound())
            {
                ResetHostStatusScreenName();
                SetHostStatusScreenName(str.GetUnquotedString());
            }
        }
#ifdef DEBUG_PRINTER_STATES
        debug_printer_states();
#endif //DEBUG_PRINTER_STATES

        if (eeprom_read_byte((uint8_t*)EEPROM_UVLO_PRINT_TYPE) == PowerPanic::PRINT_TYPE_HOST
           && printer_recovering()
           && printingIsPaused()) {
            // The print is in a paused state. The print was recovered following a power panic
            // but up to this point the printer has been waiting for the M79 from the host
            // Send action to the host, so the host can resume the print. It is up to the host
            // to resume the print correctly.
            if (uvlo_auto_recovery_ready) {
                SERIAL_ECHOLNRPGM(MSG_HOST_ACTION_UVLO_AUTO_RECOVERY_READY);
            } else {
                SERIAL_ECHOLNRPGM(MSG_HOST_ACTION_UVLO_RECOVERY_READY);
            }
        }

        break;

    /*!
	### M112 - Emergency stop <a href="https://reprap.org/wiki/G-code#M112:_Full_.28Emergency.29_Stop">M112: Full (Emergency) Stop</a>
    It is processed much earlier as to bypass the cmdqueue.
    */
    case 112: 
      kill(MSG_M112_KILL);
      break;

#if defined(AUTO_REPORT)
    /*!
	### M155 - Automatically send status <a href="https://reprap.org/wiki/G-code#M155:_Automatically_send_temperatures">M155: Automatically send temperatures</a>
	#### Usage
	
		M155 [ S ] [ C ]
	
	#### Parameters
	
	- `S` - Set autoreporting interval in seconds. 0 to disable. Maximum: 255
	- `C` - Activate auto-report function (bit mask). Default is temperature.

          bit 0 = Auto-report temperatures
          bit 1 = Auto-report fans
          bit 2 = Auto-report position
          bit 3 = free
          bit 4 = free
          bit 5 = free
          bit 6 = free
          bit 7 = free
     */
    case 155:
    {
        if (code_seen('S')){
            autoReportFeatures.SetPeriod( code_value_uint8() );
        }
        if (code_seen('C')){
            autoReportFeatures.SetMask(code_value_uint8());
        } else{
            autoReportFeatures.SetMask(1); //Backwards compability to host systems like Octoprint to send only temp if paramerter `C`isn't used.
        }
   }
    break;
#endif //AUTO_REPORT

    #if defined(FAN_PIN) && FAN_PIN > -1

      /*!
	  ### M106 - Set fan speed <a href="https://reprap.org/wiki/G-code#M106:_Fan_On">M106: Fan On</a>
      #### Usage
      
        M106 [ S ]
        
      #### Parameters
      - `S` - Specifies the duty cycle of the print fan. Allowed values are 0-255. If it's omitted, a value of 255 is used.
      */
      case 106: // M106 Sxxx Fan On S<speed> 0 .. 255
        if (code_seen('S')){
           fanSpeed = code_value_uint8();
        }
        else {
          fanSpeed = 255;
        }
        break;

      /*!
	  ### M107 - Fan off <a href="https://reprap.org/wiki/G-code#M107:_Fan_Off">M107: Fan Off</a>
      */
      case 107:
        fanSpeed = 0;
        break;
    #endif //FAN_PIN

    #if defined(PS_ON_PIN) && PS_ON_PIN > -1

      /*!
	  ### M80 - Turn on the Power Supply <a href="https://reprap.org/wiki/G-code#M80:_ATX_Power_On">M80: ATX Power On</a>
      Only works if the firmware is compiled with PS_ON_PIN defined.
      */
      case 80:
        SET_OUTPUT(PS_ON_PIN); //GND
        WRITE(PS_ON_PIN, PS_ON_AWAKE);

        // If you have a switch on suicide pin, this is useful
        // if you want to start another print with suicide feature after
        // a print without suicide...
        #if defined SUICIDE_PIN && SUICIDE_PIN > -1
            SET_OUTPUT(SUICIDE_PIN);
            WRITE(SUICIDE_PIN, HIGH);
        #endif

          powersupply = true;
          LCD_MESSAGERPGM(MSG_WELCOME);
          lcd_update(0);
        break;

      /*!
	  ### M81 - Turn off Power Supply <a href="https://reprap.org/wiki/G-code#M81:_ATX_Power_Off">M81: ATX Power Off</a>
      Only works if the firmware is compiled with PS_ON_PIN defined.
      */
      case 81: 
        disable_heater();
        st_synchronize();
        disable_e0();
        finishAndDisableSteppers();
        fanSpeed = 0;
        _delay(1000); // Wait a little before to switch off
      #if defined(SUICIDE_PIN) && SUICIDE_PIN > -1
        st_synchronize();
        suicide();
      #elif defined(PS_ON_PIN) && PS_ON_PIN > -1
        SET_OUTPUT(PS_ON_PIN);
        WRITE(PS_ON_PIN, PS_ON_ASLEEP);
      #endif
        powersupply = false;
        LCD_MESSAGERPGM(CAT4(CUSTOM_MENDEL_NAME,PSTR(" "),MSG_OFF,PSTR(".")));
        lcd_update(0);
	  break;
    #endif

    /*!
	### M82 - Set E axis to absolute mode <a href="https://reprap.org/wiki/G-code#M82:_Set_extruder_to_absolute_mode">M82: Set extruder to absolute mode</a>
	Makes the extruder interpret extrusion as absolute positions.
    */
    case 82:
      axis_relative_modes &= ~E_AXIS_MASK;
      break;

    /*!
	### M83 - Set E axis to relative mode <a href="https://reprap.org/wiki/G-code#M83:_Set_extruder_to_relative_mode">M83: Set extruder to relative mode</a>
	Makes the extruder interpret extrusion values as relative positions.
    */
    case 83:
      axis_relative_modes |= E_AXIS_MASK;
      break;

    /*!
	### M84 - Disable steppers <a href="https://reprap.org/wiki/G-code#M84:_Stop_idle_hold">M84: Stop idle hold</a>
    This command can be used to set the stepper inactivity timeout (`S`) or to disable steppers (`X`,`Y`,`Z`,`E`)
	This command can be used without any additional parameters. In that case all steppers are disabled.
    
    The file completeness check uses this parameter to detect an incomplete file. It has to be present at the end of a file with no parameters.
	
        M84 [ S | X | Y | Z | E ]
	
	  - `S` - Seconds
	  - `X` - X axis
	  - `Y` - Y axis
	  - `Z` - Z axis
	  - `E` - Extruder

	### M18 - Disable steppers <a href="https://reprap.org/wiki/G-code#M18:_Disable_all_stepper_motors">M18: Disable all stepper motors</a>
	Equal to M84 (compatibility)
    */
    case 18: //compatibility
    case 84: // M84
      if(code_seen('S')){
        stepper_inactive_time = code_value() * 1000;
      }
      else
      {
        bool all_axis = !((code_seen(axis_codes[X_AXIS])) || (code_seen(axis_codes[Y_AXIS])) || (code_seen(axis_codes[Z_AXIS]))|| (code_seen(axis_codes[E_AXIS])));
        if(all_axis)
        {
          st_synchronize();
          disable_e0();
          finishAndDisableSteppers();
        }
        else
        {
          st_synchronize();
      if (code_seen('X')) disable_x();
      if (code_seen('Y')) disable_y();
      if (code_seen('Z')) disable_z();
#if (E0_ENABLE_PIN != X_ENABLE_PIN) // Only enable on boards that have seperate ENABLE_PINS
      if (code_seen('E')) disable_e0();
#endif
        }
      }
      break;

    /*!
	### M85 - Set max inactive time <a href="https://reprap.org/wiki/G-code#M85:_Set_Inactivity_Shutdown_Timer">M85: Set Inactivity Shutdown Timer</a>
    #### Usage
    
        M85 [ S ]
    
    #### Parameters
    - `S` - specifies the time in seconds. If a value of 0 is specified, the timer is disabled.
    */
    case 85: // M85
      if(code_seen('S')) {
        max_inactive_time = code_value() * 1000;
      }
      break;
#ifdef SAFETYTIMER

    /*!
    ### M86 - Set safety timer expiration time <a href="https://reprap.org/wiki/G-code#M86:_Set_Safety_Timer_expiration_time">M86: Set Safety Timer expiration time</a>	
    When safety timer expires, heatbed and nozzle target temperatures are set to zero.
    #### Usage
    
        M86 [ S ]
    
    #### Parameters
    - `S` - specifies the time in seconds. If a value of 0 is specified, the timer is disabled.
    */
	case 86: 
	  if (code_seen('S')) {
	    safetytimer_inactive_time = code_value() * 1000;
		safetyTimer.start();
	  }
	  break;
#endif

    /*!
	### M92 Set Axis steps-per-unit <a href="https://reprap.org/wiki/G-code#M92:_Set_axis_steps_per_unit">M92: Set axis_steps_per_unit</a>
	Allows programming of steps per unit (usually mm) for motor drives. These values are reset to firmware defaults on power on, unless saved to EEPROM if available (M500 in Marlin)
	#### Usage
    
	    M92 [ X | Y | Z | E ]
	
    #### Parameters
	- `X` - Steps per mm for the X drive
	- `Y` - Steps per mm for the Y drive
	- `Z` - Steps per mm for the Z drive
	- `E` - Steps per mm for the extruder drive
    */
    case 92:
      for(int8_t i=0; i < NUM_AXIS; i++)
      {
        if(code_seen(axis_codes[i]))
        {
          float value = code_value();
          if(i == E_AXIS) { // E
            if(value < 20.0) {
              const float factor = cs.axis_steps_per_mm[E_AXIS] / value; // increase e constants if M92 E14 is given for netfab.
              cs.max_jerk[E_AXIS] *= factor;
              max_feedrate[E_AXIS] *= factor;
              max_acceleration_steps_per_s2[E_AXIS] *= factor;
            }
            cs.axis_steps_per_mm[E_AXIS] = value;
#if defined(FILAMENT_SENSOR) && (FILAMENT_SENSOR_TYPE == FSENSOR_PAT9125)
            fsensor.init();
#endif //defined(FILAMENT_SENSOR) && (FILAMENT_SENSOR_TYPE == FSENSOR_PAT9125)
          } else {
            cs.axis_steps_per_mm[i] = value;
          }
        }
      }
      reset_acceleration_rates();
      break;

    /*!
	### M110 - Set Line number <a href="https://reprap.org/wiki/G-code#M110:_Set_Current_Line_Number">M110: Set Current Line Number</a>
	Sets the line number in G-code
	#### Usage
    
	    M110 [ N ]
	
    #### Parameters
	- `N` - Line number
    */
    case 110:
      if (code_seen('N'))
	    gcode_LastN = code_value_long();
    break;

    /*!
    ### M113 - Get or set host keep-alive interval <a href="https://reprap.org/wiki/G-code#M113:_Host_Keepalive">M113: Host Keepalive</a>
    During some lengthy processes, such as G29, Marlin may appear to the host to have “gone away.” The “host keepalive” feature will send messages to the host when Marlin is busy or waiting for user response so the host won’t try to reconnect (or disconnect).
    #### Usage
    
        M113 [ S ]
	
    #### Parameters
	- `S` - Seconds. Default is 2 seconds between "busy" messages
    */
	case 113:
		if (code_seen('S')) {
			host_keepalive_interval = code_value_uint8();
		}
		else {
			SERIAL_ECHO_START;
			SERIAL_ECHOPAIR("M113 S", (unsigned long)host_keepalive_interval);
			SERIAL_PROTOCOLLN();
		}
		break;

    /*!
	### M115 - Firmware info <a href="https://reprap.org/wiki/G-code#M115:_Get_Firmware_Version_and_Capabilities">M115: Get Firmware Version and Capabilities</a>
    Print the firmware info and capabilities
    Without any arguments, prints Prusa firmware version number, machine type, extruder count and UUID.
    `M115 U` Checks the firmware version provided. If the firmware version provided by the U code is higher than the currently running firmware, it will pause the print for 30s and ask the user to upgrade the firmware.
	
	_Examples:_
	
	`M115` results:
	
	`FIRMWARE_NAME:Prusa-Firmware 3.8.1 based on Marlin FIRMWARE_URL:https://github.com/prusa3d/Prusa-Firmware PROTOCOL_VERSION:1.0 MACHINE_TYPE:Prusa i3 MK3S EXTRUDER_COUNT:1 UUID:00000000-0000-0000-0000-000000000000`
	
	`M115 V` results:
	
	`3.8.1`
	
	`M115 U3.8.2-RC1` results on LCD display for 30s or user interaction:
	
	`New firmware version available: 3.8.2-RC1 Please upgrade.`
    #### Usage
    
        M115 [ V | U ]
	
    #### Parameters
	- V - Report current installed firmware version
	- U - Firmware version provided by G-code to be compared to current one.  
	*/
	case 115: // M115
      if (code_seen('V')) {
          // Report the Prusa version number.
          SERIAL_PROTOCOLLNRPGM(FW_VERSION_STR_P());
      } else if (code_seen('U')) {
          // Check the firmware version provided. If the firmware version provided by the U code is higher than the currently running firmware,
          // pause the print for 30s and ask the user to upgrade the firmware.
          show_upgrade_dialog_if_version_newer(++ strchr_pointer);
      } else {
          char custom_mendel_name[MAX_CUSTOM_MENDEL_NAME_LENGTH];
          eeprom_read_block(custom_mendel_name,(char*)EEPROM_CUSTOM_MENDEL_NAME,MAX_CUSTOM_MENDEL_NAME_LENGTH);
          SERIAL_ECHOPGM("FIRMWARE_NAME:Prusa-Firmware ");
          SERIAL_ECHORPGM(FW_VERSION_STR_P());
          SERIAL_ECHOPGM("+");
          SERIAL_ECHOPGM(STR(FW_COMMITNR));
          SERIAL_ECHOPGM("_");
          SERIAL_ECHOPGM(FW_COMMIT_HASH);
          SERIAL_ECHOPGM(" based on Marlin FIRMWARE_URL:https://github.com/prusa3d/Prusa-Firmware PROTOCOL_VERSION:");
          SERIAL_ECHOPGM(PROTOCOL_VERSION);
          SERIAL_ECHOPGM(" MACHINE_TYPE:");
          SERIAL_PROTOCOL(custom_mendel_name);
          SERIAL_ECHOPGM(" EXTRUDER_COUNT:" STRINGIFY(EXTRUDERS));
#ifdef MACHINE_UUID
          SERIAL_ECHOPGM(" UUID:");
          SERIAL_ECHOPGM(MACHINE_UUID);
#endif //MACHINE_UUID
          SERIAL_ECHOLNPGM("");
#ifdef EXTENDED_CAPABILITIES_REPORT
          extended_capabilities_report();
#endif //EXTENDED_CAPABILITIES_REPORT
      }
      break;

    /*!
	### M114 - Get current position <a href="https://reprap.org/wiki/G-code#M114:_Get_Current_Position">M114: Get Current Position</a>
    */
    case 114:
		gcode_M114();
      break;

    /*!
    ### M117 - Display Message <a href="https://reprap.org/wiki/G-code#M117:_Display_Message">M117: Display Message</a>
    */
    case 117: {
        const char *src = strchr_pointer + 4; // "M117"
        lcd_setstatus(*src == ' '? src + 1: src);
        custom_message_type = CustomMsg::M117;
    }
    break;

    /*!
    ### M118 - Serial print <a href="https://reprap.org/wiki/G-code#M118:_Echo_message_on_host">M118: Serial print</a>
    #### Usage

        M118 [ A1 | E1 ] [ String ]

    #### Parameters
    - `A1` - Prepend // to denote a comment or action command. Hosts like OctoPrint can interpret such commands to perform special actions. See your host’s documentation.
    - `E1` - Prepend echo: to the message. Some hosts will display echo messages differently when preceded by echo:.
    - `String` - Message string. If omitted, a blank line will be sent.
    */
    case 118: {
        bool hasE = false, hasA = false;
        char *p = strchr_pointer + 5;
        
        for (uint8_t i = 2; i--;) {
          // A1, E1, and Pn are always parsed out
          if (!((p[0] == 'A' || p[0] == 'E') && p[1] == '1')) break;
          switch (p[0]) {
            case 'A': hasA = true; break;
            case 'E': hasE = true; break;
          }
          p += 2;
          while (*p == ' ') ++p;
        }

        if (hasE) SERIAL_ECHO_START;
        if (hasA) SERIAL_ECHOPGM("//");

        SERIAL_ECHOLN(p);
    }
    break;
    
    /*!
	### M119 - Get endstop states <a href="https://reprap.org/wiki/G-code#M119:_Get_Endstop_Status">M119: Get Endstop Status</a>
	Returns the current state of the configured X, Y, Z endstops. Takes into account any 'inverted endstop' settings, so one can confirm that the machine is interpreting the endstops correctly.
    */
    case 119:
    SERIAL_PROTOCOLRPGM(_N("Reporting endstop status"));////MSG_M119_REPORT
    SERIAL_PROTOCOLLN();
      #if defined(X_MIN_PIN) && X_MIN_PIN > -1
        SERIAL_PROTOCOLRPGM(_n("x_min: "));////MSG_X_MIN
        if(READ(X_MIN_PIN)^X_MIN_ENDSTOP_INVERTING){
          SERIAL_PROTOCOLRPGM(MSG_ENDSTOP_HIT);
        }else{
          SERIAL_PROTOCOLRPGM(MSG_ENDSTOP_OPEN);
        }
        SERIAL_PROTOCOLLN();
      #endif
      #if defined(X_MAX_PIN) && X_MAX_PIN > -1
        SERIAL_PROTOCOLRPGM(_n("x_max: "));////MSG_X_MAX
        if(READ(X_MAX_PIN)^X_MAX_ENDSTOP_INVERTING){
          SERIAL_PROTOCOLRPGM(MSG_ENDSTOP_HIT);
        }else{
          SERIAL_PROTOCOLRPGM(MSG_ENDSTOP_OPEN);
        }
        SERIAL_PROTOCOLLN();
      #endif
      #if defined(Y_MIN_PIN) && Y_MIN_PIN > -1
        SERIAL_PROTOCOLRPGM(_n("y_min: "));////MSG_Y_MIN
        if(READ(Y_MIN_PIN)^Y_MIN_ENDSTOP_INVERTING){
          SERIAL_PROTOCOLRPGM(MSG_ENDSTOP_HIT);
        }else{
          SERIAL_PROTOCOLRPGM(MSG_ENDSTOP_OPEN);
        }
        SERIAL_PROTOCOLLN();
      #endif
      #if defined(Y_MAX_PIN) && Y_MAX_PIN > -1
        SERIAL_PROTOCOLRPGM(_n("y_max: "));////MSG_Y_MAX
        if(READ(Y_MAX_PIN)^Y_MAX_ENDSTOP_INVERTING){
          SERIAL_PROTOCOLRPGM(MSG_ENDSTOP_HIT);
        }else{
          SERIAL_PROTOCOLRPGM(MSG_ENDSTOP_OPEN);
        }
        SERIAL_PROTOCOLLN();
      #endif
      #if defined(Z_MIN_PIN) && Z_MIN_PIN > -1
        SERIAL_PROTOCOLRPGM(MSG_Z_MIN);
        if(READ(Z_MIN_PIN)^Z_MIN_ENDSTOP_INVERTING){
          SERIAL_PROTOCOLRPGM(MSG_ENDSTOP_HIT);
        }else{
          SERIAL_PROTOCOLRPGM(MSG_ENDSTOP_OPEN);
        }
        SERIAL_PROTOCOLLN();
      #endif
      #if defined(Z_MAX_PIN) && Z_MAX_PIN > -1
        SERIAL_PROTOCOLRPGM(MSG_Z_MAX);
        if(READ(Z_MAX_PIN)^Z_MAX_ENDSTOP_INVERTING){
          SERIAL_PROTOCOLRPGM(MSG_ENDSTOP_HIT);
        }else{
          SERIAL_PROTOCOLRPGM(MSG_ENDSTOP_OPEN);
        }
        SERIAL_PROTOCOLLN();
      #endif
      break;
      //!@todo update for all axes, use for loop

#if (defined(FANCHECK) && (((defined(TACH_0) && (TACH_0 >-1)) || (defined(TACH_1) && (TACH_1 > -1)))))
    /*!
	### M123 - Tachometer value <a href="https://www.reprap.org/wiki/G-code#M123:_Tachometer_value_.28RepRap_.26_Prusa.29">M123: Tachometer value</a>
  This command is used to report fan speeds and fan pwm values.
  #### Usage
    
        M123

    - E0:     - Hotend fan speed in RPM
    - PRN1:   - Part cooling fans speed in RPM
    - E0@:    - Hotend fan PWM value
    - PRN1@:  -Part cooling fan PWM value

  _Example:_

    E0:3240 RPM PRN1:4560 RPM E0@:255 PRN1@:255

    */
    case 123:
    gcode_M123();
    break;
#endif //FANCHECK and TACH_0 and TACH_1

    /*!
	### M201 - Set Print Max Acceleration <a href="https://reprap.org/wiki/G-code#M201:_Set_max_acceleration">M201: Set max printing acceleration</a>
    For each axis individually.
    ##### Usage

    M201 [ X | Y | Z | E ]

    ##### Parameters
    - `X` - Acceleration for X axis in units/s^2
    - `Y` - Acceleration for Y axis in units/s^2
    - `Z` - Acceleration for Z axis in units/s^2
    - `E` - Acceleration for the active or specified extruder in units/s^2
    */
    case 201:
		for (int8_t i = 0; i < NUM_AXIS; i++)
		{
			if (code_seen(axis_codes[i]))
			{
				unsigned long val = code_value();
#ifdef TMC2130
				unsigned long val_silent = val;
				if ((i == X_AXIS) || (i == Y_AXIS))
				{
					if (val > NORMAL_MAX_ACCEL_XY)
						val = NORMAL_MAX_ACCEL_XY;
					if (val_silent > SILENT_MAX_ACCEL_XY)
						val_silent = SILENT_MAX_ACCEL_XY;
				}
				cs.max_acceleration_mm_per_s2_normal[i] = val;
				cs.max_acceleration_mm_per_s2_silent[i] = val_silent;
#else //TMC2130
				max_acceleration_mm_per_s2[i] = val;
#endif //TMC2130
			}
		}
		// steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
		reset_acceleration_rates();
		break;

    /*!
    ### M203 - Set Max Feedrate <a href="https://reprap.org/wiki/G-code#M203:_Set_maximum_feedrate">M203: Set maximum feedrate</a>
    For each axis individually.
    ##### Usage

    M203 [ X | Y | Z | E ]

    ##### Parameters
    - `X` - Maximum feedrate for X axis
    - `Y` - Maximum feedrate for Y axis
    - `Z` - Maximum feedrate for Z axis
    - `E` - Maximum feedrate for extruder drives
    */
    case 203: // M203 max feedrate mm/sec
		for (uint8_t i = 0; i < NUM_AXIS; i++)
		{
			if (code_seen(axis_codes[i]))
			{
				float val = code_value();
#ifdef TMC2130
				float val_silent = val;
				if ((i == X_AXIS) || (i == Y_AXIS))
				{
					if (val > NORMAL_MAX_FEEDRATE_XY)
						val = NORMAL_MAX_FEEDRATE_XY;
					if (val_silent > SILENT_MAX_FEEDRATE_XY)
						val_silent = SILENT_MAX_FEEDRATE_XY;
				}
				cs.max_feedrate_normal[i] = val;
				cs.max_feedrate_silent[i] = val_silent;
#else //TMC2130
				max_feedrate[i] = val;
#endif //TMC2130
			}
		}
		break;

    /*!
	### M204 - Acceleration settings <a href="https://reprap.org/wiki/G-code#M204:_Set_default_acceleration">M204: Set default acceleration</a>

    #### Old format:
    ##### Usage
    
        M204 [ S | T ]
        
    ##### Parameters
    - `S` - normal moves
    - `T` - filmanent only moves
    
    #### New format:
    ##### Usage
    
        M204 [ P | R | T ]
    
    ##### Parameters
    - `P` - printing moves
    - `R` - filmanent only moves
    - `T` - travel moves (as of now T is ignored)
	*/
    case 204:
      {
        if(code_seen('S')) {
          // Legacy acceleration format. This format is used by the legacy Marlin, MK2 or MK3 firmware,
          // and it is also generated by Slic3r to control acceleration per extrusion type
          // (there is a separate acceleration settings in Slicer for perimeter, first layer etc).
          cs.acceleration = cs.travel_acceleration = code_value();
          // Interpret the T value as retract acceleration in the old Marlin format.
          if(code_seen('T'))
            cs.retract_acceleration = code_value();
        } else {
          // New acceleration format, compatible with the upstream Marlin.
          if(code_seen('P'))
            cs.acceleration = code_value();
          if(code_seen('R'))
            cs.retract_acceleration = code_value();
          if(code_seen('T'))
            cs.travel_acceleration = code_value();
        }
      }
      break;

    /*!
	### M205 - Set advanced settings <a href="https://reprap.org/wiki/G-code#M205:_Advanced_settings">M205: Advanced settings</a>
    Set some advanced settings related to movement.
    #### Usage
    
        M205 [ S | T | B | X | Y | Z | E ]
        
    #### Parameters
    - `S` - Minimum feedrate for print moves (unit/s)
    - `T` - Minimum feedrate for travel moves (units/s)
    - `B` - Minimum segment time (us)
    - `X` - Maximum X jerk (units/s)
    - `Y` - Maximum Y jerk (units/s)
    - `Z` - Maximum Z jerk (units/s)
    - `E` - Maximum E jerk (units/s)
    */
    case 205: 
    {
      if(code_seen('S')) cs.minimumfeedrate = code_value();
      if(code_seen('T')) cs.mintravelfeedrate = code_value();
      if(code_seen('B')) cs.min_segment_time_us = (uint32_t)code_value();
      if(code_seen('X')) cs.max_jerk[X_AXIS] = cs.max_jerk[Y_AXIS] = code_value();
      if(code_seen('Y')) cs.max_jerk[Y_AXIS] = code_value();
      if(code_seen('Z')) cs.max_jerk[Z_AXIS] = code_value();
      if(code_seen('E'))
      {
          float e = code_value();
#ifndef LA_NOCOMPAT
          e = la10c_jerk(e);
#endif
          cs.max_jerk[E_AXIS] = e;
      }
    }
    break;

    /*!
	### M206 - Set additional homing offsets <a href="https://reprap.org/wiki/G-code#M206:_Offset_axes">M206: Offset axes</a>
    #### Usage
    
        M206 [ X | Y | Z ]
    
    #### Parameters
    - `X` - X axis offset
    - `Y` - Y axis offset
    - `Z` - Z axis offset
	*/
    case 206:
      for(uint8_t i=0; i < 3; i++)
      {
        if(code_seen(axis_codes[i])) cs.add_homing[i] = code_value();
      }
      break;

    /*!
    ### M214 - Set Arc configuration values (Use M500 to store in eeprom) <a href="https://reprap.org/wiki/G-code#M214:_Set_Arc_configuration_values">M214: Set Arc configuration values</a>

    #### Usage

        M214 [P] [S] [N] [R] [F]

    #### Parameters
    - `P` - A float representing the max and default millimeters per arc segment.  Must be greater than 0.
    - `S` - A float representing the minimum allowable millimeters per arc segment.  Set to 0 to disable
    - `N` - An int representing the number of arcs to draw before correcting the small angle approximation.  Set to 0 to disable.
    - `R` - An int representing the minimum number of segments per arcs of any radius,
            except when the results in segment lengths greater than or less than the minimum
            and maximum segment length.  Set to 0 to disable.
    - `F` - An int representing the number of segments per second, unless this results in segment lengths
            greater than or less than the minimum and maximum segment length.  Set to 0 to disable.
    */
    case 214: //!@n M214 - Set Arc Parameters (Use M500 to store in eeprom) P<MM_PER_ARC_SEGMENT> S<MIN_MM_PER_ARC_SEGMENT> R<MIN_ARC_SEGMENTS> F<ARC_SEGMENTS_PER_SEC>
    {
        // Extract all possible parameters if they appear
        float p = code_seen('P') ? code_value() : cs.mm_per_arc_segment;
        float s = code_seen('S') ? code_value() : cs.min_mm_per_arc_segment;
        unsigned char n = code_seen('N') ? code_value() : cs.n_arc_correction;
        unsigned short r = code_seen('R') ? code_value() : cs.min_arc_segments;
        unsigned short f = code_seen('F') ? code_value() : cs.arc_segments_per_sec;

        // Ensure mm_per_arc_segment is greater than 0, and that min_mm_per_arc_segment is sero or greater than or equal to mm_per_arc_segment
        if (p <=0 || s < 0 || p < s)
        {
            // Should we display some error here?
            break;
        }

        cs.mm_per_arc_segment = p;
        cs.min_mm_per_arc_segment = s;
        cs.n_arc_correction = n;
        cs.min_arc_segments = r;
        cs.arc_segments_per_sec = f;
    }break;

    /*!
	### M220 Set feedrate percentage <a href="https://reprap.org/wiki/G-code#M220:_Set_speed_factor_override_percentage">M220: Set speed factor override percentage</a>
	#### Usage
    
        M220 [ B | S | R ]
    
    #### Parameters
    - `B` - Backup current speed factor
	- `S` - Speed factor override percentage (0..100 or higher)
	- `R` - Restore previous speed factor
    */
    case 220: // M220 S<factor in percent>- set speed factor override percentage
    {
        bool codesWereSeen = false;
        if (code_seen('B')) //backup current speed factor
        {
            saved_feedmultiply_mm = feedmultiply;
            codesWereSeen = true;
        }
        if (code_seen('S'))
        {
            feedmultiply = code_value_short();
            codesWereSeen = true;
        }
        if (code_seen('R')) //restore previous feedmultiply
        {
            feedmultiply = saved_feedmultiply_mm;
            codesWereSeen = true;
        }
        if (!codesWereSeen)
        {
            printf_P(PSTR("%i%%\n"), feedmultiply);
        }
    }
    break;

    /*!
    ### M226 - Wait for Pin state <a href="https://reprap.org/wiki/G-code#M226:_Wait_for_pin_state">M226: Wait for pin state</a>
    Wait until the specified pin reaches the state required
    #### Usage
    
        M226 [ P | S ]
    
    #### Parameters
    - `P` - pin number
    - `S` - pin state
    */
	case 226: // M226 P<pin number> S<pin state>- Wait until the specified pin reaches the state required
	{
      if(code_seen('P')){
        int pin_number = code_value_short(); // pin number
        int pin_state = -1; // required pin state - default is inverted

        if(code_seen('S')) pin_state = code_value_short(); // required pin state

        if(pin_state >= -1 && pin_state <= 1){

          for(int8_t i = 0; i < (int8_t)(sizeof(sensitive_pins)/sizeof(sensitive_pins[0])); i++)
          {
            if (((int8_t)pgm_read_byte(&sensitive_pins[i]) == pin_number))
            {
              pin_number = -1;
              break;
            }
          }

          if (pin_number > -1)
          {
            int target = LOW;

            st_synchronize();

            pinMode(pin_number, INPUT);

            switch(pin_state){
            case 1:
              target = HIGH;
              break;

            case 0:
              target = LOW;
              break;

            case -1:
              target = !digitalRead(pin_number);
              break;
            }

            while(digitalRead(pin_number) != target){
              manage_heater();
              manage_inactivity();
              lcd_update(0);
            }
          }
        }
      }
    }
    break;

    #if NUM_SERVOS > 0

    /*!
	### M280 - Set/Get servo position <a href="https://reprap.org/wiki/G-code#M280:_Set_servo_position">M280: Set servo position</a>
	In Prusa Firmware this G-code is deactivated by default, must be turned on in the source code.
    #### Usage
    
        M280 [ P | S ]
    
    #### Parameters
    - `P` - Servo index (id)
    - `S` - Target position
    */
    case 280: // M280 - set servo position absolute. P: servo index, S: angle or microseconds
      {
        int servo_index = -1;
        int servo_position = 0;
        if (code_seen('P'))
          servo_index = code_value();
        if (code_seen('S')) {
          servo_position = code_value();
          if ((servo_index >= 0) && (servo_index < NUM_SERVOS)) {
#if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
		      servos[servo_index].attach(0);
#endif
            servos[servo_index].write(servo_position);
#if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
              _delay(PROBE_SERVO_DEACTIVATION_DELAY);
              servos[servo_index].detach();
#endif
          }
          else {
            SERIAL_ECHO_START;
            SERIAL_ECHO("Servo ");
            SERIAL_ECHO(servo_index);
            SERIAL_ECHOLN(" out of range");
          }
        }
        else if (servo_index >= 0) {
          SERIAL_PROTOCOL(MSG_OK);
          SERIAL_PROTOCOL(" Servo ");
          SERIAL_PROTOCOL(servo_index);
          SERIAL_PROTOCOL(": ");
          SERIAL_PROTOCOLLN(servos[servo_index].read());
        }
      }
      break;
    #endif // NUM_SERVOS > 0

    /*!
	### M300 - Play tone <a href="https://reprap.org/wiki/G-code#M300:_Play_beep_sound">M300: Play beep sound</a>
	In Prusa Firmware the defaults are `100Hz` and `1000ms`, so that `M300` without parameters will beep for a second.
    #### Usage
    
        M300 [ S | P ]
    
    #### Parameters
    - `S` - frequency in Hz. Not all firmware versions support this parameter
    - `P` - duration in milliseconds
    */
    case 300: // M300
    {
      uint16_t beepP = code_seen('P') ? code_value() : 1000;
      uint16_t beepS;
      if (!code_seen('S'))
          beepS = 0;
      else {
          beepS = code_value();
          if (!beepS) {
              // handle S0 as a pause
              _delay(beepP);
              break;
          }
      }
      Sound_MakeCustom(beepP, beepS, false);
    }
    break;
    
    /*!
	### M400 - Wait for all moves to finish <a href="https://reprap.org/wiki/G-code#M400:_Wait_for_current_moves_to_finish">M400: Wait for current moves to finish</a>
	Finishes all current moves and and thus clears the buffer.
    Equivalent to `G4` with no parameters.
    */
    case 400:
    {
      st_synchronize();
    }
    break;

    /*!
	### M403 - Set filament type (material) for particular extruder and notify the MMU <a href="https://reprap.org/wiki/G-code#M403:_Set_filament_type_.28material.29_for_particular_extruder_and_notify_the_MMU.">M403 - Set filament type (material) for particular extruder and notify the MMU</a>
    Currently three different materials are needed (default, flex and PVA).  
    And storing this information for different load/unload profiles etc. in the future firmware does not have to wait for "ok" from MMU.
    #### Usage
    
        M403 [ E | F ]
    
    #### Parameters
    - `E` - Extruder number. 0-indexed.
    - `F` - Filament type
	*/
    case 403:
	{
		// currently three different materials are needed (default, flex and PVA)
		// add storing this information for different load/unload profiles etc. in the future
		if (MMU2::mmu2.Enabled())
		{
			uint8_t extruder = 255;
			uint8_t filament = FILAMENT_UNDEFINED;
			if(code_seen('E')) extruder = code_value_uint8();
			if(code_seen('F')) filament = code_value_uint8();
			MMU2::mmu2.set_filament_type(extruder, filament);
		}
	}
	break;

    /*!
	### M500 - Store settings in EEPROM <a href="https://reprap.org/wiki/G-code#M500:_Store_parameters_in_non-volatile_storage">M500: Store parameters in non-volatile storage</a>
	Save current parameters to EEPROM.
    */
    case 500:
    {
        Config_StoreSettings();
    }
    break;

    /*!
	### M501 - Read settings from EEPROM <a href="https://reprap.org/wiki/G-code#M501:_Read_parameters_from_EEPROM">M501: Read parameters from EEPROM</a>
	Set the active parameters to those stored in the EEPROM. This is useful to revert parameters after experimenting with them.
    */
    case 501:
    {
        Config_RetrieveSettings();
    }
    break;

    /*!
	### M502 - Revert all settings to factory default <a href="https://reprap.org/wiki/G-code#M502:_Restore_Default_Settings">M502: Restore Default Settings</a>
	This command resets all tunable parameters to their default values, as set in the firmware's configuration files. This doesn't reset any parameters stored in the EEPROM, so it must be followed by M500 to write the default settings.
    */
    case 502:
    {
        Config_ResetDefault();
    }
    break;

    /*!
	### M503 - Repport all settings currently in memory <a href="https://reprap.org/wiki/G-code#M503:_Report_Current_Settings">M503: Report Current Settings</a>
	This command asks the firmware to reply with the current print settings as set in memory. Settings will differ from EEPROM contents if changed since the last load / save. The reply output includes the G-Code commands to produce each setting. For example, Steps-Per-Unit values are displayed as an M92 command.
    */
    case 503:
    {
        Config_PrintSettings();
    }
    break;

    /*!
	### M509 - Force language selection <a href="https://reprap.org/wiki/G-code#M509:_Force_language_selection">M509: Force language selection</a>
	Resets the language to English.
	Only on Original Prusa i3 MK2.5/s and MK3/s with multiple languages.
	*/
    case 509:
    {
		lang_reset();
        SERIAL_ECHO_START;
        SERIAL_PROTOCOLPGM("LANG SEL FORCED");
    }
    break;

	/*!
	### M552 - Set IP address <a href="https://reprap.org/wiki/G-code#M552:_Set_IP_address.2C_enable.2Fdisable_network_interface">M552: Set IP address, enable/disable network interface"</a>
    Sets the printer IP address that is shown in the support menu. Designed to be used with the help of host software.
    If P is not specified nothing happens.
    If the structure of the IP address is invalid, 0.0.0.0 is assumed and nothing is shown on the screen in the Support menu.
    #### Usage
    
        M552 [ P<IP_address> ]
    
    #### Parameters
    - `P` - The IP address in xxx.xxx.xxx.xxx format. Eg: P192.168.1.14
	*/
    case 552:
    {
        if (code_seen('P'))
        {
            uint8_t valCnt = 0;
            IP_address = 0;
            do
            {
                *strchr_pointer = '*';
                ((uint8_t*)&IP_address)[valCnt] = code_value_short();
                valCnt++;
            } while ((valCnt < 4) && code_seen('.'));
            
            if (valCnt != 4)
                IP_address = 0;
        }
    } break;
   
    /*!
	### M907 - Set digital trimpot motor current in mA using axis codes <a href="https://reprap.org/wiki/G-code#M907:_Set_digital_trimpot_motor">M907: Set digital trimpot motor</a>
	Set digital trimpot motor current using axis codes (X, Y, Z, E, B, S).
    M907 has no effect when the experimental Extruder motor current scaling mode is active (that applies to farm printing as well)
	#### Usage
    
        M907 [ X | Y | Z | E | B | S ]
	
    #### Parameters
    - `X` - X motor driver
    - `Y` - Y motor driver
    - `Z` - Z motor driver
    - `E` - Extruder motor driver
    - `B` - Second Extruder motor driver
    - `S` - All motors
    */
    case 907:
    {
#ifdef TMC2130
        // See tmc2130_cur2val() for translation to 0 .. 63 range
        for (uint_least8_t i = 0; i < NUM_AXIS; i++){
            if(code_seen(axis_codes[i])){
                if( i == E_AXIS && FarmOrUserECool() ){
                    SERIAL_ECHORPGM(eMotorCurrentScalingEnabled);
                    SERIAL_ECHOLNPGM(", M907 E ignored");
                    continue;
                }
                float cur_mA = code_value();
                uint8_t val = tmc2130_cur2val(cur_mA);
                currents[i].setiHold(val);
                currents[i].setiRun(val);
                tmc2130_setup_chopper(i, tmc2130_mres[i]);
            }
        }
#else //TMC2130
      #if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
        for(int i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) st_current_set(i,code_value());
        if(code_seen('B')) st_current_set(4,code_value());
        if(code_seen('S')) for(int i=0;i<=4;i++) st_current_set(i,code_value());
      #endif
      #ifdef MOTOR_CURRENT_PWM_XY_PIN
        if(code_seen('X')) st_current_set(0, code_value());
      #endif
      #ifdef MOTOR_CURRENT_PWM_Z_PIN
        if(code_seen('Z')) st_current_set(1, code_value());
      #endif
      #ifdef MOTOR_CURRENT_PWM_E_PIN
        if(code_seen('E')) st_current_set(2, code_value());
      #endif
#endif //TMC2130
    }
    break;

    /*!
	### M908 - Control digital trimpot directly <a href="https://reprap.org/wiki/G-code#M908:_Control_digital_trimpot_directly">M908: Control digital trimpot directly</a>
	In Prusa Firmware this G-code is deactivated by default, must be turned on in the source code. Not usable on Prusa printers.
    #### Usage
    
        M908 [ P | S ]
    
    #### Parameters
    - `P` - channel
    - `S` - current
    */
    case 908:
    {
      #if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
        uint8_t channel,current;
        if(code_seen('P')) channel=code_value();
        if(code_seen('S')) current=code_value();
        digitalPotWrite(channel, current);
      #endif
    }
    break;

#ifdef TMC2130
#ifdef TMC2130_SERVICE_CODES_M910_M918

    /*!
	### M910 - TMC2130 init <a href="https://reprap.org/wiki/G-code#M910:_TMC2130_init">M910: TMC2130 init</a>
	Not active in default, only if `TMC2130_SERVICE_CODES_M910_M918` is defined in source code.
	
    */
	case 910:
    {
		tmc2130_init(TMCInitParams(false, FarmOrUserECool()));
    }
    break;

    /*!
    ### M911 - Set TMC2130 holding currents <a href="https://reprap.org/wiki/G-code#M911:_Set_TMC2130_holding_currents">M911: Set TMC2130 holding currents</a>
	Not active in default, only if `TMC2130_SERVICE_CODES_M910_M918` is defined in source code.
    #### Usage
    
        M911 [ X | Y | Z | E ]
    
    #### Parameters
    - `X` - X stepper driver holding current value
    - `Y` - Y stepper driver holding current value
    - `Z` - Z stepper driver holding current value
    - `E` - Extruder stepper driver holding current value
    */
	case 911: 
    {
        for (uint8_t axis = 0; axis < NUM_AXIS; axis++) {
            if (code_seen(axis_codes[axis])) {
                currents[axis].setiHold(code_value_uint8());
                tmc2130_setup_chopper(axis, tmc2130_mres[axis]);
          }
        }
    }
    break;

    /*!
	### M912 - Set TMC2130 running currents <a href="https://reprap.org/wiki/G-code#M912:_Set_TMC2130_running_currents">M912: Set TMC2130 running currents</a>
	Not active in default, only if `TMC2130_SERVICE_CODES_M910_M918` is defined in source code.
    #### Usage
    
        M912 [ X | Y | Z | E ]
    
    #### Parameters
    - `X` - X stepper driver running current value
    - `Y` - Y stepper driver running current value
    - `Z` - Z stepper driver running current value
    - `E` - Extruder stepper driver running current value
    */
	case 912: 
    {
        for (uint8_t axis = 0; axis < NUM_AXIS; axis++) {
            if (code_seen(axis_codes[axis])) {
                currents[axis].setiRun(code_value_uint8());
                tmc2130_setup_chopper(axis, tmc2130_mres[axis]);
            }
        }
    }
    break;
#endif // TMC2130_SERVICE_CODES_M910_M918

    /*!
	### M913 - Print TMC2130 currents <a href="https://reprap.org/wiki/G-code#M913:_Print_TMC2130_currents">M913: Print TMC2130 currents</a>
	Not active in default, only if `TMC2130_SERVICE_CODES_M910_M918` is defined in source code.
	Shows TMC2130 currents.
    */
	case 913:
    {
		tmc2130_print_currents();
    }
    break;

   /*!
	### M914 - Set TMC2130 normal mode <a href="https://reprap.org/wiki/G-code#M914:_Set_TMC2130_normal_mode">M914: Set TMC2130 normal mode</a>
  Updates EEPROM only if "P" is given, otherwise temporary (lasts until reset or motor idle timeout)
      #### Usage

        M914 [ P | R | Q ]

    #### Parameters
    - `P` - Make the mode change permanent (write to EEPROM)
    - `R` - Revert to EEPROM value
    - `Q` - Print effective silent/normal status. (Does not report override)

    */

    /*!
	### M915 - Set TMC2130 silent mode <a href="https://reprap.org/wiki/G-code#M915:_Set_TMC2130_silent_mode">M915: Set TMC2130 silent mode</a>
    Updates EEPROM only if "P" is given, otherwise temporary (lasts until reset or motor idle timeout)
      #### Usage

        M915 [ P | R | Q]

    #### Parameters
    - `P` - Make the mode change permanent (write to EEPROM)
    - `R` - Revert to EEPROM value
    - `Q` - Print effective silent/normal status. (Does not report override)
    */
    case 914:
    case 915:
    {
    uint8_t newMode = (mcode_in_progress==914) ? TMC2130_MODE_NORMAL : TMC2130_MODE_SILENT;
    //printf_P(_n("tmc2130mode/smm/eep: %d %d %d %d"),tmc2130_mode,SilentModeMenu,eeprom_read_byte((uint8_t*)EEPROM_SILENT), bEnableForce_z);
    if (code_seen('R'))
    {
        newMode = eeprom_read_byte((uint8_t*)EEPROM_SILENT);
    }
    else if (code_seen('P'))
    {
        uint8_t newMenuMode = (mcode_in_progress==914) ? SILENT_MODE_NORMAL : SILENT_MODE_STEALTH;
        eeprom_update_byte_notify((unsigned char *)EEPROM_SILENT, newMenuMode);
        SilentModeMenu = newMenuMode;
        //printf_P(_n("tmc2130mode/smm/eep: %d %d %d %d"),tmc2130_mode,SilentModeMenu,eeprom_read_byte((uint8_t*)EEPROM_SILENT), bEnableForce_z);
    }
    else if (code_seen('Q'))
    {
        printf_P(PSTR("%S: %S\n"), _O(MSG_MODE),
            tmc2130_mode == TMC2130_MODE_NORMAL ?
            _O(MSG_NORMAL) : _O(MSG_SILENT)
        );

    }
      if (tmc2130_mode != newMode
#ifdef PSU_Delta 
          || !bEnableForce_z 
#endif
        )
      {
#ifdef PSU_Delta
        enable_force_z();
#endif
        change_power_mode_live(newMode);
      }
    }
    break;

#ifdef TMC2130_SERVICE_CODES_M910_M918
    /*!
    ### M916 - Set TMC2130 Stallguard sensitivity threshold <a href="https://reprap.org/wiki/G-code#M916:_Set_TMC2130_Stallguard_sensitivity_threshold">M916: Set TMC2130 Stallguard sensitivity threshold</a>
    Not active in default, only if `TMC2130_SERVICE_CODES_M910_M918` is defined in source code.
    #### Usage
    
        M916 [ X | Y | Z | E ]
    
    #### Parameters
    - `X` - X stepper driver stallguard sensitivity threshold value
    - `Y` - Y stepper driver stallguard sensitivity threshold value
    - `Z` - Z stepper driver stallguard sensitivity threshold value
    - `E` - Extruder stepper driver stallguard sensitivity threshold value
    */
    case 916:
    {
        for (uint8_t axis = 0; axis < NUM_AXIS; axis++) {
            if (code_seen(axis_codes[axis])) {
                    tmc2130_sg_thr[axis] = code_value_uint8();
            }
            printf_P(_N("tmc2130_sg_thr[%c]=%d\n"), "XYZE"[axis], tmc2130_sg_thr[axis]);
        }
    }
    break;

    /*!
    ### M917 - Set TMC2130 PWM amplitude offset (pwm_ampl) <a href="https://reprap.org/wiki/G-code#M917:_Set_TMC2130_PWM_amplitude_offset_.28pwm_ampl.29">M917: Set TMC2130 PWM amplitude offset (pwm_ampl)</a>
    Not active in default, only if `TMC2130_SERVICE_CODES_M910_M918` is defined in source code.
    #### Usage
    
        M917 [ X | Y | Z | E ]
    
    #### Parameters
    - `X` - X stepper driver PWM amplitude offset value
    - `Y` - Y stepper driver PWM amplitude offset value
    - `Z` - Z stepper driver PWM amplitude offset value
    - `E` - Extruder stepper driver PWM amplitude offset value
    */
    case 917:
    {
        for (uint8_t axis = 0; axis < NUM_AXIS; axis++) {
            if (code_seen(axis_codes[axis])) {
                    tmc2130_set_pwm_ampl(axis, code_value_uint8());
            }
        }
    }
    break;

    /*!
    ### M918 - Set TMC2130 PWM amplitude gradient (pwm_grad) <a href="https://reprap.org/wiki/G-code#M918:_Set_TMC2130_PWM_amplitude_gradient_.28pwm_grad.29">M918: Set TMC2130 PWM amplitude gradient (pwm_grad)</a>
    Not active in default, only if `TMC2130_SERVICE_CODES_M910_M918` is defined in source code.
    #### Usage
    
        M918 [ X | Y | Z | E ]
    
    #### Parameters
    - `X` - X stepper driver PWM amplitude gradient value
    - `Y` - Y stepper driver PWM amplitude gradient value
    - `Z` - Z stepper driver PWM amplitude gradient value
    - `E` - Extruder stepper driver PWM amplitude gradient value
    */
    case 918:
    {
        for (uint8_t axis = 0; axis < NUM_AXIS; axis++) {
            if (code_seen(axis_codes[axis])) {
                    tmc2130_set_pwm_grad(axis, code_value_uint8());
            }
        }
    }
    break;

#endif //TMC2130_SERVICE_CODES_M910_M918
#endif // TMC2130

    /*!
	### M350 - Set microstepping mode <a href="https://reprap.org/wiki/G-code#M350:_Set_microstepping_mode">M350: Set microstepping mode</a>
    Printers with TMC2130 drivers have `X`, `Y`, `Z` and `E` as options. The steps-per-unit value is updated accordingly. Not all resolutions are valid!
    Printers without TMC2130 drivers also have `B` and `S` options. In this case, the steps-per-unit value in not changed!
    #### Usage
    
        M350 [ X | Y | Z | E | B | S ]
    
    #### Parameters
    - `X` - X new resolution
    - `Y` - Y new resolution
    - `Z` - Z new resolution
    - `E` - E new resolution
    
    Only valid for MK2.5(S) or printers without TMC2130 drivers
    - `B` - Second extruder new resolution
    - `S` - All axes new resolution
    */
    case 350: 
    {
	#ifdef TMC2130
		for (uint_least8_t i=0; i<NUM_AXIS; i++) 
		{
			if(code_seen(axis_codes[i]))
			{
				uint16_t res_new = code_value();
#ifdef ALLOW_ALL_MRES
				bool res_valid = res_new > 0 && res_new <= 256 && !(res_new & (res_new - 1)); // must be a power of two
#else
				bool res_valid = (res_new == 8) || (res_new == 16) || (res_new == 32); // resolutions valid for all axis
				res_valid |= (i != E_AXIS) && ((res_new == 1) || (res_new == 2) || (res_new == 4)); // resolutions valid for X Y Z only
				res_valid |= (i == E_AXIS) && ((res_new == 64) || (res_new == 128)); // resolutions valid for E only
#endif
				if (res_valid)
				{
					st_synchronize();
					uint16_t res = tmc2130_get_res(i);
					tmc2130_set_res(i, res_new);
					cs.axis_ustep_resolution[i] = res_new;
					if (res_new > res)
					{
						uint16_t fac = (res_new / res);
						cs.axis_steps_per_mm[i] *= fac;
						position[i] *= fac;
					}
					else
					{
						uint16_t fac = (res / res_new);
						cs.axis_steps_per_mm[i] /= fac;
						position[i] /= fac;
					}
#if defined(FILAMENT_SENSOR) && (FILAMENT_SENSOR_TYPE == FSENSOR_PAT9125)
					if (i == E_AXIS)
						fsensor.init();
#endif //defined(FILAMENT_SENSOR) && (FILAMENT_SENSOR_TYPE == FSENSOR_PAT9125)
				}
			}
		}
		reset_acceleration_rates();
	#else //TMC2130
      #if defined(X_MS1_PIN) && X_MS1_PIN > -1
        if(code_seen('S')) for(int i=0;i<=4;i++) microstep_mode(i,code_value());
        for(int i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) microstep_mode(i,(uint8_t)code_value());
        if(code_seen('B')) microstep_mode(4,code_value());
        microstep_readings();
      #endif
	#endif //TMC2130
    }
    break;

	/*!
	#### End of M-Commands
    */
	default: 
		printf_P(MSG_UNKNOWN_CODE, 'M', cmdbuffer + bufindr + CMDHDRSIZE);
    }
//	printf_P(_N("END M-CODE=%u\n"), mcode_in_progress);
	mcode_in_progress = 0;
	}
  }
  // end if(code_seen('M')) (end of M codes)

  /**
  *---------------------------------------------------------------------------------
  *# D codes
  */
  else if(*CMDBUFFER_CURRENT_STRING == 'D') // D codes (debug)
  {
    strchr_pointer = CMDBUFFER_CURRENT_STRING;
    switch(code_value_short())
    {

    /*!
    ### D-1 - Endless Loop <a href="https://reprap.org/wiki/G-code#D-1:_Endless_Loop">D-1: Endless Loop</a>
    */
	case -1:
		dcode__1(); break;
#ifdef DEBUG_DCODES

    /*!
    ### D0 - Reset <a href="https://reprap.org/wiki/G-code#D0:_Reset">D0: Reset</a>
    #### Usage
    
        D0 [ B ]
    
    #### Parameters
    - `B` - Bootloader
    */
	case 0:
		dcode_0(); break;

    /*!
    *
    ### D1 - Clear EEPROM and RESET <a href="https://reprap.org/wiki/G-code#D1:_Clear_EEPROM_and_RESET">D1: Clear EEPROM and RESET</a>
      
          D1
      
    *
    */
	case 1:
		dcode_1(); break;
#endif

#if defined DEBUG_DCODE2 || defined DEBUG_DCODES
    /*!
    ### D2 - Read/Write RAM <a href="https://reprap.org/wiki/G-code#D2:_Read.2FWrite_RAM">D3: Read/Write RAM</a>
    This command can be used without any additional parameters. It will read the entire RAM.
    #### Usage
    
        D2 [ A | C | X ]
    
    #### Parameters
    - `A` - Address (x0000-x1fff)
    - `C` - Count (1-8192)
    - `X` - Data

	#### Notes
	- The hex address needs to be lowercase without the 0 before the x
	- Count is decimal 
	- The hex data needs to be lowercase
	
    */
	case 2:
		dcode_2(); break;
#endif //DEBUG_DCODES
#if defined DEBUG_DCODE3 || defined DEBUG_DCODES

    /*!
    ### D3 - Read/Write EEPROM <a href="https://reprap.org/wiki/G-code#D3:_Read.2FWrite_EEPROM">D3: Read/Write EEPROM</a>
    This command can be used without any additional parameters. It will read the entire eeprom.
    #### Usage
    
        D3 [ A | C | X ]
    
    #### Parameters
    - `A` - Address (x0000-x0fff)
    - `C` - Count (1-4096)
    - `X` - Data (hex)
	
	#### Notes
	- The hex address needs to be lowercase without the 0 before the x
	- Count is decimal 
	- The hex data needs to be lowercase
	
    */
	case 3:
		dcode_3(); break;
#endif //DEBUG_DCODE3
#ifdef DEBUG_DCODES

    /*!
    
    ### D4 - Read/Write PIN <a href="https://reprap.org/wiki/G-code#D4:_Read.2FWrite_PIN">D4: Read/Write PIN</a>
    To read the digital value of a pin you need only to define the pin number.
    #### Usage
    
        D4 [ P | F | V ]
    
    #### Parameters
    - `P` - Pin (0-255)
    - `F` - Function in/out (0/1)
    - `V` - Value (0/1)
    */
	case 4:
		dcode_4(); break;
#endif //DEBUG_DCODES
#if defined DEBUG_DCODE5 || defined DEBUG_DCODES

    /*!
    ### D5 - Read/Write FLASH <a href="https://reprap.org/wiki/G-code#D5:_Read.2FWrite_FLASH">D5: Read/Write Flash</a>
    This command can be used without any additional parameters. It will read the 1kb FLASH.
    #### Usage
    
        D5 [ A | C | X | E ]
    
    #### Parameters
    - `A` - Address (x00000-x3ffff)
    - `C` - Count (1-8192)
    - `X` - Data (hex)
    - `E` - Erase
 	
	#### Notes
	- The hex address needs to be lowercase without the 0 before the x
	- Count is decimal 
	- The hex data needs to be lowercase
	
   */
	case 5:
		dcode_5(); break;
#endif //DEBUG_DCODE5
#if defined DEBUG_DCODE6 || defined DEBUG_DCODES

    /*!
    ### D6 - Read/Write external FLASH <a href="https://reprap.org/wiki/G-code#D6:_Read.2FWrite_external_FLASH">D6: Read/Write external Flash</a>
    Reserved
    */
	case 6:
		dcode_6(); break;
#endif
#ifdef DEBUG_DCODES

    /*!
    ### D7 - Read/Write Bootloader <a href="https://reprap.org/wiki/G-code#D7:_Read.2FWrite_Bootloader">D7: Read/Write Bootloader</a>
    Reserved
    */
	case 7:
		dcode_7(); break;

    /*!
    ### D8 - Read/Write PINDA <a href="https://reprap.org/wiki/G-code#D8:_Read.2FWrite_PINDA">D8: Read/Write PINDA</a>
    #### Usage
    
        D8 [ ? | ! | P | Z ]
    
    #### Parameters
    - `?` - Read PINDA temperature shift values
    - `!` - Reset PINDA temperature shift values to default
    - `P` - Pinda temperature [C]
    - `Z` - Z Offset [mm]
    */
	case 8:
		dcode_8(); break;

    /*!
    ### D9 - Read ADC <a href="https://reprap.org/wiki/G-code#D9:_Read.2FWrite_ADC">D9: Read ADC</a>
    #### Usage
    
        D9 [ I | V ]
    
    #### Parameters
    - `I` - ADC channel index 
        - `0` - Heater 0 temperature
        - `1` - Heater 1 temperature
        - `2` - Bed temperature
        - `3` - PINDA temperature
        - `4` - PWR voltage
        - `5` - Ambient temperature
        - `6` - BED voltage
    - `V` Value to be written as simulated
    */
	case 9:
		dcode_9(); break;

    /*!
    ### D10 - Set XYZ calibration = OK <a href="https://reprap.org/wiki/G-code#D10:_Set_XYZ_calibration_.3D_OK">D10: Set XYZ calibration = OK</a>
    */
	case 10:
		dcode_10(); break;

    /*!
    ### D12 - Time <a href="https://reprap.org/wiki/G-code#D12:_Time">D12: Time</a>
    Writes the current time in the log file.
    */
#endif //DEBUG_DCODES

#ifdef XFLASH_DUMP
    /*!
    ### D20 - Generate an offline crash dump <a href="https://reprap.org/wiki/G-code#D20:_Generate_an_offline_crash_dump">D20: Generate an offline crash dump</a>
    Generate a crash dump for later retrival.
    #### Usage

     D20 [E]

    ### Parameters
    - `E` - Perform an emergency crash dump (resets the printer).
    ### Notes
    - A crash dump can be later recovered with D21, or cleared with D22.
    - An emergency crash dump includes register data, but will cause the printer to reset after the dump
      is completed.
    */
    case 20: {
        dcode_20();
        break;
    };

    /*!
    ### D21 - Print crash dump to serial <a href="https://reprap.org/wiki/G-code#D21:_Print_crash_dump_to_serial">D21: Print crash dump to serial</a>
    Output the complete crash dump (if present) to the serial.
    #### Usage

     D21

    ### Notes
    - The starting address can vary between builds, but it's always at the beginning of the data section.
    */
    case 21: {
        dcode_21();
        break;
    };

    /*!
    ### D22 - Clear crash dump state <a href="https://reprap.org/wiki/G-code#D22:_Clear_crash_dump_state">D22: Clear crash dump state</a>
    Clear an existing internal crash dump.
    #### Usage

     D22
    */
    case 22: {
        dcode_22();
        break;
    };
#endif //XFLASH_DUMP

#ifdef EMERGENCY_SERIAL_DUMP
    /*!
    ### D23 - Request emergency dump on serial <a href="https://reprap.org/wiki/G-code#D23:_Request_emergency_dump_on_serial">D23: Request emergency dump on serial</a>
    On boards without offline dump support, request online dumps to the serial port on firmware faults.
    When online dumps are enabled, the FW will dump memory on the serial before resetting.
    #### Usage

     D23 [E] [R]
    #### Parameters
    - `E` - Perform an emergency crash dump (resets the printer).
    - `R` - Disable online dumps.
    */
    case 23: {
        dcode_23();
        break;
    };
#endif

#ifdef THERMAL_MODEL_DEBUG
    /*!
    ## D70 - Enable low-level thermal model logging for offline simulation
    #### Usage

        D70 [ S ]

    #### Parameters
    - `S` - Enable 0-1 (default 0)
    */
    case 70: {
        if(code_seen('S'))
            thermal_model_log_enable(code_value_short());
        break;
    }
#endif

#ifdef HEATBED_ANALYSIS

    /*!
    ### D80 - Bed check <a href="https://reprap.org/wiki/G-code#D80:_Bed_check">D80: Bed check</a>
    This command will log data to SD card file "mesh.txt".
    #### Usage
    
        D80 [ E | F | G | H | I | J ]
    
    #### Parameters
    - `E` - Dimension X (default 40)
    - `F` - Dimention Y (default 40)
    - `G` - Points X (default 40)
    - `H` - Points Y (default 40)
    - `I` - Offset X (default 74)
    - `J` - Offset Y (default 34)
  */
	case 80:
		dcode_80(); break;

    /*!
    ### D81 - Bed analysis <a href="https://reprap.org/wiki/G-code#D81:_Bed_analysis">D80: Bed analysis</a>
    This command will log data to SD card file "wldsd.txt".
    #### Usage
    
        D81 [ E | F | G | H | I | J ]
    
    #### Parameters
    - `E` - Dimension X (default 40)
    - `F` - Dimention Y (default 40)
    - `G` - Points X (default 40)
    - `H` - Points Y (default 40)
    - `I` - Offset X (default 74)
    - `J` - Offset Y (default 34)
  */
	case 81:
		dcode_81(); break;
	
#endif //HEATBED_ANALYSIS
#ifdef DEBUG_DCODES

    /*!
    ### D106 - Print measured fan speed for different pwm values <a href="https://reprap.org/wiki/G-code#D106:_Print_measured_fan_speed_for_different_pwm_values">D106: Print measured fan speed for different pwm values</a>
    */
	case 106:
		dcode_106(); break;

#ifdef TMC2130
    /*!
    ### D2130 - Trinamic stepper controller <a href="https://reprap.org/wiki/G-code#D2130:_Trinamic_stepper_controller">D2130: Trinamic stepper controller</a>
    @todo Please review by owner of the code. RepRap Wiki Gcode needs to be updated after review of owner as well.
    
    #### Usage
    
        D2130 [ Axis | Command | Subcommand | Value ]
    
    #### Parameters
    - Axis
      - `X` - X stepper driver
      - `Y` - Y stepper driver
      - `Z` - Z stepper driver
      - `E` - Extruder stepper driver
    - Commands
      - `0`   - Current off
      - `1`   - Current on
      - `+`   - Single step
      - `-`   - Single step oposite direction
      - `NNN` - Value sereval steps
      - `?`   - Read register
      - Subcommands for read register
        - `mres`     - Micro step resolution. More information in datasheet '5.5.2 CHOPCONF – Chopper Configuration'
        - `step`     - Step
        - `mscnt`    - Microstep counter. More information in datasheet '5.5 Motor Driver Registers'
        - `mscuract` - Actual microstep current for motor. More information in datasheet '5.5 Motor Driver Registers'
        - `wave`     - Microstep linearity compensation curve
      - `!`   - Set register
      - Subcommands for set register
        - `mres`     - Micro step resolution
        - `step`     - Step
        - `wave`     - Microstep linearity compensation curve
        - Values for set register
          - `0, 180 --> 250` - Off
          - `0.9 --> 1.25`   - Valid values (recommended is 1.1)
      - `@`   - Home calibrate axis
    
    Examples:
      
          D2130E?wave
      
      Print extruder microstep linearity compensation curve
      
          D2130E!wave0
      
      Disable extruder linearity compensation curve, (sine curve is used)
      
          D2130E!wave220
      
      (sin(x))^1.1 extruder microstep compensation curve used
    
    Notes:
      For more information see https://www.trinamic.com/fileadmin/assets/Products/ICs_Documents/TMC2130_datasheet.pdf
    *
	*/
	case 2130:
		dcode_2130(); break;
#endif //TMC2130

#if defined(FILAMENT_SENSOR) && (FILAMENT_SENSOR_TYPE == FSENSOR_PAT9125)

    /*!
    ### D9125 - PAT9125 filament sensor <a href="https://reprap.org/wiki/G-code#D9:_Read.2FWrite_ADC">D9125: PAT9125 filament sensor</a>
    #### Usage
    
        D9125 [ ? | ! | R | X | Y | L ]
    
    #### Parameters
    - `?` - Print values
    - `!` - Print values
    - `R` - Resolution. Not active in code
    - `X` - X values
    - `Y` - Y values
    - `L` - Activate filament sensor log
    */
	case 9125:
		dcode_9125(); break;
#endif //defined(FILAMENT_SENSOR) && (FILAMENT_SENSOR_TYPE == FSENSOR_PAT9125)

#endif //DEBUG_DCODES

    default:
        printf_P(MSG_UNKNOWN_CODE, 'D', cmdbuffer + bufindr + CMDHDRSIZE);
	}
  }

  else
  {
    SERIAL_ECHO_START;
    SERIAL_ECHORPGM(MSG_UNKNOWN_COMMAND);
    SERIAL_ECHO(CMDBUFFER_CURRENT_STRING);
    SERIAL_ECHOLNPGM("\"(2)");
  }
  KEEPALIVE_STATE(NOT_BUSY);
  ClearToSend();
}

/*!
#### End of D-Codes
*/

/** @defgroup GCodes G-Code List 
*/

// ---------------------------------------------------

void FlushSerialRequestResend()
{
  //char cmdbuffer[bufindr][100]="Resend:";
  MYSERIAL.flush();
  printf_P(_N("%S: %ld\n%S\n"), _n("Resend"), gcode_LastN + 1, MSG_OK);
}

// Confirm the execution of a command, if sent from a serial line.
// Execution of a command from a SD card will not be confirmed.
void ClearToSend()
{
	previous_millis_cmd.start();
	if (buflen && ((CMDBUFFER_CURRENT_TYPE == CMDBUFFER_CURRENT_TYPE_USB) || (CMDBUFFER_CURRENT_TYPE == CMDBUFFER_CURRENT_TYPE_USB_WITH_LINENR)))
		SERIAL_PROTOCOLLNRPGM(MSG_OK);
}

#if MOTHERBOARD == BOARD_RAMBO_MINI_1_0 || MOTHERBOARD == BOARD_RAMBO_MINI_1_3
void update_currents() {
	float current_high[3] = DEFAULT_PWM_MOTOR_CURRENT_LOUD;
	float current_low[3] = DEFAULT_PWM_MOTOR_CURRENT;
	float tmp_motor[3];
	
	//SERIAL_ECHOLNPGM("Currents updated: ");

	if (destination[Z_AXIS] < Z_SILENT) {
		//SERIAL_ECHOLNPGM("LOW");
		for (uint8_t i = 0; i < 3; i++) {
			st_current_set(i, current_low[i]);		
			/*MYSERIAL.print(int(i));
			SERIAL_ECHOPGM(": ");
			MYSERIAL.println(current_low[i]);*/
		}		
	}
	else if (destination[Z_AXIS] > Z_HIGH_POWER) {
		//SERIAL_ECHOLNPGM("HIGH");
		for (uint8_t i = 0; i < 3; i++) {
			st_current_set(i, current_high[i]);
			/*MYSERIAL.print(int(i));
			SERIAL_ECHOPGM(": ");
			MYSERIAL.println(current_high[i]);*/
		}		
	}
	else {
		for (uint8_t i = 0; i < 3; i++) {
			float q = current_low[i] - Z_SILENT*((current_high[i] - current_low[i]) / (Z_HIGH_POWER - Z_SILENT));
			tmp_motor[i] = ((current_high[i] - current_low[i]) / (Z_HIGH_POWER - Z_SILENT))*destination[Z_AXIS] + q;
			st_current_set(i, tmp_motor[i]);			
			/*MYSERIAL.print(int(i));
			SERIAL_ECHOPGM(": ");
			MYSERIAL.println(tmp_motor[i]);*/
		}
	}
}
#endif //MOTHERBOARD == BOARD_RAMBO_MINI_1_0 || MOTHERBOARD == BOARD_RAMBO_MINI_1_3

void get_coordinates() {
  for (uint8_t i = X_AXIS, mask = X_AXIS_MASK; i < NUM_AXIS; i++, mask <<= 1) {
    if(code_seen(axis_codes[i]))
    {
      bool relative = axis_relative_modes & mask;
      destination[i] = code_value();
      if (i == E_AXIS) {
        float emult = extruder_multiplier[active_extruder];
        if (emult != 1.) {
          if (! relative) {
            destination[i] -= current_position[i];
            relative = true;
          }
          destination[i] *= emult;
        }
      }
      if (relative)
        destination[i] += current_position[i];
#if MOTHERBOARD == BOARD_RAMBO_MINI_1_0 || MOTHERBOARD == BOARD_RAMBO_MINI_1_3
	  if (i == Z_AXIS && SilentModeMenu == SILENT_MODE_AUTO) update_currents();
#endif //MOTHERBOARD == BOARD_RAMBO_MINI_1_0 || MOTHERBOARD == BOARD_RAMBO_MINI_1_3
    }
    else destination[i] = current_position[i]; //Are these else lines really needed?
  }
  if(code_seen('F')) {
    const float next_feedrate = code_value();
    if(next_feedrate > 0.f) feedrate = next_feedrate;
  }
}

void clamp_to_software_endstops(float target[3])
{
#ifdef DEBUG_DISABLE_SWLIMITS
	return;
#endif //DEBUG_DISABLE_SWLIMITS
    world2machine_clamp(target[0], target[1]);

    // Clamp the Z coordinate.
    if (min_software_endstops) {
        float negative_z_offset = 0;
        #ifdef ENABLE_AUTO_BED_LEVELING
            if (Z_PROBE_OFFSET_FROM_EXTRUDER < 0) negative_z_offset = negative_z_offset + Z_PROBE_OFFSET_FROM_EXTRUDER;
            if (cs.add_homing[Z_AXIS] < 0) negative_z_offset = negative_z_offset + cs.add_homing[Z_AXIS];
        #endif
        if (target[Z_AXIS] < min_pos[Z_AXIS]+negative_z_offset) target[Z_AXIS] = min_pos[Z_AXIS]+negative_z_offset;
    }
    if (max_software_endstops) {
        if (target[Z_AXIS] > max_pos[Z_AXIS]) target[Z_AXIS] = max_pos[Z_AXIS];
    }
}

uint16_t restore_interrupted_gcode() {
    // When recovering from a previous print move, restore the originally
    // calculated start position on the first USB/SD command. This accounts
    // properly for relative moves
    if (
        (saved_start_position[0] != SAVED_START_POSITION_UNSET) && (
            (CMDBUFFER_CURRENT_TYPE == CMDBUFFER_CURRENT_TYPE_SDCARD) ||
            (CMDBUFFER_CURRENT_TYPE == CMDBUFFER_CURRENT_TYPE_USB_WITH_LINENR)
        )
    ) {
        memcpy(current_position, saved_start_position, sizeof(current_position));
        saved_start_position[0] = SAVED_START_POSITION_UNSET;
        return saved_segment_idx;
    }
    else
        return 1; //begin with the first segment
}

#ifdef MESH_BED_LEVELING
void mesh_plan_buffer_line(const float &x, const float &y, const float &z, const float &e, const float &feed_rate, uint16_t start_segment_idx = 0) {
        float dx = x - current_position[X_AXIS];
        float dy = y - current_position[Y_AXIS];
        uint16_t n_segments = 0;

        if (mbl.active) {
            float len = fabs(dx) + fabs(dy);
            if (len > 0)
                // Split to 3cm segments or shorter.
                n_segments = uint16_t(ceil(len / 30.f));
        }

        if (n_segments > 1 && start_segment_idx) {

            float dz = z - current_position[Z_AXIS];
            float de = e - current_position[E_AXIS];

            for (uint16_t i = start_segment_idx; i < n_segments; ++ i) {
                float t = float(i) / float(n_segments);
                plan_buffer_line(current_position[X_AXIS] + t * dx,
                                 current_position[Y_AXIS] + t * dy,
                                 current_position[Z_AXIS] + t * dz,
                                 current_position[E_AXIS] + t * de,
                                 feed_rate, current_position, i);
                if (planner_aborted)
                    return;
            }
        }
        // The rest of the path.
        plan_buffer_line(x, y, z, e, feed_rate, current_position);
    }
#endif  // MESH_BED_LEVELING
    
void prepare_move(uint16_t start_segment_idx)
{
  clamp_to_software_endstops(destination);
  previous_millis_cmd.start();

  // Do not use feedmultiply for E or Z only moves
  if((current_position[X_AXIS] == destination[X_AXIS]) && (current_position[Y_AXIS] == destination[Y_AXIS])) {
      plan_buffer_line_destinationXYZE(feedrate/60);
  }
  else {
#ifdef MESH_BED_LEVELING
    mesh_plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate*feedmultiply*(1./(60.f*100.f)), start_segment_idx);
#else
     plan_buffer_line_destinationXYZE(feedrate*feedmultiply*(1./(60.f*100.f)));
#endif
  }

  set_current_to_destination();
}

void prepare_arc_move(bool isclockwise, uint16_t start_segment_idx) {
    float r = hypot(offset[X_AXIS], offset[Y_AXIS]); // Compute arc radius for mc_arc
    // Trace the arc
    mc_arc(current_position, destination, offset, (feedrate * feedmultiply) * (1.f / 6000.f), r, isclockwise, start_segment_idx);
    // As far as the parser is concerned, the position is now == target. In reality the
    // motion control system might still be processing the action and the real tool position
    // in any intermediate location.
    set_current_to_destination();
    previous_millis_cmd.start();
}

#if defined(CONTROLLERFAN_PIN) && CONTROLLERFAN_PIN > -1

#if defined(FAN_PIN)
  #if CONTROLLERFAN_PIN == FAN_PIN
    #error "You cannot set CONTROLLERFAN_PIN equal to FAN_PIN"
  #endif
#endif

unsigned long lastMotor = 0; //Save the time for when a motor was turned on last
unsigned long lastMotorCheck = 0;

void controllerFan()
{
  if ((_millis() - lastMotorCheck) >= 2500) //Not a time critical function, so we only check every 2500ms
  {
    lastMotorCheck = _millis();

    if(!READ(X_ENABLE_PIN) || !READ(Y_ENABLE_PIN) || !READ(Z_ENABLE_PIN) || (soft_pwm_bed > 0)
       || !READ(E0_ENABLE_PIN)) //If any of the drivers are enabled...
    {
      lastMotor = _millis(); //... set time to NOW so the fan will turn on
    }

    if ((_millis() - lastMotor) >= (CONTROLLERFAN_SECS*1000UL) || lastMotor == 0) //If the last time any driver was enabled, is longer since than CONTROLLERSEC...
    {
        digitalWrite(CONTROLLERFAN_PIN, 0);
        analogWrite(CONTROLLERFAN_PIN, 0);
    }
    else
    {
        // allows digital or PWM fan output to be used (see M42 handling)
        digitalWrite(CONTROLLERFAN_PIN, CONTROLLERFAN_SPEED);
        analogWrite(CONTROLLERFAN_PIN, CONTROLLERFAN_SPEED);
    }
  }
}
#endif

#ifdef SAFETYTIMER
/**
 * @brief Turn off heating after safetytimer_inactive_time milliseconds of inactivity
 *
 * Full screen blocking notification message is shown after heater turning off.
 * Paused print is not considered inactivity, as nozzle is cooled anyway and bed cooling would
 * damage print.
 *
 * If safetytimer_inactive_time is zero, feature is disabled (heating is never turned off because of inactivity)
 */
static void handleSafetyTimer()
{
    if (printer_active() || !(CHECK_ALL_HEATERS) || !safetytimer_inactive_time)
    {
        safetyTimer.stop();
    }
    else if ((CHECK_ALL_HEATERS) && !safetyTimer.running())
    {
        safetyTimer.start();
    }
    else if (safetyTimer.expired(farm_mode?FARM_DEFAULT_SAFETYTIMER_TIME_ms:safetytimer_inactive_time))
    {
        disable_heater();
        lcd_show_fullscreen_message_and_wait_P(_T(MSG_BED_HEATING_SAFETY_DISABLED));
    }
}
#endif //SAFETYTIMER

void manage_inactivity(bool ignore_stepper_queue/*=false*/) //default argument set in Marlin.h
{
#ifdef FILAMENT_SENSOR
    if (fsensor.update()) {
        lcd_draw_update = 1; //cause lcd update so that fsensor event polling can be done from the lcd draw routine.
    }
#endif

#ifdef SAFETYTIMER
	handleSafetyTimer();
#endif //SAFETYTIMER

#if defined(KILL_PIN) && KILL_PIN > -1
	static int killCount = 0;   // make the inactivity button a bit less responsive
   const int KILL_DELAY = 10000;
#endif
	
    if(buflen < (BUFSIZE-1)){
        get_command();
    }

    if (blocks_queued() && GetPrinterState() == PrinterState::IsHostPrinting && usb_timer.expired((USB_TIMER_TIMEOUT) / 2))
    {
        // Handle the case where planned moves may take a longer time to execute than the USB timer period.
        // An example is the toolchange unload sequence generated by PrusaSlicer with default settings.
        usb_timer.start();
    }

  if(max_inactive_time && previous_millis_cmd.expired(max_inactive_time))
    kill(PSTR("Inactivity Shutdown"));
  if(stepper_inactive_time && previous_millis_cmd.expired(stepper_inactive_time)) {
    if(blocks_queued() == false && ignore_stepper_queue == false) {
      disable_x();
      disable_y();
      disable_z();
      disable_e0();
    }
  }
  
  #ifdef CHDK //Check if pin should be set to LOW after M240 set it to HIGH
    if (chdkActive && (_millis() - chdkHigh > CHDK_DELAY))
    {
      chdkActive = false;
      WRITE(CHDK, LOW);
    }
  #endif
  
  #if defined(KILL_PIN) && KILL_PIN > -1
    
    // Check if the kill button was pressed and wait just in case it was an accidental
    // key kill key press
    // -------------------------------------------------------------------------------
    if( 0 == READ(KILL_PIN) )
    {
       killCount++;
    }
    else if (killCount > 0)
    {
       killCount--;
    }
    // Exceeded threshold and we can confirm that it was not accidental
    // KILL the machine
    // ----------------------------------------------------------------
    if ( killCount >= KILL_DELAY)
    {
       kill();
    }
  #endif
    
  #if defined(CONTROLLERFAN_PIN) && CONTROLLERFAN_PIN > -1
    controllerFan(); //Check if fan should be turned on to cool stepper drivers down
  #endif
  #ifdef EXTRUDER_RUNOUT_PREVENT
    if(previous_millis_cmd.expired(EXTRUDER_RUNOUT_SECONDS*1000))
    if(degHotend(active_extruder)>EXTRUDER_RUNOUT_MINTEMP)
    {
     bool oldstatus=READ(E0_ENABLE_PIN);
     enable_e0();
     float oldepos=current_position[E_AXIS];
     float oldedes=destination[E_AXIS];
     plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS],
                      destination[E_AXIS]+EXTRUDER_RUNOUT_EXTRUDE*EXTRUDER_RUNOUT_ESTEPS/cs.axis_steps_per_mm[E_AXIS],
                      EXTRUDER_RUNOUT_SPEED/60.*EXTRUDER_RUNOUT_ESTEPS/cs.axis_steps_per_mm[E_AXIS]);
     current_position[E_AXIS]=oldepos;
     destination[E_AXIS]=oldedes;
     plan_set_e_position(oldepos);
     previous_millis_cmd.start();
     st_synchronize();
     WRITE(E0_ENABLE_PIN,oldstatus);
    }
  #endif
  check_axes_activity();
  MMU2::mmu2.mmu_loop();

  lcd_knob_update();
  backlight_update();

  // handle longpress
  if(lcd_longpress_trigger)
  {
      lcd_consume_click(); // Reset trigger to prevent recursion
      // long press is not possible in modal mode, wait until ready
      if (lcd_longpress_func && lcd_update_enabled)
      {
          lcd_longpress_func();
      }
  }

#if defined(AUTO_REPORT)
  host_autoreport();
#endif //AUTO_REPORT
  host_keepalive();
  M79_timer_update_status();
}

void kill(const char *full_screen_message) {
    cli(); // Stop interrupts
    disable_heater();

    disable_x();
    disable_y();
    poweroff_z();
    disable_e0();

    SERIAL_ERROR_START;
    SERIAL_ERRORLNRPGM(PSTR("Printer halted. kill() called!"));

    if (full_screen_message != NULL) {
        SERIAL_ERRORLNRPGM(full_screen_message);
    } else {
        full_screen_message = PSTR("KILLED.");
    }

    // update eeprom with the correct kill message to be shown on startup
    eeprom_write_word_notify((uint16_t*)EEPROM_KILL_MESSAGE, (uint16_t)full_screen_message);
    eeprom_write_byte_notify((uint8_t*)EEPROM_KILL_PENDING_FLAG, KILL_PENDING_FLAG);

    softReset();
}

void UnconditionalStop()
{
    CRITICAL_SECTION_START;

    // Disable all heaters and unroll the temperature wait loop stack
    disable_heater();
    cancel_heatup = true;
    heating_status = HeatingStatus::NO_HEATING;

    // Clear any saved printing state
    cancel_saved_printing();

    // Abort the planner
    planner_abort_hard();

    // Reset the queue
    cmdqueue_reset();
    cmdqueue_serial_disabled = false;

    st_reset_timer();
    CRITICAL_SECTION_END;

    // clear paused state immediately
    did_pause_print = false;
    print_job_timer.stop();
}

void ConditionalStop()
{
    CRITICAL_SECTION_START;

    // Don't disable the heaters! lcd_print_stop_finish() will do it later via lcd_cooldown()
    //
    // However, the firmware must take into account the edge case when the firmware
    // is running M109 or M190 G-codes. These G-codes execute a blocking while loop
    // which waits for the bed or nozzle to reach their target temperature.
    // To exit the loop, the firmware must set cancel_heatup to true.
    cancel_heatup = true;
    heating_status = HeatingStatus::NO_HEATING;

    // Clear any saved printing state
    cancel_saved_printing();

    // Abort the planner
    planner_abort_hard();
    
    // Reset the queue
    cmdqueue_reset();
    cmdqueue_serial_disabled = false;

    st_reset_timer();
    CRITICAL_SECTION_END;
}

// Emergency stop used by overtemp functions which allows recovery
// WARNING: This function is called *continuously* during a thermal failure.
//
// This either pauses (for thermal model errors) or stops *without recovery* depending on
// "allow_recovery". If recovery is allowed, this forces a printer-initiated instantanenous pause
// (just like an LCD pause) that bypasses the host pausing functionality. In this state the printer
// is kept in busy state and *must* be recovered from the LCD.
void ThermalStop(bool allow_recovery)
{
    if(Stopped == false) {
        Stopped = true;

        // Either pause or stop the print
        if(allow_recovery && printJobOngoing()) {
            if (!printingIsPaused()) {
                lcd_setalertstatuspgm(_T(MSG_PAUSED_THERMAL_ERROR), LCD_STATUS_CRITICAL);

                // we cannot make a distinction for the host here, the pause must be instantaneous
                // so we call the lcd_pause_print to save the print state internally. Thermal errors
                // disable heaters and save the original temperatures to saved_*, which will get
                // overwritten by stop_and_save_print_to_ram. For this corner-case, re-instate the
                // original values after the pause handler is called.
                uint8_t bed_temp = saved_bed_temperature;
                uint16_t ext_temp = saved_extruder_temperature;
                uint8_t fan_speed = saved_fan_speed;
                lcd_pause_print();
                saved_bed_temperature = bed_temp;
                saved_extruder_temperature = ext_temp;
                saved_fan_speed = fan_speed;
            }
        } else {
            // We got a hard thermal error and/or there is no print going on. Just stop.
            print_stop(false, true);
        }

        // Report the error on the serial
        serialprintPGM(allow_recovery ? echomagic : errormagic);
        SERIAL_ERRORLNRPGM(MSG_ERR_STOPPED);

        // Eventually report the stopped status on the lcd (though this is usually overridden by a
        // higher-priority alert status message)
        LCD_MESSAGERPGM(_T(MSG_STOPPED));

        // Make a warning sound! We cannot use Sound_MakeCustom as this would stop further moves.
        // Turn on the speaker here (if not already), and turn it off when back in the main loop.
        WRITE(BEEPER, HIGH);

        // Always return to the status screen to ensure the NEW error is immediately shown.
        lcd_return_to_status();

        if(!allow_recovery) {
            // prevent menu access for all fatal errors
            menu_set_block(MENU_BLOCK_THERMAL_ERROR);
        }
    }
}

bool IsStopped() { return Stopped; };

void finishAndDisableSteppers()
{
  st_synchronize();
  disable_x();
  disable_y();
  disable_z();
  disable_e0();

#ifndef LA_NOCOMPAT
  // Steppers are disabled both when a print is stopped and also via M84 (which is additionally
  // checked-for to indicate a complete file), so abuse this function to reset the LA detection
  // state for the next print.
  la10c_reset();
#endif

  //in the end of print set estimated time to end of print and extruders used during print to default values for next print
  print_time_remaining_init();
}

#ifdef FAST_PWM_FAN
void setPwmFrequency(uint8_t pin, int val)
{
  val &= 0x07;
  switch(digitalPinToTimer(pin))
  {

    #if defined(TCCR0A)
    case TIMER0A:
    case TIMER0B:
//         TCCR0B &= ~(_BV(CS00) | _BV(CS01) | _BV(CS02));
//         TCCR0B |= val;
         break;
    #endif

    #if defined(TCCR1A)
    case TIMER1A:
    case TIMER1B:
//         TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
//         TCCR1B |= val;
         break;
    #endif

    #if defined(TCCR2)
    case TIMER2:
    case TIMER2:
         TCCR2 &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
         TCCR2 |= val;
         break;
    #endif

    #if defined(TCCR2A)
    case TIMER2A:
    case TIMER2B:
         TCCR2B &= ~(_BV(CS20) | _BV(CS21) | _BV(CS22));
         TCCR2B |= val;
         break;
    #endif

    #if defined(TCCR3A)
    case TIMER3A:
    case TIMER3B:
    case TIMER3C:
         TCCR3B &= ~(_BV(CS30) | _BV(CS31) | _BV(CS32));
         TCCR3B |= val;
         break;
    #endif

    #if defined(TCCR4A)
    case TIMER4A:
    case TIMER4B:
    case TIMER4C:
         TCCR4B &= ~(_BV(CS40) | _BV(CS41) | _BV(CS42));
         TCCR4B |= val;
         break;
   #endif

    #if defined(TCCR5A)
    case TIMER5A:
    case TIMER5B:
    case TIMER5C:
         TCCR5B &= ~(_BV(CS50) | _BV(CS51) | _BV(CS52));
         TCCR5B |= val;
         break;
   #endif

  }
}
#endif //FAST_PWM_FAN

void save_statistics() {
    uint32_t _previous_filament = eeprom_init_default_dword((uint32_t *)EEPROM_FILAMENTUSED, 0); //_previous_filament unit: meter
    uint32_t _previous_time = eeprom_init_default_dword((uint32_t *)EEPROM_TOTALTIME, 0);        //_previous_time unit: min

    uint32_t time_minutes = print_job_timer.duration() / 60;
    eeprom_update_dword_notify((uint32_t *)EEPROM_TOTALTIME, _previous_time + time_minutes); // EEPROM_TOTALTIME unit: min
    eeprom_update_dword_notify((uint32_t *)EEPROM_FILAMENTUSED, _previous_filament + (total_filament_used / 1000));

    print_job_timer.reset();
    total_filament_used = 0;

    if (MMU2::mmu2.Enabled()) {
        eeprom_add_dword((uint32_t *)EEPROM_MMU_MATERIAL_CHANGES, MMU2::mmu2.ToolChangeCounter());
        // @@TODO why were EEPROM_MMU_FAIL_TOT and EEPROM_MMU_LOAD_FAIL_TOT behaving differently - i.e. updated with every change?
        MMU2::mmu2.ClearToolChangeCounter();
        MMU2::mmu2.ClearTMCFailures(); // not stored into EEPROM
    }
}

float calculate_extruder_multiplier(float diameter) {
  float out = 1.f;
  if (cs.volumetric_enabled && diameter > 0.f) {
    float area = M_PI * diameter * diameter * 0.25;
    out = 1.f / area;
  }
  if (extrudemultiply != 100)
    out *= float(extrudemultiply) * 0.01f;
  return out;
}

void calculate_extruder_multipliers() {
	extruder_multiplier[0] = calculate_extruder_multiplier(cs.filament_size[0]);
#if EXTRUDERS > 1
	extruder_multiplier[1] = calculate_extruder_multiplier(cs.filament_size[1]);
#if EXTRUDERS > 2
	extruder_multiplier[2] = calculate_extruder_multiplier(cs.filament_size[2]);
#endif
#endif
}

void delay_keep_alive(unsigned int ms)
{
    for (;;) {
        manage_heater();
        // Manage inactivity, but don't disable steppers on timeout.
        manage_inactivity(true);
        lcd_update(0);
        if (ms == 0)
            break;
        else if (ms >= 50) {
            _delay(50);
            ms -= 50;
        } else {
			_delay(ms);
            ms = 0;
        }
    }
}

void check_babystep()
{
	int babystep_z = eeprom_read_word(reinterpret_cast<uint16_t *>(&(EEPROM_Sheets_base->
            s[(eeprom_read_byte(&(EEPROM_Sheets_base->active_sheet)))].z_offset)));

	if ((babystep_z < Z_BABYSTEP_MIN) || (babystep_z > Z_BABYSTEP_MAX)) {
		babystep_z = 0; //if babystep value is out of min max range, set it to 0
		SERIAL_ECHOLNPGM("Z live adjust out of range. Setting to 0");
		eeprom_write_word_notify(reinterpret_cast<uint16_t *>(&(EEPROM_Sheets_base->
            s[(eeprom_read_byte(&(EEPROM_Sheets_base->active_sheet)))].z_offset)),
                    babystep_z);
		lcd_show_fullscreen_message_and_wait_P(PSTR("Z live adjust out of range. Setting to 0. Click to continue."));
		lcd_update_enable(true);		
	}	
}
#ifdef HEATBED_ANALYSIS
void d_setup()
{	
	pinMode(D_DATACLOCK, INPUT_PULLUP);
	pinMode(D_DATA, INPUT_PULLUP);
	pinMode(D_REQUIRE, OUTPUT);
	digitalWrite(D_REQUIRE, HIGH);
}


float d_ReadData()
{
	int digit[13];
	String mergeOutput;
	float output;

	digitalWrite(D_REQUIRE, HIGH);
	for (int i = 0; i<13; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			while (digitalRead(D_DATACLOCK) == LOW) {}
			while (digitalRead(D_DATACLOCK) == HIGH) {}
			bitWrite(digit[i], j, digitalRead(D_DATA));
		}
	}

	digitalWrite(D_REQUIRE, LOW);
	mergeOutput = "";
	output = 0;
	for (int r = 5; r <= 10; r++) //Merge digits
	{
		mergeOutput += digit[r];
	}
	output = mergeOutput.toFloat();

	if (digit[4] == 8) //Handle sign
	{
		output *= -1;
	}

	for (int i = digit[11]; i > 0; i--) //Handle floating point
	{
		output /= 10;
	}

	return output;

}

void bed_check(float x_dimension, float y_dimension, int x_points_num, int y_points_num, float shift_x, float shift_y) {
	int t1 = 0;
	int t_delay = 0;
	int digit[13];
	int m;
	char str[3];
	//String mergeOutput;
	char mergeOutput[15];
	float output;

	int mesh_point = 0; //index number of calibration point
	float bed_zero_ref_x = (-22.f + X_PROBE_OFFSET_FROM_EXTRUDER); //shift between zero point on bed and target and between probe and nozzle
	float bed_zero_ref_y = (-0.6f + Y_PROBE_OFFSET_FROM_EXTRUDER);

	float mesh_home_z_search = 4;
	float measure_z_height = 0.2f;
	float row[x_points_num];
	int ix = 0;
	int iy = 0;

	const char* filename_wldsd = "mesh.txt";
	char data_wldsd[x_points_num * 7 + 1]; //6 chars(" -A.BCD")for each measurement + null 
	char numb_wldsd[8]; // (" -A.BCD" + null)
#ifdef MICROMETER_LOGGING
	d_setup();
#endif //MICROMETER_LOGGING

	int XY_AXIS_FEEDRATE = homing_feedrate[X_AXIS] / 20;
	int Z_LIFT_FEEDRATE = homing_feedrate[Z_AXIS] / 40;

	unsigned int custom_message_type_old = custom_message_type;
	unsigned int custom_message_state_old = custom_message_state;
	custom_message_type = CustomMsg::MeshBedLeveling;
	custom_message_state = (x_points_num * y_points_num) + 10;
	lcd_update(1);

	//mbl.reset();
	babystep_undo();

	card.openFile(filename_wldsd, false);

	/*destination[Z_AXIS] = mesh_home_z_search;
	//plan_buffer_line_curposXYZE(Z_LIFT_FEEDRATE);

	plan_buffer_line_destinationXYZE(Z_LIFT_FEEDRATE);
	for(int8_t i=0; i < NUM_AXIS; i++) {
		current_position[i] = destination[i];
	}
	st_synchronize();
	*/
		destination[Z_AXIS] = measure_z_height;
		plan_buffer_line_destinationXYZE(Z_LIFT_FEEDRATE);
		for(int8_t i=0; i < NUM_AXIS; i++) {
			current_position[i] = destination[i];
		}
		st_synchronize();
	/*int l_feedmultiply = */setup_for_endstop_move(false);

	SERIAL_PROTOCOLPGM("Num X,Y: ");
	SERIAL_PROTOCOL(x_points_num);
	SERIAL_PROTOCOLPGM(",");
	SERIAL_PROTOCOL(y_points_num);
	SERIAL_PROTOCOLPGM("\nZ search height: ");
	SERIAL_PROTOCOL(mesh_home_z_search);
	SERIAL_PROTOCOLPGM("\nDimension X,Y: ");
	SERIAL_PROTOCOL(x_dimension);
	SERIAL_PROTOCOLPGM(",");
	SERIAL_PROTOCOL(y_dimension);
	SERIAL_PROTOCOLLNPGM("\nMeasured points:");

	while (mesh_point != x_points_num * y_points_num) {
		ix = mesh_point % x_points_num; // from 0 to MESH_NUM_X_POINTS - 1
		iy = mesh_point / x_points_num;
		if (iy & 1) ix = (x_points_num - 1) - ix; // Zig zag
		float z0 = 0.f;
		/*destination[Z_AXIS] = mesh_home_z_search;
		//plan_buffer_line_curposXYZE(Z_LIFT_FEEDRATE);

		plan_buffer_line_destinationXYZE(Z_LIFT_FEEDRATE);
		for(int8_t i=0; i < NUM_AXIS; i++) {
			current_position[i] = destination[i];
		}
		st_synchronize();*/


		//current_position[X_AXIS] = 13.f + ix * (x_dimension / (x_points_num - 1)) - bed_zero_ref_x + shift_x;
		//current_position[Y_AXIS] = 6.4f + iy * (y_dimension / (y_points_num - 1)) - bed_zero_ref_y + shift_y;

		destination[X_AXIS] = ix * (x_dimension / (x_points_num - 1)) + shift_x;
		destination[Y_AXIS] = iy * (y_dimension / (y_points_num - 1)) + shift_y;

		mesh_plan_buffer_line_destinationXYZE(XY_AXIS_FEEDRATE/6);
		set_current_to_destination();
		st_synchronize();

	//	printf_P(PSTR("X = %f; Y= %f \n"), current_position[X_AXIS], current_position[Y_AXIS]);

		delay_keep_alive(1000);
#ifdef MICROMETER_LOGGING

		//memset(numb_wldsd, 0, sizeof(numb_wldsd));
		//dtostrf(d_ReadData(), 8, 5, numb_wldsd);
		//strcat(data_wldsd, numb_wldsd);


		
		//MYSERIAL.println(data_wldsd);
		//delay(1000);
		//delay(3000);
		//t1 = millis();
		
		//while (digitalRead(D_DATACLOCK) == LOW) {}
		//while (digitalRead(D_DATACLOCK) == HIGH) {}
		memset(digit, 0, sizeof(digit));
		//cli();
		digitalWrite(D_REQUIRE, LOW);	
		
		for (int i = 0; i<13; i++)
		{
			//t1 = millis();
			for (int j = 0; j < 4; j++)
			{
				while (digitalRead(D_DATACLOCK) == LOW) {}				
				while (digitalRead(D_DATACLOCK) == HIGH) {}
				//printf_P(PSTR("Done %d\n"), j);
				bitWrite(digit[i], j, digitalRead(D_DATA));
			}
			//t_delay = (millis() - t1);
			//SERIAL_PROTOCOLPGM(" ");
			//SERIAL_PROTOCOL_F(t_delay, 5);
			//SERIAL_PROTOCOLPGM(" ");

		}
		//sei();
		digitalWrite(D_REQUIRE, HIGH);
		mergeOutput[0] = '\0';
		output = 0;
		for (int r = 5; r <= 10; r++) //Merge digits
		{			
			sprintf(str, "%d", digit[r]);
			strcat(mergeOutput, str);
		}
		
		output = atof(mergeOutput);

		if (digit[4] == 8) //Handle sign
		{
			output *= -1;
		}

		for (int i = digit[11]; i > 0; i--) //Handle floating point
		{
			output *= 0.1;
		}
		

		//output = d_ReadData();

		//row[ix] = current_position[Z_AXIS];


		
		//row[ix] = d_ReadData();
		
		row[ix] = output;

		if (iy % 2 == 1 ? ix == 0 : ix == x_points_num - 1) {
			memset(data_wldsd, 0, sizeof(data_wldsd));
			for (int i = 0; i < x_points_num; i++) {
				SERIAL_PROTOCOLPGM(" ");
				SERIAL_PROTOCOL_F(row[i], 5);
				memset(numb_wldsd, 0, sizeof(numb_wldsd));
				dtostrf(row[i], 7, 3, numb_wldsd);
				strcat(data_wldsd, numb_wldsd);
			}
			card.write_command(data_wldsd);
			SERIAL_PROTOCOLPGM("\n");

		}

		custom_message_state--;
		mesh_point++;
		lcd_update(1);

	}
	#endif //MICROMETER_LOGGING
	card.closefile();
	//clean_up_after_endstop_move(l_feedmultiply);

}

void bed_analysis(float x_dimension, float y_dimension, int x_points_num, int y_points_num, float shift_x, float shift_y) {
	int t1 = 0;
	int t_delay = 0;
	int digit[13];
	int m;
	char str[3];
	//String mergeOutput;
	char mergeOutput[15];
	float output;

	int mesh_point = 0; //index number of calibration point
	float bed_zero_ref_x = (-22.f + X_PROBE_OFFSET_FROM_EXTRUDER); //shift between zero point on bed and target and between probe and nozzle
	float bed_zero_ref_y = (-0.6f + Y_PROBE_OFFSET_FROM_EXTRUDER);

	float mesh_home_z_search = 4;
	float row[x_points_num];
	int ix = 0;
	int iy = 0;

	const char* filename_wldsd = "wldsd.txt";
	char data_wldsd[70];
	char numb_wldsd[10];

	d_setup();

	if (!(axis_known_position[X_AXIS] && axis_known_position[Y_AXIS] && axis_known_position[Z_AXIS])) {
		// We don't know where we are! HOME!
		// Push the commands to the front of the message queue in the reverse order!
		// There shall be always enough space reserved for these commands.
		repeatcommand_front(); // repeat G80 with all its parameters
		
		enquecommand_front_P(G28W);
		enquecommand_front_P((PSTR("G1 Z5")));
		return;
	}
	unsigned int custom_message_type_old = custom_message_type;
	unsigned int custom_message_state_old = custom_message_state;
	custom_message_type = CustomMsg::MeshBedLeveling;
	custom_message_state = (x_points_num * y_points_num) + 10;
	lcd_update(1);

	mbl.reset();
	babystep_undo();

	card.openFile(filename_wldsd, false);

	current_position[Z_AXIS] = mesh_home_z_search;
	plan_buffer_line_curposXYZE(homing_feedrate[Z_AXIS] / 60, active_extruder);

	int XY_AXIS_FEEDRATE = homing_feedrate[X_AXIS] / 20;
	int Z_LIFT_FEEDRATE = homing_feedrate[Z_AXIS] / 40;

	int l_feedmultiply = setup_for_endstop_move(false);

	SERIAL_PROTOCOLPGM("Num X,Y: ");
	SERIAL_PROTOCOL(x_points_num);
	SERIAL_PROTOCOLPGM(",");
	SERIAL_PROTOCOL(y_points_num);
	SERIAL_PROTOCOLPGM("\nZ search height: ");
	SERIAL_PROTOCOL(mesh_home_z_search);
	SERIAL_PROTOCOLPGM("\nDimension X,Y: ");
	SERIAL_PROTOCOL(x_dimension);
	SERIAL_PROTOCOLPGM(",");
	SERIAL_PROTOCOL(y_dimension);
	SERIAL_PROTOCOLLNPGM("\nMeasured points:");

	while (mesh_point != x_points_num * y_points_num) {
		ix = mesh_point % x_points_num; // from 0 to MESH_NUM_X_POINTS - 1
		iy = mesh_point / x_points_num;
		if (iy & 1) ix = (x_points_num - 1) - ix; // Zig zag
		float z0 = 0.f;
		current_position[Z_AXIS] = mesh_home_z_search;
		plan_buffer_line_curposXYZE(Z_LIFT_FEEDRATE, active_extruder);
		st_synchronize();


		current_position[X_AXIS] = 13.f + ix * (x_dimension / (x_points_num - 1)) - bed_zero_ref_x + shift_x;
		current_position[Y_AXIS] = 6.4f + iy * (y_dimension / (y_points_num - 1)) - bed_zero_ref_y + shift_y;

		plan_buffer_line_curposXYZE(XY_AXIS_FEEDRATE, active_extruder);
		st_synchronize();

		if (!find_bed_induction_sensor_point_z(-10.f)) { //if we have data from z calibration max allowed difference is 1mm for each point, if we dont have data max difference is 10mm from initial point  
			break;
			card.closefile();
		}


		//memset(numb_wldsd, 0, sizeof(numb_wldsd));
		//dtostrf(d_ReadData(), 8, 5, numb_wldsd);
		//strcat(data_wldsd, numb_wldsd);


		
		//MYSERIAL.println(data_wldsd);
		//_delay(1000);
		//_delay(3000);
		//t1 = _millis();
		
		//while (digitalRead(D_DATACLOCK) == LOW) {}
		//while (digitalRead(D_DATACLOCK) == HIGH) {}
		memset(digit, 0, sizeof(digit));
		//cli();
		digitalWrite(D_REQUIRE, LOW);	
		
		for (int i = 0; i<13; i++)
		{
			//t1 = _millis();
			for (int j = 0; j < 4; j++)
			{
				while (digitalRead(D_DATACLOCK) == LOW) {}				
				while (digitalRead(D_DATACLOCK) == HIGH) {}
				bitWrite(digit[i], j, digitalRead(D_DATA));
			}
			//t_delay = (_millis() - t1);
			//SERIAL_PROTOCOLPGM(" ");
			//SERIAL_PROTOCOL_F(t_delay, 5);
			//SERIAL_PROTOCOLPGM(" ");
		}
		//sei();
		digitalWrite(D_REQUIRE, HIGH);
		mergeOutput[0] = '\0';
		output = 0;
		for (int r = 5; r <= 10; r++) //Merge digits
		{			
			sprintf(str, "%d", digit[r]);
			strcat(mergeOutput, str);
		}
		
		output = atof(mergeOutput);

		if (digit[4] == 8) //Handle sign
		{
			output *= -1;
		}

		for (int i = digit[11]; i > 0; i--) //Handle floating point
		{
			output *= 0.1;
		}
		

		//output = d_ReadData();

		//row[ix] = current_position[Z_AXIS];

		memset(data_wldsd, 0, sizeof(data_wldsd));

		for (int i = 0; i <3; i++) {
			memset(numb_wldsd, 0, sizeof(numb_wldsd));
			dtostrf(current_position[i], 8, 5, numb_wldsd);
			strcat(data_wldsd, numb_wldsd);
			strcat(data_wldsd, ";");

		}
		memset(numb_wldsd, 0, sizeof(numb_wldsd));
		dtostrf(output, 8, 5, numb_wldsd);
		strcat(data_wldsd, numb_wldsd);
		//strcat(data_wldsd, ";");
		card.write_command(data_wldsd);

		
		//row[ix] = d_ReadData();
		
		row[ix] = output; // current_position[Z_AXIS];

		if (iy % 2 == 1 ? ix == 0 : ix == x_points_num - 1) {
			for (int i = 0; i < x_points_num; i++) {
				SERIAL_PROTOCOLPGM(" ");
				SERIAL_PROTOCOL_F(row[i], 5);


			}
			SERIAL_PROTOCOLPGM("\n");
		}
		custom_message_state--;
		mesh_point++;
		lcd_update(1);

	}
	card.closefile();
	clean_up_after_endstop_move(l_feedmultiply);
}
#endif //HEATBED_ANALYSIS

#ifndef PINDA_THERMISTOR
static void temp_compensation_start() {
	custom_message_type = CustomMsg::TempCompPreheat;
	custom_message_state = PINDA_HEAT_T + 1;
	lcd_update(2);
	if (!temp_compensation_retracted && (int)degHotend(active_extruder) > extrude_min_temp) {
		temp_compensation_retracted = true;
		current_position[E_AXIS] -= default_retraction;
	}
	plan_buffer_line_curposXYZE(400);
	
	current_position[X_AXIS] = PINDA_PREHEAT_X;
	current_position[Y_AXIS] = PINDA_PREHEAT_Y;
	current_position[Z_AXIS] = PINDA_PREHEAT_Z;
	plan_buffer_line_curposXYZE(3000 / 60);
	st_synchronize();
	while (fabs(degBed() - target_temperature_bed) > 1) delay_keep_alive(1000);

	for (int i = 0; i < PINDA_HEAT_T; i++) {
		delay_keep_alive(1000);
		custom_message_state = PINDA_HEAT_T - i;
		if (custom_message_state == 99 || custom_message_state == 9) lcd_update(2); //force whole display redraw if number of digits changed
		else lcd_update(1);
	}	
	custom_message_type = CustomMsg::Status;
	custom_message_state = 0;
}

static void temp_compensation_apply() {
	int i_add;
	int z_shift = 0;
	float z_shift_mm;

	if (calibration_status_pinda()) {
		if (target_temperature_bed % 10 == 0 && target_temperature_bed >= 60 && target_temperature_bed <= 100) {
			i_add = (target_temperature_bed - 60) / 10;
			z_shift = eeprom_read_word((uint16_t*)EEPROM_PROBE_TEMP_SHIFT + i_add);
			z_shift_mm = z_shift / cs.axis_steps_per_mm[Z_AXIS];
		}else {
			//interpolation
			z_shift_mm = temp_comp_interpolation(target_temperature_bed) / cs.axis_steps_per_mm[Z_AXIS];
		}
		printf_P(_N("\nZ shift applied:%.3f\n"), z_shift_mm);
		plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] - z_shift_mm, current_position[E_AXIS], homing_feedrate[Z_AXIS] / 40);
		st_synchronize();
		plan_set_z_position(current_position[Z_AXIS]);
	}
	else {		
		//we have no temp compensation data
	}
}
#endif //ndef PINDA_THERMISTOR

float temp_comp_interpolation(float inp_temperature) {

	//cubic spline interpolation

	int n, i, j;
	float h[10], a, b, c, d, sum, s[10] = { 0 }, x[10], F[10], f[10], m[10][10] = { 0 }, temp;
	int shift[10];
	int temp_C[10];

	n = 6; //number of measured points

	shift[0] = 0;
	for (i = 0; i < n; i++) {
		if (i > 0) {
			//read shift in steps from EEPROM
			shift[i] = eeprom_read_word((uint16_t*)EEPROM_PROBE_TEMP_SHIFT + (i - 1));
		}
		temp_C[i] = 50 + i * 10; //temperature in C
#ifdef PINDA_THERMISTOR
		constexpr int start_compensating_temp = 35;
		temp_C[i] = start_compensating_temp + i * 5; //temperature in degrees C
#ifdef SUPERPINDA_SUPPORT
    static_assert(start_compensating_temp >= PINDA_MINTEMP, "Temperature compensation start point is lower than PINDA_MINTEMP.");
#endif //SUPERPINDA_SUPPORT
#else
		temp_C[i] = 50 + i * 10; //temperature in C
#endif
		x[i] = (float)temp_C[i];
		f[i] = (float)shift[i];
	}
	if (inp_temperature < x[0]) return 0;


	for (i = n - 1; i>0; i--) {
		F[i] = (f[i] - f[i - 1]) / (x[i] - x[i - 1]);
		h[i - 1] = x[i] - x[i - 1];
	}
	//*********** formation of h, s , f matrix **************
	for (i = 1; i<n - 1; i++) {
		m[i][i] = 2 * (h[i - 1] + h[i]);
		if (i != 1) {
			m[i][i - 1] = h[i - 1];
			m[i - 1][i] = h[i - 1];
		}
		m[i][n - 1] = 6 * (F[i + 1] - F[i]);
	}
	//*********** forward elimination **************
	for (i = 1; i<n - 2; i++) {
		temp = (m[i + 1][i] / m[i][i]);
		for (j = 1; j <= n - 1; j++)
			m[i + 1][j] -= temp*m[i][j];
	}
	//*********** backward substitution *********
	for (i = n - 2; i>0; i--) {
		sum = 0;
		for (j = i; j <= n - 2; j++)
			sum += m[i][j] * s[j];
		s[i] = (m[i][n - 1] - sum) / m[i][i];
	}

		for (i = 0; i<n - 1; i++)
			if ((x[i] <= inp_temperature && inp_temperature <= x[i + 1]) || (i == n-2 && inp_temperature > x[i + 1])) {
				a = (s[i + 1] - s[i]) / (6 * h[i]);
				b = s[i] / 2;
				c = (f[i + 1] - f[i]) / h[i] - (2 * h[i] * s[i] + s[i + 1] * h[i]) / 6;
				d = f[i];
				sum = a*pow((inp_temperature - x[i]), 3) + b*pow((inp_temperature - x[i]), 2) + c*(inp_temperature - x[i]) + d;
			}

		return sum;

}

#ifdef PINDA_THERMISTOR
float temp_compensation_pinda_thermistor_offset(float temperature_pinda)
{
	if (!eeprom_read_byte((unsigned char *)EEPROM_TEMP_CAL_ACTIVE)) return 0;
	if (!calibration_status_pinda()) return 0;
	return temp_comp_interpolation(temperature_pinda) / cs.axis_steps_per_mm[Z_AXIS];
}
#endif //PINDA_THERMISTOR

void long_pause() //long pause print
{
	st_synchronize();

    // Stop heaters
    heating_status = HeatingStatus::NO_HEATING;
    setTargetHotend(0);

    // Lift z
    raise_z(pause_position[Z_AXIS]);

    // Move XY to side
    if (axis_known_position[X_AXIS] && axis_known_position[Y_AXIS]) {
        current_position[X_AXIS] = pause_position[X_AXIS];
        current_position[Y_AXIS] = pause_position[Y_AXIS];
        plan_buffer_line_curposXYZE(50);
    }

    // did we come here from a thermal error?
    if(get_temp_error()) {
        // time to stop the error beep
        WRITE(BEEPER, LOW);
    } else {
        // Turn off the print fan
        fanSpeed = 0;
    }
}

void serialecho_temperatures() {
	float tt = degHotend(active_extruder);
	SERIAL_PROTOCOLPGM("T:");
	SERIAL_PROTOCOL(tt);
	SERIAL_PROTOCOLPGM(" E:0 B:");
	SERIAL_PROTOCOL_F(degBed(), 1);
	SERIAL_PROTOCOLLN();
}

void save_print_file_state() {
    uint8_t nlines;
    uint16_t sdlen_cmdqueue;
    uint16_t sdlen_planner;

    if (card.sdprinting) {
        saved_sdpos = sdpos_atomic; //atomic sd position of last command added in queue
        sdlen_planner = planner_calc_sd_length(); //length of sd commands in planner
        saved_sdpos -= sdlen_planner;
        sdlen_cmdqueue = cmdqueue_calc_sd_length(); //length of sd commands in cmdqueue
        saved_sdpos -= sdlen_cmdqueue;
        saved_printing_type = PowerPanic::PRINT_TYPE_SD;
    }
    else if (usb_timer.running()) { //reuse saved_sdpos for storing line number
        saved_sdpos = gcode_LastN; //start with line number of command added recently to cmd queue
        //reuse planner_calc_sd_length function for getting number of lines of commands in planner:
        nlines = planner_calc_sd_length(); //number of lines of commands in planner 
        saved_sdpos -= nlines;
        saved_sdpos -= buflen; //number of blocks in cmd buffer
        saved_printing_type = PowerPanic::PRINT_TYPE_HOST;
    }
    else {
        saved_printing_type = PowerPanic::PRINT_TYPE_NONE;
        //not sd printing nor usb printing
    }

}

void restore_print_file_state() {
    if (saved_printing_type == PowerPanic::PRINT_TYPE_SD) { //was sd printing
        card.setIndex(saved_sdpos);
        sdpos_atomic = saved_sdpos;
        card.sdprinting = true;
    } else if (saved_printing_type == PowerPanic::PRINT_TYPE_HOST) { //was usb printing
        gcode_LastN = saved_sdpos; //saved_sdpos was reused for storing line number when usb printing
        serial_count = 0; 
        FlushSerialRequestResend();
    } else {
      //not sd printing nor usb printing
    }
}

void save_planner_global_state() {
    if (current_block && !(mesh_bed_leveling_flag || homing_flag))
    {
        memcpy(saved_start_position, current_block->gcode_start_position, sizeof(saved_start_position));
        saved_feedrate2 = current_block->gcode_feedrate;
        saved_segment_idx = current_block->segment_idx;
    }
    else
    {
        saved_start_position[0] = SAVED_START_POSITION_UNSET;
        saved_feedrate2 = feedrate;
        saved_segment_idx = 0;
    }
}

/// Take a backup of the current state of variables
/// e.g. feedrate, Z-axis position etc.
/// This function should backup variables which may be lost
/// For example a power panic in M600 or during MMU error
void refresh_print_state_in_ram()
{
    if (saved_printing) return;
    memcpy(saved_pos, current_position, sizeof(saved_pos));
    saved_feedmultiply2 = feedmultiply; //save feedmultiply
    saved_extruder_temperature = (uint16_t)degTargetHotend(active_extruder);
    saved_bed_temperature = (uint8_t)degTargetBed();
    saved_extruder_relative_mode = axis_relative_modes & E_AXIS_MASK;
    saved_fan_speed = fanSpeed;
    isPartialBackupAvailable = true;
}

void __attribute__((noinline)) refresh_saved_feedrate_multiplier_in_ram() {
    if (!saved_printing) {
        // There is no saved print, therefore nothing to refresh
        return;
    }

    saved_feedmultiply2 = feedmultiply;
}

void clear_print_state_in_ram()
{
    // Set flag to false in order to avoid using
    // the saved values during power panic
    isPartialBackupAvailable = false;
}

//! @brief Immediately stop print moves
//!
//! Immediately stop print moves, save current extruder temperature and position to RAM.
//! If printing from sd card, position in file is saved.
//! If printing from USB, line number is saved.
//!
//! @param z_move
//! @param e_move
void stop_and_save_print_to_ram(float z_move, float e_move)
{
	if (saved_printing) return;

	cli();
	save_print_file_state();

    // save the global state at planning time
    const bool pos_invalid = mesh_bed_leveling_flag || homing_flag;
    save_planner_global_state();

	planner_abort_hard(); //abort printing

	memcpy(saved_pos, current_position, sizeof(saved_pos));
    if (pos_invalid) saved_pos[X_AXIS] = X_COORD_INVALID;

	cmdqueue_reset(); //empty cmdqueue
//	card.closefile();
	saved_printing = true;
  // We may have missed a stepper timer interrupt. Be safe than sorry, reset the stepper timer before re-enabling interrupts.
  st_reset_timer();
	sei();
	if ((z_move != 0) || (e_move != 0)) { // extruder or z move

    // Rather than calling plan_buffer_line directly, push the move into the command queue so that
    // the caller can continue processing. This is used during powerpanic to save the state as we
    // move away from the print.

    if(e_move)
    {
        // First unretract (relative extrusion)
        if(!saved_extruder_relative_mode){
            enquecommand_P(MSG_M83);
        }
        //retract 45mm/s
        // A single sprintf may not be faster, but is definitely 20B shorter
        // than a sequence of commands building the string piece by piece
        // A snprintf would have been a safer call, but since it is not used
        // in the whole program, its implementation would bring more bytes to the total size
        // The behavior of dtostrf 8,3 should be roughly the same as %-0.3
        enquecommandf_P(G1_E_F2700, e_move);
    }

    if(z_move)
    {
        // Then lift Z axis
        enquecommandf_P(PSTR("G1 Z%-.3f F%-.3f"), saved_pos[Z_AXIS] + z_move, homing_feedrate[Z_AXIS]);
    }

    // If this call is invoked from the main Arduino loop() function, let the caller know that the command
    // in the command queue is not the original command, but a new one, so it should not be removed from the queue.
    repeatcommand_front();
  }
}

/// @brief Read saved filename from EEPROM and send g-code command: M23 <filename>
void restore_file_from_sd()
{
    char filename[FILENAME_LENGTH];
    char dir_name[9];
    char extension_ptr[5];
    uint8_t depth = eeprom_read_byte((uint8_t*)EEPROM_DIR_DEPTH);

    for (uint8_t i = 0; i < depth; i++) {
        eeprom_read_block(dir_name, (const char *)EEPROM_DIRS + 8 * i, 8);
        dir_name[8] = '\0';
        card.chdir(dir_name, false);
    }

    // Recover DOS 8.3 filename without extension.
    // Short filenames are always null terminated.
    eeprom_read_block(filename, (const char *)EEPROM_FILENAME, 8);

    // Add null delimiter in case all 8 characters were not NULL
    filename[8] = '\0';

    // Add extension to complete the DOS 8.3 filename e.g. ".gco" or ".g"
    extension_ptr[0] = '.';
    eeprom_read_block(&extension_ptr[1], (const char *)EEPROM_FILENAME_EXTENSION, 3);
    extension_ptr[4] = '\0';
    strcat(filename, extension_ptr);

    enquecommandf_P(MSG_M23, filename);
}

//! @brief Restore print from ram
//!
//! Restore print saved by stop_and_save_print_to_ram(). Is blocking, restores
//! print fan speed, waits for extruder temperature restore, then restores
//! position and continues print moves.
//!
//! Internally lcd_update() is called by wait_for_heater().
//!
//! @param e_move
void restore_print_from_ram_and_continue(float e_move)
{
    if (!saved_printing) return;

    // do not restore XY for commands that do not require that
    if (saved_pos[X_AXIS] == X_COORD_INVALID)
    {
        saved_pos[X_AXIS] = current_position[X_AXIS];
        saved_pos[Y_AXIS] = current_position[Y_AXIS];
    }

	//first move print head in XY to the saved position:
	plan_buffer_line(saved_pos[X_AXIS], saved_pos[Y_AXIS], current_position[Z_AXIS], saved_pos[E_AXIS] - e_move, homing_feedrate[Z_AXIS]/13);
	//then move Z
	plan_buffer_line(saved_pos[X_AXIS], saved_pos[Y_AXIS], saved_pos[Z_AXIS], saved_pos[E_AXIS] - e_move, homing_feedrate[Z_AXIS]/13);
	//and finaly unretract (35mm/s)
	plan_buffer_line(saved_pos[X_AXIS], saved_pos[Y_AXIS], saved_pos[Z_AXIS], saved_pos[E_AXIS], FILAMENTCHANGE_RFEED);
	st_synchronize();

  #ifdef FANCHECK
    fans_check_enabled = true;
  #endif

    // restore original feedrate/feedmultiply _after_ restoring the extruder position
	feedrate = saved_feedrate2;
	feedmultiply = saved_feedmultiply2;

	memcpy(current_position, saved_pos, sizeof(saved_pos));
	set_destination_to_current();

    restore_print_file_state();
    eeprom_update_byte_notify((uint8_t*)EEPROM_UVLO, PowerPanic::NO_PENDING_RECOVERY);
    eeprom_update_byte_notify((uint8_t*)EEPROM_UVLO_Z_LIFTED, 0);
	lcd_setstatuspgm(MSG_WELCOME);
    saved_printing_type = PowerPanic::PRINT_TYPE_NONE;
	saved_printing = false;
    planner_aborted = true; // unroll the stack
}

// Cancel the state related to a currently saved print
void cancel_saved_printing()
{
    eeprom_update_byte_notify((uint8_t*)EEPROM_UVLO, PowerPanic::NO_PENDING_RECOVERY);
    eeprom_update_byte_notify((uint8_t*)EEPROM_UVLO_Z_LIFTED, 0);
    saved_start_position[0] = SAVED_START_POSITION_UNSET;
    saved_printing_type = PowerPanic::PRINT_TYPE_NONE;
    saved_printing = false;
}

void print_world_coordinates()
{
	printf_P(_N("world coordinates: (%.3f, %.3f, %.3f)\n"), current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]);
}

void print_physical_coordinates()
{
	printf_P(_N("physical coordinates: (%.3f, %.3f, %.3f)\n"), st_get_position_mm(X_AXIS), st_get_position_mm(Y_AXIS), st_get_position_mm(Z_AXIS));
}

uint8_t calc_percent_done()
{
    //in case that we have information from M73 gcode return percentage counted by slicer, else return percentage counted as byte_printed/filesize
    uint8_t percent_done = 0;
#ifdef TMC2130
    if (SilentModeMenu == SILENT_MODE_OFF && print_percent_done_normal <= 100)
    {
        percent_done = print_percent_done_normal;
    }
    else if (print_percent_done_silent <= 100)
    {
        percent_done = print_percent_done_silent;
    }
#else
    if (print_percent_done_normal <= 100)
    {
        percent_done = print_percent_done_normal;
    }
#endif //TMC2130
    else
    {
        percent_done = card.percentDone();
    }
    return percent_done;
}

static void print_time_remaining_init()
{
    print_time_remaining_normal = PRINT_TIME_REMAINING_INIT;
    print_percent_done_normal = PRINT_PERCENT_DONE_INIT;
    print_time_remaining_silent = PRINT_TIME_REMAINING_INIT;
    print_percent_done_silent = PRINT_PERCENT_DONE_INIT;
    print_time_to_change_normal = PRINT_TIME_REMAINING_INIT;
    print_time_to_change_silent = PRINT_TIME_REMAINING_INIT;
}

void load_filament_final_feed()
{
	current_position[E_AXIS]+= FILAMENTCHANGE_FINALFEED;
	plan_buffer_line_curposXYZE(FILAMENTCHANGE_EFEED_FINAL);
}

//! @brief Wait for user action
//!
//! Beep, manage nozzle heater and wait for user to start unload filament
//! If times out, active extruder temperature is set to 0.
void M600_wait_for_user() {

		KEEPALIVE_STATE(PAUSED_FOR_USER);

		unsigned long waiting_start_time = _millis();
		uint8_t wait_for_user_state = 0;
		lcd_display_message_fullscreen_P(_T(MSG_PRESS_TO_UNLOAD));

		while (!(wait_for_user_state == 0 && lcd_clicked())){
			manage_heater();
			manage_inactivity(true);
      if (wait_for_user_state != 2) sound_wait_for_user();
			
			switch (wait_for_user_state) {
			case 0: //nozzle is hot, waiting for user to press the knob to unload filament
				delay_keep_alive(4);

				if (_millis() > waiting_start_time + (unsigned long)M600_TIMEOUT * 1000) {
					lcd_display_message_fullscreen_P(_T(MSG_PRESS_TO_PREHEAT));
					wait_for_user_state = 1;
					setTargetHotend(0);
					st_synchronize();
					disable_e0();
				}
				break;
			case 1: //nozzle target temperature is set to zero, waiting for user to start nozzle preheat
				delay_keep_alive(4);
		
				if (lcd_clicked()) {
					setTargetHotend(saved_extruder_temperature);
					lcd_wait_for_heater();
					wait_for_user_state = 2;
				}
				break;
			case 2: //waiting for nozzle to reach target temperature
				if (fabs(degTargetHotend(active_extruder) - degHotend(active_extruder)) < TEMP_HYSTERESIS) {
					lcd_display_message_fullscreen_P(_T(MSG_PRESS_TO_UNLOAD));
					waiting_start_time = _millis();
					wait_for_user_state = 0;
				} else {
					lcd_set_cursor(1, 4);
					lcd_printf_P(PSTR("%3d"), (int16_t)degHotend(active_extruder));
				}
				break;
			}
		}
		sound_wait_for_user_reset();
}

void M600_load_filament_movements()
{
	current_position[E_AXIS]+= FILAMENTCHANGE_FIRSTFEED;
	plan_buffer_line_curposXYZE(FILAMENTCHANGE_EFEED_FIRST);
	load_filament_final_feed();
	lcd_loading_filament();
	st_synchronize();
}

void M600_load_filament() {
	//load filament for single material and MMU
	lcd_wait_interact();

	//load_filament_time = _millis();
	KEEPALIVE_STATE(PAUSED_FOR_USER);

	while(!lcd_clicked())
	{
		manage_heater();
		manage_inactivity(true);
#ifdef FILAMENT_SENSOR
		if (fsensor.getFilamentLoadEvent()) {
			Sound_MakeCustom(50,1000,false);
			break;
		}
#endif //FILAMENT_SENSOR
	}
	KEEPALIVE_STATE(IN_HANDLER);

	M600_load_filament_movements();

	Sound_MakeCustom(50,1000,false);
}


//! @brief Wait for click
//!
//! Set
void marlin_wait_for_click()
{
    int8_t busy_state_backup = busy_state;
    KEEPALIVE_STATE(PAUSED_FOR_USER);
    lcd_consume_click();
    while(!lcd_clicked())
    {
        delay_keep_alive(0);
    }
    KEEPALIVE_STATE(busy_state_backup);
}

#ifdef PSU_Delta
bool bEnableForce_z;

void init_force_z()
{
WRITE(Z_ENABLE_PIN,Z_ENABLE_ON);
bEnableForce_z=true;                              // "true"-value enforce "disable_force_z()" executing
disable_force_z();
}

void check_force_z()
{
if(!(bEnableForce_z||eeprom_read_byte((uint8_t*)EEPROM_SILENT)))
     init_force_z();                              // causes enforced switching into disable-state
}

void disable_force_z()
{
    if(!bEnableForce_z) return;   // motor already disabled (may be ;-p )

    bEnableForce_z=false;

    // switching to silent mode
#ifdef TMC2130
    tmc2130_mode=TMC2130_MODE_SILENT;
    update_mode_profile();
    tmc2130_init(TMCInitParams(true, FarmOrUserECool()));
#endif // TMC2130
}

void enable_force_z()
{
if(bEnableForce_z)
     return;                                      // motor already enabled (may be ;-p )
bEnableForce_z=true;

// mode recovering
#ifdef TMC2130
tmc2130_mode=eeprom_read_byte((uint8_t*)EEPROM_SILENT)?TMC2130_MODE_SILENT:TMC2130_MODE_NORMAL;
update_mode_profile();
tmc2130_init(TMCInitParams(true, FarmOrUserECool()));
#endif // TMC2130

WRITE(Z_ENABLE_PIN,Z_ENABLE_ON);                  // slightly redundant ;-p
}
#endif // PSU_Delta
