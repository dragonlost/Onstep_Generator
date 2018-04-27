# test.py

import time
import os
import numpy as np

def read_conf_file(path_name):
    file = open(path_name,'r')
    
    text = file.readlines()
    text = np.array(text)
    for i in range(text.size) : text[i]=text[i].strip()
    # Convertion des variables (float32, int32, bool)
    
    ff = [5,6,14,15,29,30,35,40,32,33,37,38,42,43,71,72]
    ii = [3,26,24,39,46,48,56,58,63,66,73,75,76,77,78,79,80]
    bb = [11,20,23,24,25,31,36,41,45,47,50,51,52,53,54,55,57,59,60,61,62,64,65,67,68,69,70,74]
    ss = [0,1,4,7,8,7,9,10,12,13,16,17,18,19,21,22,27,28,44,58,49]
    
    for i in ff:
        text[ff] = np.float32(text[ff])
    for i in ii:
        text[ii] = np.int32(text[ii])
    for i in bb:
        text[bb] = np.bool(text[bb])
    
    
    file.close()
    
    return text


def test_config(path_read, path=os.getcwd()+'/config_'+time.strftime("%d_%Y_%H_%M_%S", time.localtime())+'.txt'):

    
    var = read_conf_file(path_read)
    

    board_type = var[0]
    mount_type = var[1]
    max_rate = var[2]
    pec_spin = var[3]
    auto_sid = var[4]
    
    worm1 = var[5]
    gear1 = var[6]
    stepper1 = var[7]
    micro1 = var[8] 
    slew1 = var[9]
    driver1 = var[10] 
    reverse1 = var[11]
    ena1 = var[12]
    fault1 = var[13] 
    
    worm2 = var[14]
    gear2 = var[15]
    stepper2 = var[16] 
    micro2 = var[17]
    slew2 = var[18]
    driver2 = var[19]
    reverse2 = var[20]
    ena2 = var[21]    
    fault2 = var[22]
    
    rot3 = var[23] 
    foc1 = var[24] 
    foc2 = var[25]
    
    rot_rate = var[26]
    rot_step = var[27]
    rot_micro = var[28]
    rot_gear = var[29]
    rot_gear_2 = var[30] 
    rot_reverse = var[31] 
    rot_min_degr = var[32]
    rot_max_degr = var[33]
    
    foc1_rate = var[34]
    foc1_ratio = var[35] 
    foc1_reverse = var[36] 
    foc1_min_mm = var[37] 
    foc1_max_mm = var[38]
    
    foc2_rate = var[39]
    foc2_ratio = var[40]
    foc2_reverse = var[41] 
    foc2_min_mm = var[42]
    foc2_max_mm = var[43] 
    
    baud = var[44]
    pec = var[45] 
    pec_buffer = var[46] 
    pec_set = var[47]
    analog_pec = var[48] 
    pec_logic = var[49] 
    goto_assist = var[50] 
    strict_park = var[51]
    st4 = var[52]
    alt_st4 = var[53]
    hand = var[54]
    pulse = var[55]
    guide_time = var[56]
    rtc = var[57]
    rtc_time = var[58] 
    pps = var[59]
    limit = var[60] 
    led1 = var[61]
    reticule = var[62] 
    led_intensity = var[63]
    led2 = var[64]
    buzzer = var[65]
    freq_sound = var[66] 
    def_sound = var[67] 
    atmos = var[68]
    home_pause = var[69] 
    max_rate = var[70]
    accel = var[71]
    rapid_stop = var[72] 
    backlash = var[73]
    off_axis = var[74] 
    degre_e = var[75] 
    degre_w = var[76] 
    min_dec = var[77]
    max_dec = var[78]
    pol_limit = var[79] 
    max_az = var[80]
    


    if board_type == "RAMPS 1.4 or 1.5":
        board="Ramps14"
    elif board_type == "MiniPCB (2 axis)":
        board="MiniPCB"
    elif board_type == "MaxPCB (4 axis)":
        board="MaxPCB"
    elif board_type == "STM32F1":
        board="STM32"
    elif board_type == "TivaC":
        board="TM4C"
    elif board_type == "Classic":
        board="Classic"
    elif board_type == "Mega2560 Alternate":
        board="Mega2560Alt"
    elif board_type == "Custom":
        board="Custom"



    var = [board_type, mount_type, max_rate, pec_spin, auto_sid, worm1, 
           gear1, stepper1, micro1, slew1, driver1, reverse1, ena1, fault1,
           worm2, gear2, stepper2, micro2, slew2, driver2, reverse2, ena2,
           fault2, rot3, foc1, foc2, rot_rate, rot_step, rot_micro, 
           rot_gear, rot_gear_2, rot_reverse, rot_min_degr, rot_max_degr,
           foc1_rate, foc1_ratio, foc1_reverse, foc1_min_mm, foc1_max_mm,
           foc2_rate, foc2_ratio, foc2_reverse, foc2_min_mm, foc2_max_mm, 
           baud, pec, pec_buffer, pec_set, analog_pec, pec_logic, 
           goto_assist, strict_park, st4, alt_st4, hand, pulse, guide_time,
           rtc, rtc_time, pps, limit, led1, reticule, led_intensity, led2,
           buzzer, freq_sound, def_sound, atmos, home_pause, max_rate, 
           accel, rapid_stop, backlash, off_axis, degre_e, degre_w, 
           min_dec, max_dec, pol_limit, max_az]

    


    config_file = open("Config"+board+".h","w")

    config_file.write("// -----------------------------------------------------------------------------------")
    config_file.write("// Configuration for OnStep"+board)
    config_file.write("")
    config_file.write("/*")
    config_file.write("* For more information on setting OnStep up see http://www.stellarjourney.com/index.php?r=site/equipment_onstep and")
    config_file.write("* join the OnStep Groups.io at https://groups.io/g/onstep")
    config_file.write("* ")
    config_file.write("* See the Pins."+board+".h file for detailed information on this pin map to be sure it matches your wiring *** USE AT YOUR OWN RISK ***")
    config_file.write("*")
    config_file.write("*/")
    config_file.write("")
    config_file.write("#define "+board+"_ON")
    config_file.write("")
    config_file.write("#ifdef "+board+"_ON")
    config_file.write("// -------------------------------------------------------------------------------------------------------------------------")
    config_file.write("// ADJUST THE FOLLOWING TO CONFIGURE YOUR CONTROLLER FEATURES --------------------------------------------------------------")
    config_file.write("")
    config_file.write("// Enables internal goto assist mount modeling (for Eq mounts), default=_OFF (Experimental)")
    config_file.write("// Note that Goto Assist in Sky Planetarium works even if this is off")
    config_file.write("#define ALIGN_GOTOASSIST_OFF")
    config_file.write("")
    config_file.write("// Default speed for Serial1 and Serial4 com ports, Default=9600")
    config_file.write("#define SERIAL1_BAUD_DEFAULT 9600")
    config_file.write("#define SERIAL4_BAUD_DEFAULT 9600")
    config_file.write("")
    config_file.write("// ESP8266 reset and GPIO0 control, this sets run mode for normal operation.  Uploading programmer firmware to the OpStep MCU can then enable sending new firmware to the ESP8266-01")
    config_file.write("// Pin 18 (Aux1) for GPIO0 and Pin 5 (Aux2) for Rst control.  Choose only one feature on Aux1/2.")
    config_file.write("#define ESP8266_CONTROL_OFF")
    config_file.write("")
    config_file.write("// Mount type, default is _GEM (German Equatorial) other options are _FORK, _FORK_ALT.  _FORK switches off Meridian Flips after (1, 2 or 3 star) alignment is done.  _FORK_ALT disables Meridian Flips (1 star align.)")
    config_file.write("// _ALTAZM is for Alt/Azm mounted 'scopes (1 star align only.)")
    config_file.write("#define MOUNT_TYPE_GEM")
    config_file.write("")
    config_file.write("// Strict parking, default=_OFF.  Set to _ON and unparking is only allowed if successfully parked.  Otherwise unparking is allowed if at home and not parked (the Home/Reset command \":hF#\" sets this state.) ")
    config_file.write("#define STRICT_PARKING_OFF")
    config_file.write("")
    config_file.write("// ST4 interface on pins 24, 25, 26, 27.  Pin 24 is RA- (West), Pin 25 is Dec- (South), Pin 26 is Dec+ (North), Pin 27 is RA+ (East.)")
    config_file.write("// ST4_ON enables the interface.  ST4_PULLUP enables the interface and any internal pullup resistors.")
    config_file.write("// It is up to you to create an interface that meets the electrical specifications of any connected device, use at your own risk.  default=_OFF")
    config_file.write("#define ST4_OFF")
    config_file.write("// If SEPARATE_PULSE_GUIDE_RATE_ON is used the ST4 port is limited to guide rates <= 1X except when ST4_HAND_CONTROL_ON is used.")
    config_file.write("// Additionally, ST4_HAND_CONTROL_ON enables special features: Press and hold [E]+[W] buttons for > 2 seconds...  In this mode [E] decreases and [W] increases guide rates (or if tracking isn't on yet adjusts illuminated recticule brightness.)")
    config_file.write("// [S] for Sync (or Accept if in align mode.) [N] for Tracking on/off. -OR- Press and hold [N]+[S] buttons for > 2 seconds...  In this mode [E] selects prior and [W] next user catalog item.")
    config_file.write("// [N] to do a Goto to the catalog item.  [S] for Sound on/off.  The keypad returns to normal operation after 4 seconds of inactivity.  ST4_HAND_CONTROL_ON also adds a 100ms de-bounce to all button presses.")
    config_file.write("// Finally, during a goto pressing any button aborts the slew.  If meridian flip paused at home, pressing any button continues.  default=_ON")
    config_file.write("#define ST4_HAND_CONTROL_ON")
    config_file.write("")
    config_file.write("// Separate pulse-guide rate so centering and guiding don't disturb each other, default=_ON")
    config_file.write("#define SEPARATE_PULSE_GUIDE_RATE_ON")
    config_file.write("")
    config_file.write("// Guide time limit (in seconds,) default=0 (no limit.)  A safety feature, some guides are started with one command and stopped with another.")
    config_file.write("// If the stop command is never received the guide will continue forever unless this is enabled.")
    config_file.write("#define GUIDE_TIME_LIMIT 0")
    config_file.write("")
    config_file.write("// RTC (Real Time Clock) support, default=_OFF.")
    config_file.write("// Other options: RTC_DS3234 for a DS3234 on the default SPI interface pins (CS on pin 10) or RTC_DS3231 for a DS3231 on the default I2C pins (optionally wire the SQW output to the PPS pin below.)")
    config_file.write("#define RTC_OFF")
    config_file.write("// PPS use _ON or _PULLUP to enable the input and use the built-in pullup resistor.  Sense rising edge on Pin 28 for optional precision clock source (GPS, for example), default=_OFF")
    config_file.write("#define PPS_SENSE_OFF")
    config_file.write("// Note: The MaxPCB has a DS3234 connector")
    config_file.write("")
    config_file.write("// PEC sense on Pin 23 (A9) use _ON or _PULLUP to enable the input/use the built-in pullup resistor (digital input) or provide a comparison value (see below) for analog operation, default=_OFF")
    config_file.write("// Analog values range from 0 to 1023 which indicate voltages from 0-3.3VDC on the analog pin, for example \"PEC_SENSE 600\" would detect an index when the voltage exceeds 1.93V")
    config_file.write("// With either index detection method, once triggered 60s must expire before another detection can happen.  This gives time for the index magnet to pass by the detector before another cycle begins.")
    config_file.write("// Ignored on Alt/Azm mounts.")
    config_file.write("#define PEC_SENSE_OFF")
    config_file.write("// PEC sense, rising edge (default with PEC_SENSE_STATE HIGH, use LOW for falling edge, ex. PEC_SENSE_ON) ; for optional PEC index")
    config_file.write("#define PEC_SENSE_STATE HIGH")
    config_file.write("")
    config_file.write("// Switch close (to ground) on Pin 4 for optional limit sense (stops gotos and/or tracking), default=_OFF")
    config_file.write("#define LIMIT_SENSE_OFF")
    config_file.write("")
    config_file.write("// Light status LED by sink to ground (Pin 19), default=_ON.")
    config_file.write("// _ON and OnStep keeps this illuminated to indicate that the controller is active.  When sidereal tracking this LED will rapidly flash.")
    config_file.write("#define STATUS_LED_PINS_ON")
    config_file.write("// Light 2nd status LED by sink to ground (Pin 22), default=_OFF.")
    config_file.write("// _ON sets this to blink at 1 sec intervals when PPS is synced.  Turns off if tracking is stopped.  Turns on during gotos.")
    config_file.write("#define STATUS_LED2_PINS_OFF")
    config_file.write("// Light reticule LED by sink to ground (Pin 22), default=_OFF.  (don't use with STATUS_LED2_PINS_ON)")
    config_file.write("// RETICULE_LED_PINS n, where n=0 to 255 activates this feature and sets default brightness")
    config_file.write("#define RETICULE_LED_PINS_OFF")
    config_file.write("")
    config_file.write("// Sound/buzzer on Pin 29, default=_OFF.")
    config_file.write("// Specify frequency for a piezo speaker (for example \"BUZZER 2000\") or use BUZZER_ON for a piezo buzzer.")
    config_file.write("#define BUZZER_OFF")
    config_file.write("// Sound state at startup, default=_ON.")
    config_file.write("#define DEFAULT_SOUND_ON")
    config_file.write("")
    config_file.write("// Optionally adjust tracking rate to compensate for atmospheric refraction, default=_OFF")
    config_file.write("// can be turned on/off with the :Tr# and :Tn# commands regardless of this setting")
    config_file.write("#define TRACK_REFRACTION_RATE_DEFAULT_OFF")
    config_file.write("")
    config_file.write("// Set to _ON and OnStep will remember the last auto meridian flip setting (on/off), default=_OFF")
    config_file.write("#define REMEMBER_AUTO_MERIDIAN_FLIP_OFF")
    config_file.write("")
    config_file.write("// Set to _ON and OnStep will remember the last meridian flip pause at home setting (on/off), default=_OFF")
    config_file.write("#define REMEMBER_PAUSE_HOME_OFF")
    config_file.write("")
    config_file.write("// ADJUST THE FOLLOWING TO MATCH YOUR MOUNT --------------------------------------------------------------------------------")
    config_file.write(" #define REMEMBER_MAX_RATE_OFF        // set to _ON and OnStep will remember rates set in the ASCOM driver, Android App, etc. default=_OFF ")
    config_file.write(" #define MaxRate                   96 // microseconds per microstep default setting for gotos, can be adjusted for two times lower or higher at run-time")
    config_file.write("                                     // minimum* (fastest goto) is around 12 (Teensy3.5,) 4 (Teensy3.6,) default=96 higher is ok")
    config_file.write("                                     // * = minimum can be lower, when both AXIS1/AXIS2_MICROSTEPS are used the compiler will warn you if it's too low")
    config_file.write("")
    config_file.write("#define DegreesForAcceleration   5.0 // approximate number of degrees for full acceleration or deceleration: higher values=longer acceleration/deceleration")
    config_file.write("                                     // Default=5.0, too low (about <1) can cause gotos to never end if micro-step mode switching is enabled.")
    config_file.write("#define DegreesForRapidStop      1.0 // approximate number of degrees required to stop when requested or if limit is exceeded during a slew: higher values=longer deceleration")
    config_file.write("                                     // Default=1.0, too low (about <1) can cause gotos to never end if micro-step mode switching is enabled.")
    config_file.write("")
    config_file.write("#define BacklashTakeupRate        25 // backlash takeup rate (in multipules of the sidereal rate): too fast and your motors will stall,")
    config_file.write("                                     // too slow and the mount will be sluggish while it moves through the backlash")
    config_file.write("                                     // for the most part this doesn't need to be changed, but adjust when needed.  Default=25")
    config_file.write("")
    config_file.write("                                     // Axis1 is for RA/Az")
    config_file.write("#define StepsPerDegreeAxis1  12800.0 // calculated as    :  stepper_steps * micro_steps * gear_reduction1 * (gear_reduction2/360)")
    config_file.write("                                     // G11              :  400           * 32          * 1               *  360/360              = 12800")
    config_file.write("                                     // Axis2 is for Dec/Alt")
    config_file.write("#define StepsPerDegreeAxis2  12800.0 // calculated as    :  stepper_steps * micro_steps * gear_reduction1 * (gear_reduction2/360)")
    config_file.write("                                     // G11              :  400           * 32          * 1               *  360/360              = 12800")
    config_file.write("")
    config_file.write("                                     // PEC, number of steps for a complete worm rotation (in RA), (StepsPerDegreeAxis1*360)/gear_reduction2.  Ignored on Alt/Azm mounts.")
    config_file.write("#define StepsPerWormRotationAxis1 12800L")
    config_file.write("                                     // G11              : (12800*360)/360 = 12800")
    config_file.write("")
    config_file.write("#define PECBufferSize           824  // PEC, buffer size, max should be no more than 3384, your required buffer size >= StepsPerAxis1WormRotation/(StepsPerDegeeAxis1/240)")
    config_file.write("                                     // for the most part this doesn't need to be changed, but adjust when needed.  824 seconds is the default.  Ignored on Alt/Azm mounts.")
    config_file.write("")
    config_file.write("#define MinutesPastMeridianE      30 // for goto's, how far past the meridian to allow before we do a flip (if on the East side of the pier) - a half hour of RA is the default = 30.  Sometimes used for Fork mounts in Align mode.  Ignored on Alt/Azm mounts.")
    config_file.write("#define MinutesPastMeridianW      30 // as above, if on the West side of the pier.  If left alone, the mount will stop tracking when it hits the this limit.  Sometimes used for Fork mounts in Align mode.  Ignored on Alt/Azm mounts.")
    config_file.write("                                     // The above two lines can be removed and settings in EEPROM will be used instead, be sure to set the Meridian limits in control software if you do this!")
    config_file.write("                                     // If you don't remove these lines Meridian limits will return to these defaults on power up.")
    config_file.write("#define UnderPoleLimit            12 // maximum allowed hour angle (+/-) under the celestial pole.  Default=12.  Ignored on Alt/Azm mounts.")
    config_file.write("                                     // If left alone, the mount will stop tracking when it hits this limit.  Valid range is 10 to 12 hours.")
    config_file.write("#define MinDec                   -91 // minimum allowed declination, default = -91 (off)  Ignored on Alt/Azm mounts.")
    config_file.write("#define MaxDec                   +91 // maximum allowed declination, default =  91 (off)  Ignored on Alt/Azm mounts.")
    config_file.write("                                     // For example, a value of +80 would stop gotos/tracking near the north celestial pole.")
    config_file.write("                                     // For a Northern Hemisphere user, this would stop tracking when the mount is in the polar home position but")
    config_file.write("                                     // that can be easily worked around by doing an alignment once and saving a park position (assuming a ")
    config_file.write("                                     // fork/yolk mount with meridian flips turned off by setting the minutesPastMeridian values to cover the whole sky)")
    config_file.write("#define MaxAzm                   180 // Alt/Az mounts only. +/- maximum allowed Azimuth, default =  180.  Allowed range is 180 to 360")
    config_file.write("")
    config_file.write("// AXIS1/2 STEPPER DRIVER CONTROL ------------------------------------------------------------------------------------------")
    config_file.write("// Axis1: Pins 20,21 = Step,Dir (RA/Azm)")
    config_file.write("// Axis2: Pins  3, 2 = Step,Dir (Dec/Alt)")
    config_file.write("")
    config_file.write("// Reverse the direction of movement.  Adjust as needed or reverse your wiring so things move in the right direction")
    config_file.write("#define AXIS1_REVERSE_OFF            // RA/Azm axis")
    config_file.write("#define AXIS2_REVERSE_OFF            // Dec/Alt axis")
    config_file.write("")
    config_file.write("// Stepper driver Enable support, just wire Enable to Pins 14 (Axis1) and 9 (Axis2) and OnStep will pull these HIGH to disable the stepper drivers on startup and when Parked or Homed.")
    config_file.write("// An Align, Sync, or Un-Park will enable the drivers.  Adjust below if you need these pulled LOW to disable the drivers.")
    config_file.write("#define AXIS1_DISABLE HIGH")
    config_file.write("#define AXIS2_DISABLE HIGH")
    config_file.write("")
    config_file.write("// For equatorial mounts, _ON powers down the Declination axis when it's not being used to help lower power use.  During low rate guiding (<=1x) the axis stays enabled")
    config_file.write("// for 10 minutes after any guide on either axis.  Otherwise, the Dec axis is disabled (powered off) 10 seconds after movement stops.")
    config_file.write("#define AXIS2_AUTO_POWER_DOWN_OFF")
    config_file.write("")
    config_file.write("// Basic stepper driver mode setup . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .")
    config_file.write("// If used, this requires connections M0, M1, and M2 on Pins 15,16,17 for Axis1 (RA/Azm) and Pins 8,7,6 for Axis2 (Dec/Alt.)")
    config_file.write("// Stepper driver models are as follows: (for example AXIS1_DRIVER_MODEL DRV8825,) A4988, LV8729, RAPS128, TMC2208, TMC2130 (spreadCycle,) ")
    config_file.write("// TMC2130_QUIET (stealthChop tracking,) TMC2130_VQUIET (full stealthChop mode,) add _LOWPWR for 50% power during tracking (for example: TMC2130_QUIET_LOWPWR)")
    config_file.write("#define AXIS1_DRIVER_MODEL_OFF      // Axis1 (RA/Azm):  Default _OFF, Stepper driver model (see above)")
    config_file.write("#define AXIS1_MICROSTEPS_OFF        // Axis1 (RA/Azm):  Default _OFF, Microstep mode when the scope is doing sidereal tracking (for example: AXIS1_MICROSTEPS 32)")
    config_file.write("#define AXIS1_MICROSTEPS_GOTO_OFF   // Axis1 (RA/Azm):  Default _OFF, Optional microstep mode used during gotos (for example: AXIS1_MICROSTEPS_GOTO 2)")
    config_file.write("#define AXIS2_DRIVER_MODEL_OFF      // Axis2 (Dec/Alt): Default _OFF, Stepper driver model (see above)")
    config_file.write("#define AXIS2_MICROSTEPS_OFF        // Axis2 (Dec/Alt): Default _OFF, Microstep mode when the scope is doing sidereal tracking")
    config_file.write("#define AXIS2_MICROSTEPS_GOTO_OFF   // Axis2 (Dec/Alt): Default _OFF, Optional microstep mode used during gotos")
    config_file.write("// Note: you can replace this section with the contents of \"AdvancedStepperSetup.txt\" . . . . . . . . . . . . . . . . . . . ")
    config_file.write("")
    config_file.write("// Stepper driver Fault detection on Pins 18 (Aux1) and 5 (Aux2,) choose only one feature to use on Aux1/2.  The SPI interface (on M0/M1/M2/Aux) can be used to detect errors on the TMC2130.")
    config_file.write("// other settings are LOW, HIGH, TMC2130 (if available applies internal pullup if LOW and pulldown if HIGH.)")
    config_file.write("#define AXIS1_FAULT_OFF")
    config_file.write("#define AXIS2_FAULT_OFF")
    config_file.write("")
    config_file.write("// ------------------------------------------------------------------------------------------------------------------------")
    config_file.write("// FOCUSER ROTATOR OR ALT/AZ DE-ROTATION ----------------------------------------------------------------------------------")
    config_file.write("// Pins 30,33 = Step,Dir (choose either this option or the second focuser, not both)")
    config_file.write("#define ROTATOR_OFF                  // enable or disable rotator feature (for any mount type,) default=_OFF (de-rotator is available only for MOUNT_TYPE_ALTAZM.)")
    config_file.write("#define MaxRateAxis3               8 // this is the minimum number of milli-seconds between micro-steps, default=8")
    config_file.write("#define StepsPerDegreeAxis3     64.0 // calculated as    :  stepper_steps * micro_steps * gear_reduction1 * (gear_reduction2/360)")
    config_file.write("                                     // Rotator          :  24            * 8           * 20              *  6/360                = 64")
    config_file.write("                                     // For de-rotation of Alt/Az mounts a quick estimate of the required resolution (in StepsPerDegree)")
    config_file.write("                                     // would be an estimate of the circumference of the useful imaging circle in (pixels * 2)/360")
    config_file.write("#define MinAxis3                -180 // minimum allowed Axis3 rotator, default = -180")
    config_file.write("#define MaxAxis3                 180 // maximum allowed Axis3 rotator, default =  180")
    config_file.write("#define AXIS3_REVERSE_OFF            // reverse the direction of Axis3 rotator movement")
    config_file.write("#define AXIS3_DISABLE_OFF            // Pin 36 (Aux3.)  Use HIGH for common stepper drivers if you want to power down the motor at stand-still.  Default _OFF.")
    config_file.write("")
    config_file.write("// FOCUSER1 ---------------------------------------------------------------------------------------------------------------")
    config_file.write("// Pins 34,35 = Step,Dir")
    config_file.write("#define FOCUSER1_OFF                 // enable or disable focuser feature, default=_OFF")
    config_file.write("#define MaxRateAxis4               8 // this is the minimum number of milli-seconds between micro-steps, default=8")
    config_file.write("#define StepsPerMicrometerAxis4  0.5 // figure this out by testing or other means")
    config_file.write("#define MinAxis4               -25.0 // minimum allowed Axis4 position in millimeters, default = -25.0")
    config_file.write("#define MaxAxis4                25.0 // maximum allowed Axis4 position in millimeters, default =  25.0")
    config_file.write("#define AXIS4_REVERSE_OFF            // reverse the direction of Axis4 focuser movement")
    config_file.write("#define AXIS4_DISABLE_OFF            // Pin 39 (Aux4.)  Use HIGH for common stepper drivers if you want to power down the motor at stand-still.  Default _OFF.")
    config_file.write("")
    
    if board != "TM4C":
        config_file.write("// FOCUSER2 ---------------------------------------------------------------------------------------------------------------")
        config_file.write("// Pins 30,33 = Step,Dir (choose either this option or the rotator, not both) ")
        config_file.write("#define FOCUSER2_OFF                 // enable or disable focuser feature, default=_OFF")
        config_file.write("#define MaxRateAxis5               8 // this is the minimum number of milli-seconds between micro-steps, default=8")
        config_file.write("#define StepsPerMicrometerAxis5  0.5 // figure this out by testing or other means")
        config_file.write("#define MinAxis5               -25.0 // minimum allowed Axis5 position in millimeters, default = -25.0")
        config_file.write("#define MaxAxis5                25.0 // maximum allowed Axis5 position in millimeters, default =  25.0")
        config_file.write("#define AXIS5_REVERSE_OFF            // reverse the direction of Axis5 focuser movement")
        config_file.write("#define AXIS5_DISABLE_OFF            // Pin 36 (Aux3.)  Use HIGH for common stepper drivers if you want to power down the motor at stand-still.  Default _OFF.")
        config_file.write("")
        
    config_file.write("// THAT'S IT FOR USER CONFIGURATION!")
    config_file.write("")
    config_file.write("// -------------------------------------------------------------------------------------------------------------------------")
    config_file.write("#define FileVersionConfig 2")
    config_file.write("#include \"Pins."+board+".h\"")
    config_file.write("#endif")
                      
    config_file.close()