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
    
    ff = [4,5,13,14,28,29,31,32,37,38,43,44,80,81]
    ii = [2,6,15,25,26,34,35,40,41,51,59,64,69,71,74,82,84,85,86,87,88,89]
    bol = [3,10,19,22,23,24,30,36,42,48,49,50,52,53,54,55,56,57,58,61,62,63,66,67,68,70,72,75,76,77,78,79,83]
    ss = [0,1,7,8,9,11,12,16,17,18,20,21,27,33,39,45,46,47,60,65,73]
    
    for i in ff:
        text[ff] = np.float32(text[ff])
    for i in ii:
        text[ii] = np.int32(text[ii])
    for i in bol:
        text[bol] = np.bool(text[bol])
    
    
    file.close()
    
    return text


def test_config(path_read, path=os.getcwd()+'/config_'+time.strftime("%d_%Y_%H_%M_%S", time.localtime())+'.txt'):

    
    var = read_conf_file(path_read)
    

    board_type = var[0] #string
    mount_type = var[1] #string
    max_rate = var[2] #int
    auto_sid = var[3] #bool
    
    worm1 = var[4] #float
    gear1 = var[5] #float
    stepper1 = var[6] #int
    micro1 = var[7] #string
    slew1 = var[8] #string
    driver1 = var[9] #string
    reverse1 = var[10] #bool
    ena1 = var[11] #string
    fault1 = var[12] #string
    
    worm2 = var[13] #float
    gear2 = var[14] #float
    stepper2 = var[15] #int
    micro2 = var[16] #string
    slew2 = var[17] #string
    driver2 = var[18] #string
    reverse2 = var[19] #bool
    ena2 = var[20] #string
    fault2 = var[21] #string
    
    rot3 = var[22] #bool
    foc1 = var[23] #bool
    foc2 = var[24] #bool
    
    rot_rate = var[25] #int
    rot_step = var[26] #int
    rot_micro = var[27] #string
    rot_gear = var[28] #float
    rot_gear_2 = var[29] #float
    rot_reverse = var[30] #bool
    rot_min_degr = var[31] #float
    rot_max_degr = var[32] #float
    rot_disable = var[33] #string
    
    foc1_rate = var[34] #int
    foc1_ratio = var[35] #int
    foc1_reverse = var[36] #bool
    foc1_min_mm = var[37] #float
    foc1_max_mm = var[38] #float
    focus1_disable = var[39] #string
    
    foc2_rate = var[40] #int
    foc2_ratio = var[41] #int
    foc2_reverse = var[42] #bool
    foc2_min_mm = var[43] #float
    foc2_max_mm = var[44] #float
    focus2_disable = var[45] #string
    
    baud = var[46] #string
    baud4 = var[47] #string
    esp = var[48] #bool
    pec = var[49] #bool
    pec_pul = var[50] #bool
    pec_buffer = var[51] #int
    goto_assist = var[52] #bool
    strict_park = var[53] #bool
    st4 = var[54] #bool
    st4_pul = var[55] #bool
    alt_st4 = var[56] #bool
    hand = var[57] #bool
    pulse = var[58] #bool
    guide_time = var[59] #int
    rtc = var[60] #string
    pps = var[61] #bool
    pps_pul = var[62] #bool
    pec_set = var[63] #bool
    analog_pec = var[64] #int
    pec_logic = var[65] #string
    limit = var[66] #bool
    led1 = var[67] #bool
    led2 = var[68] #bool
    led2_intensity = var[69] #int
    reticule = var[70] #bool
    ret_intensity = var[71] #int
    buzzer = var[72] #bool
    buzzer_type = var[73] #string
    freq_sound = var[74] #int
    def_sound = var[75] #bool
    atmos = var[76] #bool
    mem_flip_mer = var[77] #bool 
    home_pause = var[78] #bool
    mem_max_rate = var[79] #bool
    accel = var[80] #float
    rapid_stop = var[81] #float
    backlash = var[82] #int
    off_axis2 = var[83] #bool
    degre_e = var[84] #int
    degre_w = var[85] #int
    min_dec = var[86] #int
    max_dec = var[87] #int
    pol_limit = var[88] #int
    max_az = var[89] #int
    
    
    ####__________________Axis1_____________________________
    # Axis1 reverse
    if reverse1:
        rev1="ON"
    else:
        rev1="OFF"
    worm1 = var[5]
    gear1 = var[6]
    stepper1 = var[7]
    micro1 = var[8] 
    step_degre_axis1 = stepper1 * np.int32(micro1) * worm1 * (gear1/360.)
    
    # Driver Type Axis 1
    if driver1 == "A4988":
        driver1_mod="A4988"
    elif driver1 == "DRV8825":
        driver1_mod="DRV8825"
    elif driver1 == "LV8729 or RAPS128":
        driver1_mod="LV8729"
    elif driver1 == "TMC2208":
        driver1_mod="TMC2208"
    elif driver1 == "TMC2130":
        driver1_mod="TMC2130"
    elif driver1 == "TMC2130 (Quiet)":
        driver1_mod="TMC2130_QUIET"
    elif driver1 == "TMC2130 (VQuiet)":
        driver1_mod="TMC2130_VQUIET"
    elif driver1 == "TMC2130 (Quiet, LowPWR)":
        driver1_mod="TMC2130_QUIET_LOWPWR"
    elif driver1 == "TMC2130 (VQuiet, LowPWR)":
        driver1_mod="TMC2130_VQUIET_LOWPWR"
        
    ####__________________Axis2_____________________________
    # Axis2 reverse
    if reverse2:
        rev2="ON"
    else:
        rev2="OFF"
    step_degre_axis2 = stepper2 * np.int32(micro2) * worm2 * (gear2/360.) 
    
    # Driver Type Axis 2
    if driver2 == "A4988":
        driver2_mod="A4988"
    elif driver2 == "DRV8825":
        driver2_mod="DRV8825"
    elif driver2 == "LV8729 or RAPS128":
        driver2_mod="LV8729"
    elif driver2 == "TMC2208":
        driver2_mod="TMC2208"
    elif driver2 == "TMC2130":
        driver2_mod="TMC2130"
    elif driver2 == "TMC2130 (Quiet)":
        driver2_mod="TMC2130_QUIET"
    elif driver2 == "TMC2130 (VQuiet)":
        driver2_mod="TMC2130_VQUIET"
    elif driver2 == "TMC2130 (Quiet, LowPWR)":
        driver2_mod="TMC2130_QUIET_LOWPWR"
    elif driver2 == "TMC2130 (VQuiet, LowPWR)":
        driver2_mod="TMC2130_VQUIET_LOWPWR"
    
    ####__________________Focuser1__________________________
    # Focus 1 reverse
    if foc1_reverse:
        foc1_rev="ON"
    else:
        foc1_rev="OFF"
    
    # Focus 1 ON/OFF
    if foc1:
        focus1="ON"
    else:
        focus1="OFF"   
        
    ####__________________Focuser2__________________________
    # Focus 2 reverse
    if foc2_reverse:
        foc2_rev="ON"
    else:
        foc2_rev="OFF"  
      
    # Focus 2 ON/OFF
    if foc2:
        focus2="ON"
    else:
        focus2="OFF"
        
    ####__________________Rotator___________________________
    # Rotator reverse
    if rot_reverse:
        rot_rev="ON"
    else:
        rot_rev="OFF"
    
    # Rotator ON/OFF
    if rot3:
        rotator="ON"
    else:
        rotator="OFF"

    step_degre_rot = rot_step  * rot_micro * rot_gear * (rot_gear_2/360)
    
    ####__________________OPTION_____________________________
    
    # Board Type
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
    
    # Mount Type
    if mount_type == "Equatorial":
        mount="GEM"
    elif mount_type == "Fork":
        mount="FORK"
    elif mount_type == "Alt-Azimuth":
        mount="FORK_ALT"
    
    # Goto Assist 
    if goto_assist:
        assist="ON"
    else:
        assist="OFF"
        
    # ESP8266 Control
    if esp:
        esp_s="ON"
    else:
        esp_s="OFF"
        
    # Strict Parking
    if strict_park:
        s_park="ON"
    else:
        s_park="OFF"
        
        
    # alternative st4
    if alt_st4:
        a_st4="ON"
    else:
        a_st4="OFF"    

    # hand controler st4
    if hand:
        h_st4="ON"
    else:
        h_st4="OFF"

    # Separate Pulse guide rate
    if pulse:
        pulse_g="ON"
    else:
        pulse_g="OFF"  
        
    # Limit Sensor
    if limit:
        s_limit="ON"
    else:
        s_limit="OFF" 
        
    # Default sound at Startup
    if def_sound:
        sound="ON"
    else:
        sound="OFF"
        
    # Refraction tracking
    if atmos:
        s_atm="ON"
    else:
        s_atm="OFF"
        
    # Home Pause
    if home_pause:
        p_home="ON"
    else:
        p_home="OFF"
        
    # Remember Max Rate 
    if mem_max_rate:
        mem_rate="ON"
    else:
        mem_rate="OFF"
    
    # Remember Flip meridian position    
    if mem_flip_mer:
        mem_flip="ON"
    else:
        mem_flip="OFF"
        
    # Led 1 status    
    if led1:
        s_led1="ON"
    else:
        s_led1="OFF"
        
        
    # Off Axis 2 at time limit    
    if off_axis2:
        off2="ON"
    else:
        off2="OFF"
        
    # Set auto sideral tracking at start
    if auto_sid:
        a_sid="ON"
    else:
        a_sid="OFF"
        

    """var = [board_type, mount_type, max_rate, pec_pul, auto_sid, worm1, 
           gear1, stepper1, micro1, slew1, driver1, reverse1, ena1, fault1,
           worm2, gear2, stepper2, micro2, slew2, driver2, reverse2, ena2,
           fault2, rot3, foc1, foc2, rot_rate, rot_step, rot_micro, 
           rot_gear, rot_gear_2, rot_reverse, rot_min_degr, rot_max_degr,
           foc1_rate, foc1_ratio, foc1_reverse, foc1_min_mm, foc1_max_mm,
           foc2_rate, foc2_ratio, foc2_reverse, foc2_min_mm, foc2_max_mm, 
           baud, pec, pec_buffer, pec_set, analog_pec, pec_logic, 
           goto_assist, strict_park, st4, alt_st4, hand, pulse, guide_time,
           rtc, mem_flip_mer, pps, limit, led1, reticule, led_intensity, led2,
           buzzer, freq_sound, def_sound, atmos, home_pause, mem_max_rate, 
           accel, rapid_stop, backlash, off_axis, degre_e, degre_w, 
           min_dec, max_dec, pol_limit, max_az, buzzer_type,esp_8266,rot_disable, 
           focus1_disable, focus2_disable]
    """
    


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
    config_file.write("#define ALIGN_GOTOASSIST_"+assist)
    config_file.write("")
    config_file.write("// Default speed for Serial1 and Serial4 com ports, Default=9600")
    config_file.write("#define SERIAL1_BAUD_DEFAULT "+np.str(baud))
                      
    if board =="MaxPCB":
        config_file.write("#define SERIAL4_BAUD_DEFAULT "+np.str(baud4))
                          
    config_file.write("")
    config_file.write("// ESP8266 reset and GPIO0 control, this sets run mode for normal operation.  Uploading programmer firmware to the OpStep MCU can then enable sending new firmware to the ESP8266-01")
    config_file.write("// Pin 18 (Aux1) for GPIO0 and Pin 5 (Aux2) for Rst control.  Choose only one feature on Aux1/2.")
    config_file.write("#define ESP8266_CONTROL_"+esp_s)
    config_file.write("")
    config_file.write("// Mount type, default is _GEM (German Equatorial) other options are _FORK, _FORK_ALT.  _FORK switches off Meridian Flips after (1, 2 or 3 star) alignment is done.  _FORK_ALT disables Meridian Flips (1 star align.)")
    config_file.write("// _ALTAZM is for Alt/Azm mounted 'scopes (1 star align only.)")
    config_file.write("#define MOUNT_TYPE_"+mount)
    config_file.write("")
    config_file.write("// Strict parking, default=_OFF.  Set to _ON and unparking is only allowed if successfully parked.  Otherwise unparking is allowed if at home and not parked (the Home/Reset command \":hF#\" sets this state.) ")
    config_file.write("#define STRICT_PARKING_"+s_park)
    config_file.write("")
    config_file.write("// ST4 interface on pins 24, 25, 26, 27.  Pin 24 is RA- (West), Pin 25 is Dec- (South), Pin 26 is Dec+ (North), Pin 27 is RA+ (East.)")
    config_file.write("// ST4_ON enables the interface.  ST4_PULLUP enables the interface and any internal pullup resistors.")
    config_file.write("// It is up to you to create an interface that meets the electrical specifications of any connected device, use at your own risk.  default=_OFF")
    
    
    if st4:
        if st4_pul:
            config_file.write("#define ST4_PULLUP")
        else:
            config_file.write("#define ST4_ON")
    else:
        config_file.write("#define ST4_OFF")
    
                      
    config_file.write("// If SEPARATE_PULSE_GUIDE_RATE_ON is used the ST4 port is limited to guide rates <= 1X except when ST4_HAND_CONTROL_ON is used.")
    config_file.write("// Additionally, ST4_HAND_CONTROL_ON enables special features: Press and hold [E]+[W] buttons for > 2 seconds...  In this mode [E] decreases and [W] increases guide rates (or if tracking isn't on yet adjusts illuminated recticule brightness.)")
    config_file.write("// [S] for Sync (or Accept if in align mode.) [N] for Tracking on/off. -OR- Press and hold [N]+[S] buttons for > 2 seconds...  In this mode [E] selects prior and [W] next user catalog item.")
    config_file.write("// [N] to do a Goto to the catalog item.  [S] for Sound on/off.  The keypad returns to normal operation after 4 seconds of inactivity.  ST4_HAND_CONTROL_ON also adds a 100ms de-bounce to all button presses.")
    config_file.write("// Finally, during a goto pressing any button aborts the slew.  If meridian flip paused at home, pressing any button continues.  default=_ON")
    config_file.write("#define ST4_HAND_CONTROL_"+h_st4)
                      
    if board == "Classic":
        config_file.write("// ST4_ALTERNATE_PINS_ON moves the interface (Mega2560 only) to pins 43, 45, 47, 49.  Pin 43 is Dec- (South), Pin 45 is Dec+ (North), Pin 47 is RA- (West), Pin 49 is RA+ (East.)")
        config_file.write("// ST4_ALTERNATE_PINS_ON is required for Steve's ST4 board and is also required if the ST4 interface is to be used alongside the Arduino Ethernet Shield")
        config_file.write("#define ST4_ALTERNATE_PINS_"+a_st4)
                          
    config_file.write("")
    config_file.write("// Separate pulse-guide rate so centering and guiding don't disturb each other, default=_ON")
    config_file.write("#define SEPARATE_PULSE_GUIDE_RATE_"+pulse_g)
    config_file.write("")
    config_file.write("// Guide time limit (in seconds,) default=0 (no limit.)  A safety feature, some guides are started with one command and stopped with another.")
    config_file.write("// If the stop command is never received the guide will continue forever unless this is enabled.")
    config_file.write("#define GUIDE_TIME_LIMIT "+np.str(guide_time))
    config_file.write("")
    config_file.write("// RTC (Real Time Clock) support, default=_OFF.")
    config_file.write("// Other options: RTC_DS3234 for a DS3234 on the default SPI interface pins (CS on pin 10) or RTC_DS3231 for a DS3231 on the default I2C pins (optionally wire the SQW output to the PPS pin below.)")
    config_file.write("#define RTC_"+rtc)
    config_file.write("// PPS use _ON or _PULLUP to enable the input and use the built-in pullup resistor.  Sense rising edge on Pin 28 for optional precision clock source (GPS, for example), default=_OFF")
    
    if pps:
        if pps_pul:
            config_file.write("#define PPS_SENSE_PULLUP")
        else:
            config_file.write("#define PPS_SENSE_ON")
    else:
        config_file.write("#define PPS_SENSE_OFF")
    
                      
    config_file.write("// Note: The MaxPCB has a DS3234 connector")
    config_file.write("")
    config_file.write("// PEC sense on Pin 23 (A9) use _ON or _PULLUP to enable the input/use the built-in pullup resistor (digital input) or provide a comparison value (see below) for analog operation, default=_OFF")
    config_file.write("// Analog values range from 0 to 1023 which indicate voltages from 0-3.3VDC on the analog pin, for example \"PEC_SENSE 600\" would detect an index when the voltage exceeds 1.93V")
    config_file.write("// With either index detection method, once triggered 60s must expire before another detection can happen.  This gives time for the index magnet to pass by the detector before another cycle begins.")
    config_file.write("// Ignored on Alt/Azm mounts.")
    if pec:
        if (pec_set and analog_pec!=0):
            config_file.write("#define PEC_SENSE "+np.str(analog_pec))
        else:
            if pec_pul:
                config_file.write("#define PEC_SENSE_PULLUP")
            else:
                config_file.write("#define PEC_SENSE_ON")
    else:
        config_file.write("#define PEC_SENSE_OFF")
                          
    config_file.write("// PEC sense, rising edge (default with PEC_SENSE_STATE HIGH, use LOW for falling edge, ex. PEC_SENSE_ON) ; for optional PEC index")
    config_file.write("#define PEC_SENSE_STATE "+pec_logic)
    config_file.write("")
    config_file.write("// Switch close (to ground) on Pin 4 for optional limit sense (stops gotos and/or tracking), default=_OFF")
    config_file.write("#define LIMIT_SENSE_"+s_limit)
    config_file.write("")
    config_file.write("// Light status LED by sink to ground (Pin 19), default=_ON.")
    config_file.write("// _ON and OnStep keeps this illuminated to indicate that the controller is active.  When sidereal tracking this LED will rapidly flash.")
    config_file.write("#define STATUS_LED_PINS_"+s_led1)
    config_file.write("// Light 2nd status LED by sink to ground (Pin 22), default=_OFF.")
    config_file.write("// _ON sets this to blink at 1 sec intervals when PPS is synced.  Turns off if tracking is stopped.  Turns on during gotos.")
    
    if led2:
        if led2_intensity!=0:
            config_file.write("#define STATUS_LED2_PINS "+np.str(led2_intensity))
        else:
            config_file.write("#define STATUS_LED2_PINS_ON")
    else:
        config_file.write("#define STATUS_LED2_PINS_OFF")


    config_file.write("// Light reticule LED by sink to ground (Pin 22), default=_OFF.  (don't use with STATUS_LED2_PINS_ON)")
    config_file.write("// RETICULE_LED_PINS n, where n=0 to 255 activates this feature and sets default brightness")
    
    if reticule:
        config_file.write("#define RETICULE_LED_PINS "+np.str(ret_intensity))
    else:
        config_file.write("#define RETICULE_LED_PINS_OFF")
    
    config_file.write("")
    config_file.write("// Sound/buzzer on Pin 29, default=_OFF.")
    config_file.write("// Specify frequency for a piezo speaker (for example \"BUZZER 2000\") or use BUZZER_ON for a piezo buzzer.")
    if buzzer:
        if buzzer_type == "Buzzer":
            config_file.write("#define BUZZER_ON")
        elif buzzer_type == "Speaker":
            config_file.write("#define BUZZER "+np.str(freq_sound)+" Hz")
    else:
        config_file.write("#define BUZZER_OFF")
                          
    config_file.write("// Sound state at startup, default=_ON.")
    config_file.write("#define DEFAULT_SOUND_"+sound)
    config_file.write("")
    config_file.write("// Optionally adjust tracking rate to compensate for atmospheric refraction, default=_OFF")
    config_file.write("// can be turned on/off with the :Tr# and :Tn# commands regardless of this setting")
    config_file.write("#define TRACK_REFRACTION_RATE_DEFAULT_"+s_atm)
    config_file.write("")
    config_file.write("// Set to _ON and OnStep will remember the last auto meridian flip setting (on/off), default=_OFF")
    config_file.write("#define REMEMBER_AUTO_MERIDIAN_FLIP_"+mem_flip)
    config_file.write("")
    config_file.write("// Set to _ON and OnStep will remember the last meridian flip pause at home setting (on/off), default=_OFF")
    config_file.write("#define REMEMBER_PAUSE_HOME_"+p_home)
    config_file.write("")
    config_file.write("// Auto Tracking at Start")
    config_file.write("#define AUTOSTART_TRACKING_"+a_sid)
    config_file.write("")
    config_file.write("// ADJUST THE FOLLOWING TO MATCH YOUR MOUNT --------------------------------------------------------------------------------")
    config_file.write(" #define REMEMBER_MAX_RATE_"+mem_rate+"        // set to _ON and OnStep will remember rates set in the ASCOM driver, Android App, etc. default=_OFF ")
    config_file.write(" #define MaxRate                   "+np.str(max_rate)+" // microseconds per microstep default setting for gotos, can be adjusted for two times lower or higher at run-time")
    config_file.write("                                     // minimum* (fastest goto) is around 12 (Teensy3.5,) 4 (Teensy3.6,) default=96 higher is ok")
    config_file.write("                                     // * = minimum can be lower, when both AXIS1/AXIS2_MICROSTEPS are used the compiler will warn you if it's too low")
    config_file.write("")
    config_file.write("#define DegreesForAcceleration   "+np.str(accel)+" // approximate number of degrees for full acceleration or deceleration: higher values=longer acceleration/deceleration")
    config_file.write("                                     // Default=5.0, too low (about <1) can cause gotos to never end if micro-step mode switching is enabled.")
    config_file.write("#define DegreesForRapidStop      "+np.str(rapid_stop)+" // approximate number of degrees required to stop when requested or if limit is exceeded during a slew: higher values=longer deceleration")
    config_file.write("                                     // Default=1.0, too low (about <1) can cause gotos to never end if micro-step mode switching is enabled.")
    config_file.write("")
    config_file.write("#define BacklashTakeupRate        "+np.str(backlash)+" // backlash takeup rate (in multipules of the sidereal rate): too fast and your motors will stall,")
    config_file.write("                                     // too slow and the mount will be sluggish while it moves through the backlash")
    config_file.write("                                     // for the most part this doesn't need to be changed, but adjust when needed.  Default=25")
    config_file.write("")
    config_file.write("                                     // Axis1 is for RA/Az")
    config_file.write("#define StepsPerDegreeAxis1  "+np.str(step_degre_axis1)+" // calculated as    :  stepper_steps * micro_steps * gear_reduction1 * (gear_reduction2/360)")
    config_file.write("                                     // G11              :  400           * 32          * 1               *  360/360              = 12800")
    config_file.write("                                     // Axis2 is for Dec/Alt")
    config_file.write("#define StepsPerDegreeAxis2  "+np.str(step_degre_axis2)+" // calculated as    :  stepper_steps * micro_steps * gear_reduction1 * (gear_reduction2/360)")
    config_file.write("                                     // G11              :  400           * 32          * 1               *  360/360              = 12800")
    config_file.write("")
    config_file.write("                                     // PEC, number of steps for a complete worm rotation (in RA), (StepsPerDegreeAxis1*360)/gear_reduction2.  Ignored on Alt/Azm mounts.")
    config_file.write("#define StepsPerWormRotationAxis1 "+np.str(np.int32(step_degre_axis1))+"L")
    config_file.write("                                     // G11              : (12800*360)/360 = 12800")
    config_file.write("")
    config_file.write("#define PECBufferSize           "+np.str(pec_buffer)+"  // PEC, buffer size, max should be no more than 3384, your required buffer size >= StepsPerAxis1WormRotation/(StepsPerDegeeAxis1/240)")
    config_file.write("                                     // for the most part this doesn't need to be changed, but adjust when needed.  824 seconds is the default.  Ignored on Alt/Azm mounts.")
    config_file.write("")
    config_file.write("#define MinutesPastMeridianE      "+np.str(degre_e)+" // for goto's, how far past the meridian to allow before we do a flip (if on the East side of the pier) - a half hour of RA is the default = 30.  Sometimes used for Fork mounts in Align mode.  Ignored on Alt/Azm mounts.")
    config_file.write("#define MinutesPastMeridianW      "+np.str(degre_w)+" // as above, if on the West side of the pier.  If left alone, the mount will stop tracking when it hits the this limit.  Sometimes used for Fork mounts in Align mode.  Ignored on Alt/Azm mounts.")
    config_file.write("                                     // The above two lines can be removed and settings in EEPROM will be used instead, be sure to set the Meridian limits in control software if you do this!")
    config_file.write("                                     // If you don't remove these lines Meridian limits will return to these defaults on power up.")
    config_file.write("#define UnderPoleLimit            "+np.str(pol_limit)+" // maximum allowed hour angle (+/-) under the celestial pole.  Default=12.  Ignored on Alt/Azm mounts.")
    config_file.write("                                     // If left alone, the mount will stop tracking when it hits this limit.  Valid range is 10 to 12 hours.")
    config_file.write("#define MinDec                   "+np.str(min_dec)+" // minimum allowed declination, default = -91 (off)  Ignored on Alt/Azm mounts.")
    config_file.write("#define MaxDec                   "+np.str(max_dec)+" // maximum allowed declination, default =  91 (off)  Ignored on Alt/Azm mounts.")
    config_file.write("                                     // For example, a value of +80 would stop gotos/tracking near the north celestial pole.")
    config_file.write("                                     // For a Northern Hemisphere user, this would stop tracking when the mount is in the polar home position but")
    config_file.write("                                     // that can be easily worked around by doing an alignment once and saving a park position (assuming a ")
    config_file.write("                                     // fork/yolk mount with meridian flips turned off by setting the minutesPastMeridian values to cover the whole sky)")
    config_file.write("#define MaxAzm                   "+np.str(max_az)+" // Alt/Az mounts only. +/- maximum allowed Azimuth, default =  180.  Allowed range is 180 to 360")
    config_file.write("")
    config_file.write("// AXIS1/2 STEPPER DRIVER CONTROL ------------------------------------------------------------------------------------------")
    config_file.write("// Axis1: Pins 20,21 = Step,Dir (RA/Azm)")
    config_file.write("// Axis2: Pins  3, 2 = Step,Dir (Dec/Alt)")
    config_file.write("")
    config_file.write("// Reverse the direction of movement.  Adjust as needed or reverse your wiring so things move in the right direction")
    config_file.write("#define AXIS1_REVERSE_"+rev1+"            // RA/Azm axis")
    config_file.write("#define AXIS2_REVERSE_"+rev2+"            // Dec/Alt axis")
    config_file.write("")
    config_file.write("// Stepper driver Enable support, just wire Enable to Pins 14 (Axis1) and 9 (Axis2) and OnStep will pull these HIGH to disable the stepper drivers on startup and when Parked or Homed.")
    config_file.write("// An Align, Sync, or Un-Park will enable the drivers.  Adjust below if you need these pulled LOW to disable the drivers.")
    config_file.write("#define AXIS1_DISABLE "+ena1)
    config_file.write("#define AXIS2_DISABLE "+ena2)
    config_file.write("")
    config_file.write("// For equatorial mounts, _ON powers down the Declination axis when it's not being used to help lower power use.  During low rate guiding (<=1x) the axis stays enabled")
    config_file.write("// for 10 minutes after any guide on either axis.  Otherwise, the Dec axis is disabled (powered off) 10 seconds after movement stops.")
    config_file.write("#define AXIS2_AUTO_POWER_DOWN_"+off2)
    config_file.write("")
    config_file.write("// Basic stepper driver mode setup . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .")
    config_file.write("// If used, this requires connections M0, M1, and M2 on Pins 15,16,17 for Axis1 (RA/Azm) and Pins 8,7,6 for Axis2 (Dec/Alt.)")
    config_file.write("// Stepper driver models are as follows: (for example AXIS1_DRIVER_MODEL DRV8825,) A4988, LV8729, RAPS128, TMC2208, TMC2130 (spreadCycle,) ")
    config_file.write("// TMC2130_QUIET (stealthChop tracking,) TMC2130_VQUIET (full stealthChop mode,) add _LOWPWR for 50% power during tracking (for example: TMC2130_QUIET_LOWPWR)")
    
    config_file.write("#define AXIS1_DRIVER_MODEL "+driver1_mod+"      // Axis1 (RA/Azm):  Default _OFF, Stepper driver model (see above)")                   
    
    if micro1 == "OFF":
        config_file.write("#define AXIS1_MICROSTEPS_"+micro1+"        // Axis1 (RA/Azm):  Default _OFF, Microstep mode when the scope is doing sidereal tracking (for example: AXIS1_MICROSTEPS 32)")
    else:
        config_file.write("#define AXIS1_MICROSTEPS "+micro1+"        // Axis1 (RA/Azm):  Default _OFF, Microstep mode when the scope is doing sidereal tracking (for example: AXIS1_MICROSTEPS 32)")
    
    if slew1 == "OFF":
        config_file.write("#define AXIS1_MICROSTEPS_GOTO_"+slew1+"   // Axis1 (RA/Azm):  Default _OFF, Optional microstep mode used during gotos (for example: AXIS1_MICROSTEPS_GOTO 2)")
    else:
        config_file.write("#define AXIS1_MICROSTEPS_GOTO "+slew1+"   // Axis1 (RA/Azm):  Default _OFF, Optional microstep mode used during gotos (for example: AXIS1_MICROSTEPS_GOTO 2)")
    
    config_file.write("#define AXIS2_DRIVER_MODEL "+driver2_mod+"      // Axis2 (Dec/Alt): Default _OFF, Stepper driver model (see above)")
    
    if micro2 == "OFF":
        config_file.write("#define AXIS2_MICROSTEPS_"+micro2+"        // Axis2 (Dec/Alt): Default _OFF, Microstep mode when the scope is doing sidereal tracking")
    else:
        config_file.write("#define AXIS2_MICROSTEPS "+micro2+"        // Axis2 (Dec/Alt): Default _OFF, Microstep mode when the scope is doing sidereal tracking")
        
    if slew2 == "OFF":
        config_file.write("#define AXIS2_MICROSTEPS_GOTO_"+slew2+"   // Axis2 (Dec/Alt): Default _OFF, Optional microstep mode used during gotos")
    else:
        config_file.write("#define AXIS2_MICROSTEPS_GOTO "+slew2+"   // Axis2 (Dec/Alt): Default _OFF, Optional microstep mode used during gotos")
        
    config_file.write("// Note: you can replace this section with the contents of \"AdvancedStepperSetup.txt\" . . . . . . . . . . . . . . . . . . . ")
    config_file.write("")
    config_file.write("// Stepper driver Fault detection on Pins 18 (Aux1) and 5 (Aux2,) choose only one feature to use on Aux1/2.  The SPI interface (on M0/M1/M2/Aux) can be used to detect errors on the TMC2130.")
    config_file.write("// other settings are LOW, HIGH, TMC2130 (if available applies internal pullup if LOW and pulldown if HIGH.)")
    if fault1 == "OFF":
        config_file.write("#define AXIS1_FAULT_"+fault1)
    else:
        config_file.write("#define AXIS1_FAULT "+fault1)
                          
    if fault2 == "OFF":
        config_file.write("#define AXIS2_FAULT_"+fault2)
    else:
        config_file.write("#define AXIS2_FAULT_"+fault2)
                          
    config_file.write("")
    config_file.write("// ------------------------------------------------------------------------------------------------------------------------")
    config_file.write("// FOCUSER ROTATOR OR ALT/AZ DE-ROTATION ----------------------------------------------------------------------------------")
    config_file.write("// Pins 30,33 = Step,Dir (choose either this option or the second focuser, not both)")
    config_file.write("#define ROTATOR_"+rotator+"                  // enable or disable rotator feature (for any mount type,) default=_OFF (de-rotator is available only for MOUNT_TYPE_ALTAZM.)")
    config_file.write("#define MaxRateAxis3               "+np.str(rot_rate)+" // this is the minimum number of milli-seconds between micro-steps, default=8")
    config_file.write("#define StepsPerDegreeAxis3     "+np.str(step_degre_rot)+" // calculated as    :  stepper_steps * micro_steps * gear_reduction1 * (gear_reduction2/360)")
    config_file.write("                                     // Rotator          :  24            * 8           * 20              *  6/360                = 64")
    config_file.write("                                     // For de-rotation of Alt/Az mounts a quick estimate of the required resolution (in StepsPerDegree)")
    config_file.write("                                     // would be an estimate of the circumference of the useful imaging circle in (pixels * 2)/360")
    config_file.write("#define MinAxis3                "+np.str(rot_min_degr)+" // minimum allowed Axis3 rotator, default = -180")
    config_file.write("#define MaxAxis3                 "+np.str(rot_max_degr)+" // maximum allowed Axis3 rotator, default =  180")
    config_file.write("#define AXIS3_REVERSE_"+rot_rev+"            // reverse the direction of Axis3 rotator movement")
    
    if rot_disable == "OFF":
        config_file.write("#define AXIS3_DISABLE_"+rot_disable+"            // Pin 36 (Aux3.)  Use HIGH for common stepper drivers if you want to power down the motor at stand-still.  Default _OFF.")
    else:
        config_file.write("#define AXIS3_DISABLE "+rot_disable+"            // Pin 36 (Aux3.)  Use HIGH for common stepper drivers if you want to power down the motor at stand-still.  Default _OFF.")
        
    config_file.write("")
    config_file.write("// FOCUSER1 ---------------------------------------------------------------------------------------------------------------")
    config_file.write("// Pins 34,35 = Step,Dir")
    config_file.write("#define FOCUSER1_"+focus1+"                 // enable or disable focuser feature, default=_OFF")
    config_file.write("#define MaxRateAxis4               "+np.str(foc1_rate)+" // this is the minimum number of milli-seconds between micro-steps, default=8")
    config_file.write("#define StepsPerMicrometerAxis4  "+np.str(foc1_ratio)+" // figure this out by testing or other means")
    config_file.write("#define MinAxis4               "+np.str(foc1_min_mm)+" // minimum allowed Axis4 position in millimeters, default = -25.0")
    config_file.write("#define MaxAxis4                "+np.str(foc1_max_mm)+" // maximum allowed Axis4 position in millimeters, default =  25.0")
    config_file.write("#define AXIS4_REVERSE_"+foc1_rev+"            // reverse the direction of Axis4 focuser movement")
    
    if focus1_disable == "OFF":
        config_file.write("#define AXIS4_DISABLE_"+focus1_disable+"            // Pin 39 (Aux4.)  Use HIGH for common stepper drivers if you want to power down the motor at stand-still.  Default _OFF.")
    else:                      
        config_file.write("#define AXIS4_DISABLE "+focus1_disable+"            // Pin 39 (Aux4.)  Use HIGH for common stepper drivers if you want to power down the motor at stand-still.  Default _OFF.")
    
    config_file.write("")
    
    if board != "TM4C":
        config_file.write("// FOCUSER2 ---------------------------------------------------------------------------------------------------------------")
        config_file.write("// Pins 30,33 = Step,Dir (choose either this option or the rotator, not both) ")
        config_file.write("#define FOCUSER2_"+focus2+"                 // enable or disable focuser feature, default=_OFF")
        config_file.write("#define MaxRateAxis5               "+np.str(foc2_rate)+" // this is the minimum number of milli-seconds between micro-steps, default=8")
        config_file.write("#define StepsPerMicrometerAxis5  "+np.str(foc2_ratio)+" // figure this out by testing or other means")
        config_file.write("#define MinAxis5               "+np.str(foc2_min_mm)+" // minimum allowed Axis5 position in millimeters, default = -25.0")
        config_file.write("#define MaxAxis5                "+np.str(foc2_max_mm)+" // maximum allowed Axis5 position in millimeters, default =  25.0")
        config_file.write("#define AXIS5_REVERSE_"+foc2_rev+"            // reverse the direction of Axis5 focuser movement")
        
        if focus2_disable == "OFF":                  
            config_file.write("#define AXIS5_DISABLE_"+focus2_disable+"            // Pin 36 (Aux3.)  Use HIGH for common stepper drivers if you want to power down the motor at stand-still.  Default _OFF.")
        else:
            config_file.write("#define AXIS5_DISABLE "+focus2_disable+"            // Pin 36 (Aux3.)  Use HIGH for common stepper drivers if you want to power down the motor at stand-still.  Default _OFF.")
                          
        config_file.write("")
        
    config_file.write("// THAT'S IT FOR USER CONFIGURATION!")
    config_file.write("")
    config_file.write("// -------------------------------------------------------------------------------------------------------------------------")
    config_file.write("#define FileVersionConfig 2")
    config_file.write("#include \"Pins."+board+".h\"")
    config_file.write("#endif")
                      
    config_file.close()