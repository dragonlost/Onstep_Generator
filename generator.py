# test.py

import time

version = "BETA_0.7"

def read_conf_file(path_name):
    file = open(path_name,'r')
    
    text = file.readlines()
    for i in range(len(text)) : text[i]=text[i].strip()
    # Convertion des variables (float, int, bool)
    
    ff = [4,5,13,14,28,29,31,32,35,37,38,41,43,44,80,81]#float
    ii = [2,6,15,25,26,34,40,51,59,64,69,71,74,82,84,85,86,87,88,89]# integer
    bol = [3,10,19,22,23,24,30,36,42,48,49,50,52,53,54,55,56,57,58,61,62,63,66,
           67,68,70,72,75,76,77,78,79,83,90]# boolen
    ss = [0,1,7,8,9,11,12,16,17,18,20,21,27,33,39,45,46,47,60,65,73]#string
    var_str = ['board_type','mount_type','max_rate','auto_sid',
               'worm1','gear1','stepper1','micro1','slew1','driver1','reverse1','ena1','fault1',
               'worm2','gear2','stepper2','micro2','slew2','driver2','reverse2','ena2','fault2',
               'rot3','foc1','foc2',
               'rot_rate','rot_step','rot_micro','rot_gear','rot_gear_2','rot_reverse','rot_min_degr','rot_max_degr','rot_disable',
               'foc1_rate','foc1_ratio','foc1_reverse','foc1_min_mm','foc1_max_mm','focus1_disable',
               'foc2_rate','foc2_ratio','foc2_reverse','foc2_min_mm','foc2_max_mm','focus2_disable',
               'baud','baud4', 'esp','pec','pec_pul','pec_buffer',
               'strict_park','st4','st4_pul','alt_st4','hand','pulse','guide_time',
               'rtc','pps','pps_pul','pec_set','analog_pec','pec_logic','limit',
               'led1','led2','led2_intensity','reticule','ret_intensity','buzzer',
               'buzzer_type','freq_sound','def_sound','atmos','mem_flip_mer',
               'home_pause','mem_max_rate','accel','rapid_stop','backlash','off_axis2',
               'degre_e','degre_w','min_dec','max_dec','pol_limit','max_az','s_side',
               'rot_autodi','foc1_autodi','foc2_autodi', 'home_flip_mer',
               'home_sense','ax1_home_rev','ax2_home_rev', 'ax1_home_rev_v', 'ax2_home_rev_v']
    dico = {}
    for i in range(len(text)):
        
        if i in ff:
            dico[var_str[i]] = float(text[i])
            
        elif i in ii:
            dico[var_str[i]] = int(text[i])
            
        elif i in bol:
            dico[var_str[i]] = bool_convert(text[i])
            
        elif i in ss:
            dico[var_str[i]] = str(text[i])

    file.close()
    
    return dico

def bool_convert(var_str):

    if var_str == 'True':
        var_bool = True
    else:
        var_bool = False
        
    return var_bool


def onstep_config(path_read, path=""):
    
    dico = read_conf_file(path_read)
    
    ####__________________Axis1_____________________________
    # Axis1 reverse
    if dico["reverse1"]:
        rev1="ON"
    else:
        rev1="OFF"
    step_degre_axis1 = dico["stepper1"] * int(dico["micro1"]) * dico["worm1"] * (dico["gear1"]/360.)
    
    # Driver Type Axis 1
    if dico["driver1"] == "A4988":
        driver1_mod="A4988"
    elif dico["driver1"] == "DRV8825":
        driver1_mod="DRV8825"
    elif dico["driver1"] == "LV8729 or RAPS128":
        driver1_mod="LV8729"
    elif dico["driver1"] == "TMC2208":
        driver1_mod="TMC2208"
    elif dico["driver1"] == "TMC2130":
        driver1_mod="TMC2130"
    elif dico["driver1"] == "TMC2100":
        driver1_mod="TMC2100"
    elif dico["driver1"] == "TMC2130 (Quiet)":
        driver1_mod="TMC2130_QUIET"
    elif dico["driver1"] == "TMC2130 (VQuiet)":
        driver1_mod="TMC2130_VQUIET"
    elif dico["driver1"] == "TMC2130 (Quiet, LowPWR)":
        driver1_mod="TMC2130_QUIET_LOWPWR"
    elif dico["driver1"] == "TMC2130 (VQuiet, LowPWR)":
        driver1_mod="TMC2130_VQUIET_LOWPWR"
        
    ####__________________Axis2_____________________________
    # Axis2 reverse
    if dico["reverse2"]:
        rev2="ON"
    else:
        rev2="OFF"
    step_degre_axis2 = dico["stepper2"] * int(dico["micro2"]) * dico["worm2"] * (dico["gear2"]/360.) 
    
    # Driver Type Axis 2
    if dico["driver2"] == "A4988":
        driver2_mod="A4988"
    elif dico["driver2"] == "DRV8825":
        driver2_mod="DRV8825"
    elif dico["driver2"] == "LV8729 or RAPS128":
        driver2_mod="LV8729"
    elif dico["driver2"] == "TMC2208":
        driver2_mod="TMC2208"
    elif dico["driver2"] == "TMC2130":
        driver2_mod="TMC2130"
    elif dico["driver1"] == "TMC2100":
        driver1_mod="TMC2100"
    elif dico["driver2"] == "TMC2130 (Quiet)":
        driver2_mod="TMC2130_QUIET"
    elif dico["driver2"] == "TMC2130 (VQuiet)":
        driver2_mod="TMC2130_VQUIET"
    elif dico["driver2"] == "TMC2130 (Quiet, LowPWR)":
        driver2_mod="TMC2130_QUIET_LOWPWR"
    elif dico["driver2"] == "TMC2130 (VQuiet, LowPWR)":
        driver2_mod="TMC2130_VQUIET_LOWPWR"
    
    ####__________________Focuser1__________________________
    # Focus 1 reverse
    if dico["foc1_reverse"]:
        foc1_rev="ON"
    else:
        foc1_rev="OFF"
    
    # Focus 1 ON/OFF
    if dico["foc1"]:
        focus1="ON"
    else:
        focus1="OFF"   
        
    ####__________________Focuser2__________________________
    # Focus 2 reverse
    if dico["foc2_reverse"]:
        foc2_rev="ON"
    else:
        foc2_rev="OFF"  
      
    # Focus 2 ON/OFF
    if dico["foc2"]:
        focus2="ON"
    else:
        focus2="OFF"
        
    ####__________________Rotator___________________________
    # Rotator reverse
    if dico["rot_reverse"]:
        rot_rev="ON"
    else:
        rot_rev="OFF"
    
    # Rotator ON/OFF
    if dico["rot3"]:
        rotator="ON"
    else:
        rotator="OFF"

    step_degre_rot = dico["rot_step"]  * int(dico["rot_micro"]) * dico["rot_gear"] * (dico["rot_gear_2"]/360)
    
    ####__________________OPTION_____________________________
    
    # Board Type
    if dico["board_type"] == "RAMPS 1.4 or 1.5":
        board="Ramps14"
    elif dico["board_type"] == "MiniPCB (2 axis)":
        board="MiniPCB"
    elif dico["board_type"] == "MaxPCB (4 axis)":
        board="MaxPCB"
    elif dico["board_type"] == "STM32F1 (outdate)":
        board="STM32"
    elif dico["board_type"] == "TivaC (just for BETA)":
        board="TM4C"
    elif dico["board_type"] == "Classic":
        board="Classic"
    elif dico["board_type"] == "Mega2560 Alternate":
        board="Mega2560Alt"
    elif dico["board_type"] == "Custom":
        board="Custom"
    
    # Mount Type
    if dico["mount_type"] == "Equatorial":
        mount="GEM"
    elif dico["mount_type"] == "Fork":
        mount="FORK"
    elif dico["mount_type"] == "Alt-Azimuth":
        mount="FORK_ALT"
        
    # ESP8266 Control
    if dico["esp"]:
        esp_s="ON"
    else:
        esp_s="OFF"
        
    # Strict Parking
    if dico["strict_park"]:
        s_park="ON"
    else:
        s_park="OFF"
        
        
    # alternative st4
    if dico["alt_st4"]:
        a_st4="ON"
    else:
        a_st4="OFF"    

    # hand controler st4
    if dico["hand"]:
        h_st4="ON"
    else:
        h_st4="OFF"

    # Separate Pulse guide rate
    if dico["pulse"]:
        pulse_g="ON"
    else:
        pulse_g="OFF"  
        
    # Limit Sensor
    if dico["limit"]:
        s_limit="ON"
    else:
        s_limit="OFF" 
        
    # Default sound at Startup
    if dico["def_sound"]:
        sound="ON"
    else:
        sound="OFF"
        
    # Refraction tracking
    if dico["atmos"]:
        s_atm="ON"
    else:
        s_atm="OFF"

    # sync side
    if dico["s_side"]:
        sync_side="ON"
    else:
        sync_side="OFF"	

    # Home Pause
    if dico["home_pause"]:
        p_home="ON"
    else:
        p_home="OFF"
        
    # Remember Max Rate 
    if dico["mem_max_rate"]:
        mem_rate="ON"
    else:
        mem_rate="OFF"
    
    # Remember Flip meridian position    
    if dico["mem_flip_mer"]:
        mem_flip="ON"
    else:
        mem_flip="OFF"
        
    # meridian flip without home position  
    if dico["home_flip_mer"]:
        home_flip="ON"
    else:
        home_flip="OFF"
        
    if dico["home_sense"]:
        home_s="ON"
    else:
        home_s="OFF"
        
    # Led 1 status    
    if dico["led1"]:
        s_led1="ON"
    else:
        s_led1="OFF"
        
        
    # Off Axis 2 at time limit    
    if dico["off_axis2"]:
        off2="ON"
    else:
        off2="OFF"
        
    # Set auto sideral tracking at start
    if dico["auto_sid"]:
        a_sid="ON"
    else:
        a_sid="OFF"

    if path == "":
        config_file = open("Config."+board+".h","w")
        
    else:
        config_file = open(path,"w")
        
    #==========================================================================
    #==========================================================================
    # START
    #==========================================================================
    #==========================================================================
        
    config_file.write("// Config File Generate by OnStep Generator, ver. "+version+"\n")
    config_file.write("// Creation Date : "+ time.strftime("%d %m %Y, %H:%M:%S", time.localtime())+"\n")
    config_file.write("// -----------------------------------------------------------------------------------\n")
    config_file.write("// Configuration for OnStep"+board+"\n")
    config_file.write("\n")
    config_file.write("/*\n")
    config_file.write("* For more information on setting OnStep up see http://www.stellarjourney.com/index.php?r=site/equipment_onstep and\n")
    config_file.write("* join the OnStep Groups.io at https://groups.io/g/onstep\n")
    config_file.write("* \n")
    config_file.write("* See the Pins."+board+".h file for detailed information on this pin map to be sure it matches your wiring *** USE AT YOUR OWN RISK ***\n")
    config_file.write("*\n")
    config_file.write("*/\n")
    config_file.write("\n")
    config_file.write("#define "+board+"_ON\n")
    config_file.write("\n")
    config_file.write("#ifdef "+board+"_ON\n")
                    
    #==========================================================================
    #==========================================================================
    # Options
    #==========================================================================
    #==========================================================================
    config_file.write("// -------------------------------------------------------------------------------------------------------------------------\n")
    config_file.write("// ADJUST THE FOLLOWING TO CONFIGURE YOUR CONTROLLER FEATURES --------------------------------------------------------------\n")
    config_file.write("\n")
    config_file.write("// Default speed for Serial1=B and Serial4=C com ports, Default=9600\n")
    config_file.write("#define SERIAL_B_BAUD_DEFAULT "+dico["baud"]+"\n")
                      
    if ((board =="MaxPCB") or (board =="TM4C")):
        config_file.write("#define SERIAL_C_BAUD_DEFAULT "+dico["baud4"]+"\n")
                          
    config_file.write("\n")
    config_file.write("// ESP8266 reset and GPIO0 control, this sets run mode for normal operation.  Uploading programmer firmware to the OpStep MCU can then enable sending new firmware to the ESP8266-01\n")
    config_file.write("// Pin 18 (Aux1) for GPIO0 and Pin 5 (Aux2) for Rst control.  Choose only one feature on Aux1/2.\n")
    config_file.write("#define ESP8266_CONTROL_"+esp_s+"\n")
    config_file.write("\n")
    config_file.write("// Mount type, default is _GEM (German Equatorial) other options are _FORK, _FORK_ALT.  _FORK switches off Meridian Flips after (1, 2 or 3 star) alignment is done.  _FORK_ALT disables Meridian Flips (1 star align.)\n")
    config_file.write("// _ALTAZM is for Alt/Azm mounted 'scopes (1 star align only.)\n")
    config_file.write("#define MOUNT_TYPE_"+mount+"\n")
    config_file.write("\n")
    config_file.write("// Strict parking, default=_OFF.  Set to _ON and unparking is only allowed if successfully parked.  Otherwise unparking is allowed if at home and not parked (the Home/Reset command \":hF#\" sets this state.) \n")
    config_file.write("#define STRICT_PARKING_"+s_park+"\n")
    config_file.write("\n")
    config_file.write("// ST4 interface on pins 24, 25, 26, 27.  Pin 24 is RA- (West), Pin 25 is Dec- (South), Pin 26 is Dec+ (North), Pin 27 is RA+ (East.)\n")
    config_file.write("// ST4_ON enables the interface.  ST4_PULLUP enables the interface and any internal pullup resistors.\n")
    config_file.write("// It is up to you to create an interface that meets the electrical specifications of any connected device, use at your own risk.  default=_OFF\n")
    
    
    if dico["st4"]:
        if dico["st4_pul"]:
            config_file.write("#define ST4_PULLUP\n")
        else:
            config_file.write("#define ST4_ON\n")
    else:
        config_file.write("#define ST4_OFF\n")
    
                      
    config_file.write("// If SEPARATE_PULSE_GUIDE_RATE_ON is used the ST4 port is limited to guide rates <= 1X except when ST4_HAND_CONTROL_ON is used.\n")
    config_file.write("// Additionally, ST4_HAND_CONTROL_ON enables special features: Press and hold [E]+[W] buttons for > 2 seconds...  In this mode [E] decreases and [W] increases guide rates (or if tracking isn't on yet adjusts illuminated recticule brightness.)\n")
    config_file.write("// [S] for Sync (or Accept if in align mode.) [N] for Tracking on/off. -OR- Press and hold [N]+[S] buttons for > 2 seconds...  In this mode [E] selects prior and [W] next user catalog item.\n")
    config_file.write("// [N] to do a Goto to the catalog item.  [S] for Sound on/off.  The keypad returns to normal operation after 4 seconds of inactivity.  ST4_HAND_CONTROL_ON also adds a 100ms de-bounce to all button presses.\n")
    config_file.write("// Finally, during a goto pressing any button aborts the slew.  If meridian flip paused at home, pressing any button continues.  default=_ON\n")
    config_file.write("#define ST4_HAND_CONTROL_"+h_st4+"\n")
                      
    if board == "Classic":
        config_file.write("// ST4_ALTERNATE_PINS_ON moves the interface (Mega2560 only) to pins 43, 45, 47, 49.  Pin 43 is Dec- (South), Pin 45 is Dec+ (North), Pin 47 is RA- (West), Pin 49 is RA+ (East.)\n")
        config_file.write("// ST4_ALTERNATE_PINS_ON is required for Steve's ST4 board and is also required if the ST4 interface is to be used alongside the SPI interface\n")
        config_file.write("#define ST4_ALTERNATE_PINS_"+a_st4+"\n")
                          
    config_file.write("\n")
    config_file.write("// Separate pulse-guide rate so centering and guiding don't disturb each other, default=_ON\n")
    config_file.write("#define SEPARATE_PULSE_GUIDE_RATE_"+pulse_g+"\n")
    config_file.write("\n")
    config_file.write("// Guide time limit (in seconds,) default=0 (no limit.)  A safety feature, some guides are started with one command and stopped with another.\n")
    config_file.write("// If the stop command is never received the guide will continue forever unless this is enabled.\n")
    config_file.write("#define GUIDE_TIME_LIMIT "+str(dico["guide_time"])+"\n")
    config_file.write("\n")
    if ((board != "Classic") or (board !="STM32")):
        config_file.write("// RTC (Real Time Clock) support, default=_OFF.\n")
        config_file.write("// Other options: RTC_DS3234 for a DS3234 on the default SPI interface (CS on pin 10) or RTC_DS3231 for a DS3231 on the default I2C pins (optionally wire the SQW output to the PPS pin below.)\n")
        config_file.write("#define RTC_"+dico["rtc"]+"\n")
    config_file.write("// PPS use _ON or _PULLUP to enable the input and use the built-in pullup resistor.  Sense rising edge on Pin 28 for optional precision clock source (GPS, for example), default=_OFF\n")
    
    if dico["pps"]:
        if dico["pps_pul"]:
            config_file.write("#define PPS_SENSE_PULLUP\n")
        else:
            config_file.write("#define PPS_SENSE_ON\n")
    else:
        config_file.write("#define PPS_SENSE_OFF\n")
    
                      
    config_file.write("// Note: The MaxPCB has a DS3234 connector\n")
    config_file.write("\n")
    config_file.write("// PEC sense on Pin 23 (A9) use _ON or _PULLUP to enable the input/use the built-in pullup resistor (digital input) or provide a comparison value (see below) for analog operation, default=_OFF\n")
    config_file.write("// Analog values range from 0 to 1023 which indicate voltages from 0-3.3VDC on the analog pin, for example \"PEC_SENSE 600\" would detect an index when the voltage exceeds 1.93V\n")
    config_file.write("// With either index detection method, once triggered 60s must expire before another detection can happen.  This gives time for the index magnet to pass by the detector before another cycle begins.\n")
    config_file.write("// Ignored on Alt/Azm mounts.\n")
    if dico["pec"]:
        if (dico["pec_set"] and dico["analog_pec"]!=0):
            config_file.write("#define PEC_SENSE "+str(dico["analog_pec"])+"\n")
        else:
            if dico["pec_pul"]:
                config_file.write("#define PEC_SENSE_PULLUP\n")
            else:
                config_file.write("#define PEC_SENSE_ON\n")
    else:
        config_file.write("#define PEC_SENSE_OFF\n")
                          
    config_file.write("// PEC sense, rising edge (default with PEC_SENSE_STATE HIGH, use LOW for falling edge, ex. PEC_SENSE_ON) ; for optional PEC index\n")
    config_file.write("#define PEC_SENSE_STATE "+dico["pec_logic"]+"\n")
    config_file.write("\n")
    config_file.write("// Switch close (to ground) on Pin 4 for optional limit sense (stops gotos and/or tracking), default=_OFF\n")
    config_file.write("#define LIMIT_SENSE_"+s_limit+"\n")
    config_file.write("\n")
    config_file.write("// Light status LED by sink to ground (Pin 19), default=_ON.\n")
    config_file.write("// _ON and OnStep keeps this illuminated to indicate that the controller is active.  When sidereal tracking this LED will rapidly flash.\n")
    config_file.write("#define STATUS_LED_PINS_"+s_led1+"\n")
    config_file.write("// Light 2nd status LED by sink to ground (Pin 22), default=_OFF.\n")
    config_file.write("// _ON sets this to blink at 1 sec intervals when PPS is synced.  Turns off if tracking is stopped.  Turns on during gotos.\n")
    
    if dico["led2"]:
        if dico["led2_intensity"]!=0:
            config_file.write("#define STATUS_LED2_PINS "+str(dico["led2_intensity"])+"\n")
        else:
            config_file.write("#define STATUS_LED2_PINS_ON\n")
    else:
        config_file.write("#define STATUS_LED2_PINS_OFF\n")


    config_file.write("// Light reticule LED by sink to ground (Pin 22), default=_OFF.  (don't use with STATUS_LED2_PINS_ON)\n")
    config_file.write("// RETICULE_LED_PINS n, where n=0 to 255 activates this feature and sets default brightness\n")
    
    if dico["reticule"]:
        config_file.write("#define RETICULE_LED_PINS "+str(dico["ret_intensity"])+"\n")
    else:
        config_file.write("#define RETICULE_LED_PINS_OFF\n")
    
    config_file.write("")
    config_file.write("// Sound/buzzer on Pin 29, default=_OFF.\n")
    config_file.write("// Specify frequency for a piezo speaker (for example \"BUZZER 2000\") or use BUZZER_ON for a piezo buzzer.\n")
    if dico["buzzer"]:
        if dico["buzzer_type"] == "Buzzer":
            config_file.write("#define BUZZER_ON\n")
        elif dico["buzzer_type"] == "Speaker":
            config_file.write("#define BUZZER "+str(dico["freq_sound"])+" Hz\n")
    else:
        config_file.write("#define BUZZER_OFF\n")
                          
    config_file.write("// Sound state at startup, default=_ON.\n")
    config_file.write("#define DEFAULT_SOUND_"+sound+"\n")
    config_file.write("\n")
    config_file.write("// Optionally adjust tracking rate to compensate for atmospheric refraction, default=_OFF\n")
    config_file.write("// can be turned on/off with the :Tr# and :Tn# commands regardless of this setting\n")
    config_file.write("#define TRACK_REFRACTION_RATE_DEFAULT_"+s_atm+"\n")
    config_file.write("\n")
    config_file.write("// Set to _OFF and OnStep will allow Syncs to change pier side for GEM mounts (on/off), default=_ON\n")
    config_file.write("#define SYNC_CURRENT_PIER_SIDE_ONLY_"+sync_side+"\n")
    config_file.write("\n")
    config_file.write("// Set to _ON and OnStep will remember the last auto meridian flip setting (on/off), default=_OFF\n")
    config_file.write("#define REMEMBER_AUTO_MERIDIAN_FLIP_"+mem_flip+"\n")
    config_file.write("\n")
    config_file.write("// Set to _ON and OnStep will travel directly across a meridian flip without visiting the home position (on/off), default=_OFF (only applies if pause at home is disabled)\n")
    config_file.write("#define MERIDIAN_FLIP_SKIP_HOME_"+home_flip+"\n")
    config_file.write("\n")
    config_file.write("// Set to _ON and OnStep will remember the last meridian flip pause at home setting (on/off), default=_OFF\n")
    config_file.write("#define REMEMBER_PAUSE_HOME_"+p_home+"\n")
    config_file.write("\n")
    if ((board == "MaxPCB") or (board == "Ramps14")):
        config_file.write("// Automatic homing; switch state changes at the home position (uses Aux3 and Aux4.)\n")
        config_file.write("#define HOME_SENSE_"+home_s+"               // Default _OFF, use _ON to have OnStep automatically detect and use home switches as follows\n")
        if dico["ax1_home_rev"]:
            config_file.write("#define HOME_AXIS1_REVERSE "+dico["ax1_home_rev_v"]+"       // Pin Aux3 state should be HIGH when clockwise of the home position (as seen from front,) LOW otherwise; reverse if necessary\n")
        else:
            config_file.write("#define HOME_AXIS1_REVERSE_OFF       // Pin Aux3 state should be HIGH when clockwise of the home position (as seen from front,) LOW otherwise; reverse if necessary\n")
        if dico["ax2_home_rev"]:
            config_file.write("#define HOME_AXIS2_REVERSE_"+dico["ax2_home_rev_v"]+"       // Pin Aux4 state should be HIGH when clockwise of the home position (as seen from above,) LOW otherwise; reverse if necessary\n")
        else:
            config_file.write("#define HOME_AXIS2_REVERSE_OFF       // Pin Aux4 state should be HIGH when clockwise of the home position (as seen from above,) LOW otherwise; reverse if necessary\n")
    config_file.write("// Auto Tracking at Start\n")
    config_file.write("#define AUTOSTART_TRACKING_"+a_sid+"\n")
    config_file.write("\n")
    
    #==========================================================================
    #==========================================================================
    # Axis 1 and 2 compute
    #==========================================================================
    #==========================================================================
    
    config_file.write("// ADJUST THE FOLLOWING TO MATCH YOUR MOUNT --------------------------------------------------------------------------------\n")
    config_file.write(" #define REMEMBER_MAX_RATE_"+mem_rate+"        // set to _ON and OnStep will remember rates set in the ASCOM driver, Android App, etc. default=_OFF \n")
    config_file.write(" #define MaxRate                   "+str(dico["max_rate"])+" // microseconds per microstep default setting for gotos, can be adjusted for two times lower or higher at run-time\n")
    config_file.write("                                     // minimum* (fastest goto) is around 12 (Teensy3.5,) 4 (Teensy3.6,) default=96 higher is ok\n")
    config_file.write("                                     // * = minimum can be lower, when both AXIS1/AXIS2_MICROSTEPS are used the compiler will warn you if it's too low\n")
    config_file.write("\n")
    config_file.write("#define DegreesForAcceleration   "+str(dico["accel"])+" // approximate number of degrees for full acceleration or deceleration: higher values=longer acceleration/deceleration\n")
    config_file.write("                                     // Default=5.0, too low (about <1) can cause gotos to never end if micro-step mode switching is enabled.\n")
    config_file.write("#define DegreesForRapidStop      "+str(dico["rapid_stop"])+" // approximate number of degrees required to stop when requested or if limit is exceeded during a slew: higher values=longer deceleration\n")
    config_file.write("                                     // Default=1.0, too low (about <1) can cause gotos to never end if micro-step mode switching is enabled.\n")
    config_file.write("\n")
    config_file.write("#define BacklashTakeupRate        "+str(dico["backlash"])+" // backlash takeup rate (in multipules of the sidereal rate): too fast and your motors will stall,\n")
    config_file.write("                                     // too slow and the mount will be sluggish while it moves through the backlash\n")
    config_file.write("                                     // for the most part this doesn't need to be changed, but adjust when needed.  Default=25\n")
    config_file.write("\n")
    config_file.write("                                     // Axis1 is for RA/Az\n")
    config_file.write("#define StepsPerDegreeAxis1  "+str(step_degre_axis1)+" // calculated as    :  stepper_steps * micro_steps * gear_reduction1 * (gear_reduction2/360)\n")
    config_file.write("                                     // G11              :  400           * 32          * 1               *  360/360              = 12800\n")
    config_file.write("                                     // Axis2 is for Dec/Alt\n")
    config_file.write("#define StepsPerDegreeAxis2  "+str(step_degre_axis2)+" // calculated as    :  stepper_steps * micro_steps * gear_reduction1 * (gear_reduction2/360)\n")
    config_file.write("                                     // G11              :  400           * 32          * 1               *  360/360              = 12800\n")
    config_file.write("\n")
    config_file.write("                                     // PEC, number of steps for a complete worm rotation (in RA), (StepsPerDegreeAxis1*360)/gear_reduction2.  Ignored on Alt/Azm mounts.\n")
    config_file.write("#define StepsPerWormRotationAxis1 "+str(int(step_degre_axis1))+"L\n")
    config_file.write("                                     // G11              : (12800*360)/360 = 12800\n")
    config_file.write("\n")
    config_file.write("#define PECBufferSize           "+str(dico["pec_buffer"])+"  // PEC, buffer size, max should be no more than 3384, your required buffer size >= StepsPerAxis1WormRotation/(StepsPerDegeeAxis1/240)\n")
    config_file.write("                                     // for the most part this doesn't need to be changed, but adjust when needed.  824 seconds is the default.  Ignored on Alt/Azm mounts.\n")
    config_file.write("\n")
    config_file.write("#define MinutesPastMeridianE      "+str(dico["degre_e"])+" // for goto's, how far past the meridian to allow before we do a flip (if on the East side of the pier) - a half hour of RA is the default = 30.  Sometimes used for Fork mounts in Align mode.  Ignored on Alt/Azm mounts.\n")
    config_file.write("#define MinutesPastMeridianW      "+str(dico["degre_w"])+" // as above, if on the West side of the pier.  If left alone, the mount will stop tracking when it hits the this limit.  Sometimes used for Fork mounts in Align mode.  Ignored on Alt/Azm mounts.\n")
    config_file.write("                                     // The above two lines can be removed and settings in EEPROM will be used instead, be sure to set the Meridian limits in control software if you do this!\n")
    config_file.write("                                     // If you don't remove these lines Meridian limits will return to these defaults on power up.\n")
    config_file.write("#define UnderPoleLimit            "+str(dico["pol_limit"])+" // maximum allowed hour angle (+/-) under the celestial pole.  Default=12.  Ignored on Alt/Azm mounts.\n")
    config_file.write("                                     // If left alone, the mount will stop tracking when it hits this limit.  Valid range is 10 to 12 hours.\n")
    config_file.write("#define MinDec                   "+str(dico["min_dec"])+" // minimum allowed declination, default = -91 (off)  Ignored on Alt/Azm mounts.\n")
    config_file.write("#define MaxDec                   "+str(dico["max_dec"])+" // maximum allowed declination, default =  91 (off)  Ignored on Alt/Azm mounts.\n")
    config_file.write("                                     // For example, a value of +80 would stop gotos/tracking near the north celestial pole.\n")
    config_file.write("                                     // For a Northern Hemisphere user, this would stop tracking when the mount is in the polar home position but\n")
    config_file.write("                                     // that can be easily worked around by doing an alignment once and saving a park position (assuming a \n")
    config_file.write("                                     // fork/yolk mount with meridian flips turned off by setting the minutesPastMeridian values to cover the whole sky)\n")
    config_file.write("#define MaxAzm                   "+str(dico["max_az"])+" // Alt/Az mounts only. +/- maximum allowed Azimuth, default =  180.  Allowed range is 180 to 360\n")
    config_file.write("\n")
    
    #==========================================================================
    #==========================================================================
    # Axis 1 and 2 options
    #==========================================================================
    #==========================================================================
    
    config_file.write("// AXIS1/2 STEPPER DRIVER CONTROL ------------------------------------------------------------------------------------------\n")
    config_file.write("// Axis1: Pins 20,21 = Step,Dir (RA/Azm) (Teensy3.x Pins 12,10)\n")
    config_file.write("// Axis2: Pins  3, 2 = Step,Dir (Dec/Alt) (Teensy3.x Pins 6,4)\n")
    config_file.write("\n")
    config_file.write("// Reverse the direction of movement.  Adjust as needed or reverse your wiring so things move in the right direction\n")
    config_file.write("#define AXIS1_REVERSE_"+rev1+"            // RA/Azm axis\n")
    config_file.write("#define AXIS2_REVERSE_"+rev2+"            // Dec/Alt axis\n")
    config_file.write("\n")
    config_file.write("// Stepper driver Enable support, just wire Enable to Pins 14 (Axis1) and 9 (Axis2) and OnStep will pull these HIGH to disable the stepper drivers on startup and when Parked or Homed.\n")
    config_file.write("// An Align, Sync, or Un-Park will enable the drivers.  Adjust below if you need these pulled LOW to disable the drivers.\n")
    config_file.write("#define AXIS1_DISABLE "+dico["ena1"]+"\n")
    config_file.write("#define AXIS2_DISABLE "+dico["ena2"]+"\n")
    config_file.write("\n")
    config_file.write("// For equatorial mounts, _ON powers down the Declination axis when it's not being used to help lower power use.  During low rate guiding (<=1x) the axis stays enabled\n")
    config_file.write("// for 10 minutes after any guide on either axis.  Otherwise, the Dec axis is disabled (powered off) 10 seconds after movement stops.\n")
    config_file.write("#define AXIS2_AUTO_POWER_DOWN_"+off2+"\n")
    config_file.write("\n")
    config_file.write("// Basic stepper driver mode setup . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .\n")
    config_file.write("// If used, this requires connections M0, M1, and M2 on Pins 15,16,17 for Axis1 (RA/Azm) and Pins 8,7,6 for Axis2 (Dec/Alt.)\n")
    config_file.write("// Stepper driver models are as follows: (for example AXIS1_DRIVER_MODEL DRV8825,) A4988, LV8729, RAPS128, TMC2208, TMC2130 (spreadCycle,) \n")
    config_file.write("// TMC2130_QUIET (stealthChop tracking,) TMC2130_VQUIET (full stealthChop mode,) add _LOWPWR for 50% power during tracking (for example: TMC2130_QUIET_LOWPWR)\n")
    
    config_file.write("#define AXIS1_DRIVER_MODEL "+driver1_mod+"      // Axis1 (RA/Azm):  Default _OFF, Stepper driver model (see above)\n")                   
    
    if dico["micro1"] == "OFF":
        config_file.write("#define AXIS1_MICROSTEPS_"+dico["micro1"]+"        // Axis1 (RA/Azm):  Default _OFF, Microstep mode when the scope is doing sidereal tracking (for example: AXIS1_MICROSTEPS 32)\n")
    else:
        config_file.write("#define AXIS1_MICROSTEPS "+dico["micro1"]+"        // Axis1 (RA/Azm):  Default _OFF, Microstep mode when the scope is doing sidereal tracking (for example: AXIS1_MICROSTEPS 32)\n")
    
    if dico["slew1"] == "OFF":
        config_file.write("#define AXIS1_MICROSTEPS_GOTO_"+dico["slew1"]+"   // Axis1 (RA/Azm):  Default _OFF, Optional microstep mode used during gotos (for example: AXIS1_MICROSTEPS_GOTO 2)\n")
    else:
        config_file.write("#define AXIS1_MICROSTEPS_GOTO "+dico["slew1"]+"   // Axis1 (RA/Azm):  Default _OFF, Optional microstep mode used during gotos (for example: AXIS1_MICROSTEPS_GOTO 2)\n")
    
    config_file.write("#define AXIS2_DRIVER_MODEL "+driver2_mod+"      // Axis2 (Dec/Alt): Default _OFF, Stepper driver model (see above)\n")
    
    if dico["micro2"] == "OFF":
        config_file.write("#define AXIS2_MICROSTEPS_"+dico["micro2"]+"        // Axis2 (Dec/Alt): Default _OFF, Microstep mode when the scope is doing sidereal tracking\n")
    else:
        config_file.write("#define AXIS2_MICROSTEPS "+dico["micro2"]+"        // Axis2 (Dec/Alt): Default _OFF, Microstep mode when the scope is doing sidereal tracking\n")
        
    if dico["slew2"] == "OFF":
        config_file.write("#define AXIS2_MICROSTEPS_GOTO_"+dico["slew2"]+"   // Axis2 (Dec/Alt): Default _OFF, Optional microstep mode used during gotos\n")
    else:
        config_file.write("#define AXIS2_MICROSTEPS_GOTO "+dico["slew2"]+"   // Axis2 (Dec/Alt): Default _OFF, Optional microstep mode used during gotos\n")
        
    config_file.write("// Note: you can replace this section with the contents of \"AdvancedStepperSetup.txt\" . . . . . . . . . . . . . . . . . . . \n")
    config_file.write("\n")
    config_file.write("// Stepper driver Fault detection on Pins 18 (Aux1) and 5 (Aux2,) choose only one feature to use on Aux1/2.  The SPI interface (on M0/M1/M2/Aux) can be used to detect errors on the TMC2130.\n")
    config_file.write("// other settings are LOW, HIGH, TMC2130 (if available applies internal pullup if LOW and pulldown if HIGH.)\n")
    if dico["fault1"] == "OFF":
        config_file.write("#define AXIS1_FAULT_"+dico["fault1"]+"\n")
    else:
        config_file.write("#define AXIS1_FAULT "+dico["fault1"]+"\n")
                          
    if dico["fault2"] == "OFF":
        config_file.write("#define AXIS2_FAULT_"+dico["fault2"]+"\n")
    else:
        config_file.write("#define AXIS2_FAULT_"+dico["fault2"]+"\n")
                          
    config_file.write("\n")
    
    #==========================================================================
    #==========================================================================
    # Axis 3, 4 et 5 (rot, focus1 and focus2)
    #==========================================================================
    #==========================================================================
    
    if board != "Mega2560Alt":
        config_file.write("// ------------------------------------------------------------------------------------------------------------------------\n")
        if board == "MiniPCB":
            config_file.write("// THE FOLLOWING ARE INFREQUENTLY USED OPTIONS FOR THE MINIPCB SINCE USING ANY OF THESE WOULD REQUIRE SOLDERING TO THE PCB BACK AND ADDING OFF-PCB CIRCUITRY, MUCH EASIER TO USE A MAXPCB AND TEENSY3.5/3.6\n")
        config_file.write("// CAMERA ROTATOR OR ALT/AZ DE-ROTATION ----------------------------------------------------------------------------------\n")
        if board != "TM4C":
            if board != "ramps14":
                config_file.write("// Pins 30,33 = Step,Dir (choose either this option or the second focuser, not both)\n")
            else:
                config_file.write("// Pins 30,33 = Step,Dir\n")
        else:
            config_file.write("// Pins 30,33 = Step,Dir (choose either this option or the first focuser, not both)\n")
            
        config_file.write("#define ROTATOR_"+rotator+"                  // use _ON to enable the rotator (for any mount type,) default=_OFF (this is also a de-rotator for MOUNT_TYPE_ALTAZM mounts.)\n")
        config_file.write("#define MaxRateAxis3               "+str(dico["rot_rate"])+" // this is the minimum number of milli-seconds between micro-steps, default=8\n")
        config_file.write("#define StepsPerDegreeAxis3     "+str(step_degre_rot)+" // calculated as    :  stepper_steps * micro_steps * gear_reduction1 * (gear_reduction2/360)\n")
        config_file.write("                                     // Rotator          :  24            * 8           * 20              *  6/360                = 64\n")
        config_file.write("                                     // For de-rotation of Alt/Az mounts a quick estimate of the required resolution (in StepsPerDegree)\n")
        config_file.write("                                     // would be an estimate of the circumference of the useful imaging circle in (pixels * 2)/360\n")
        config_file.write("#define MinAxis3                "+str(dico["rot_min_degr"])+" // minimum allowed Axis3 rotator, default = -180\n")
        config_file.write("#define MaxAxis3                 "+str(dico["rot_max_degr"])+" // maximum allowed Axis3 rotator, default =  180\n")
        config_file.write("#define AXIS3_REVERSE_"+rot_rev+"            // reverse the direction of Axis3 rotator movement\n")
        
        if ((board != "Classic") and (board != "MiniPCB")):
            if dico["rot_disable"] == "OFF":
                config_file.write("#define AXIS3_DISABLE_"+dico["rot_disable"]+"            // Pin 36 (Aux3.)  Default _OFF, use HIGH for common stepper drivers.\n")
            else:
                config_file.write("#define AXIS3_DISABLE "+dico["rot_disable"]+"            // Pin 36 (Aux3.)  Default _OFF, use HIGH for common stepper drivers.\n")
            config_file.write("#define AXIS3_AUTO_POWER_DOWN_"+dico["rot_autodi"]+"    // use _ON if you want to power down the motor at stand-still.  Default _OFF.\n")
        config_file.write("\n")
        
        
        config_file.write("// FOCUSER1 ---------------------------------------------------------------------------------------------------------------\n")
        if board != "TM4C":
            config_file.write("// Pins 34,35 = Step,Dir\n")
        else:
            config_file.write("// Pins (see pinmap) = Step,Dir (choose either this option or the rotator, not both)\n")
        config_file.write("#define FOCUSER1_"+focus1+"                 // use _ON to enable this focuser, default=_OFF\n")
        config_file.write("#define MaxRateAxis4               "+str(dico["foc1_rate"])+" // this is the minimum number of milli-seconds between micro-steps, default=8\n")
        config_file.write("#define StepsPerMicrometerAxis4  "+str(dico["foc1_ratio"])+" // figure this out by testing or other means\n")
        config_file.write("#define MinAxis4               "+str(dico["foc1_min_mm"])+" // minimum allowed Axis4 position in millimeters, default = -25.0\n")
        config_file.write("#define MaxAxis4                "+str(dico["foc1_max_mm"])+" // maximum allowed Axis4 position in millimeters, default =  25.0\n")
        config_file.write("#define AXIS4_REVERSE_"+foc1_rev+"            // reverse the direction of Axis4 focuser movement\n")          
                          
        if ((board != "Classic") and (board != "MiniPCB")):
            if dico["foc1_disable"] == "OFF":
                config_file.write("#define AXIS4_DISABLE_"+dico["foc1_disable"]+"            // Pin 39 (Aux4.)  Default _OFF, use HIGH for common stepper drivers.\n")
            else:                      
                config_file.write("#define AXIS4_DISABLE "+dico["foc1_disable"]+"            // Pin 39 (Aux4.)  Default _OFF, use HIGH for common stepper drivers.\n")
            config_file.write("#define AXIS4_AUTO_POWER_DOWN_"+dico["foc1_autodi"]+"    // use _ON if you want to power down the motor at stand-still.  Default _OFF.\n")
        config_file.write("\n")
            
        
        if ((board != "TM4C") and (board != "STM32")):
            config_file.write("// FOCUSER2 ---------------------------------------------------------------------------------------------------------------\n")
            if board != "ramps14":
                config_file.write("// Pins 30,33 = Step,Dir (choose either this option or the rotator, not both) \n")
            else:
                config_file.write("// Pins 30,33 = Step,Dir \n")
                
            config_file.write("#define FOCUSER2_"+focus2+"                 // use _ON to enable this focuser, default=_OFF\n")
            config_file.write("#define MaxRateAxis5               "+str(dico["foc2_rate"])+" // this is the minimum number of milli-seconds between micro-steps, default=8\n")
            config_file.write("#define StepsPerMicrometerAxis5  "+str(dico["foc2_ratio"])+" // figure this out by testing or other means\n")
            config_file.write("#define MinAxis5               "+str(dico["foc2_min_mm"])+" // minimum allowed Axis5 position in millimeters, default = -25.0\n")
            config_file.write("#define MaxAxis5                "+str(dico["foc2_max_mm"])+" // maximum allowed Axis5 position in millimeters, default =  25.0\n")
            config_file.write("#define AXIS5_REVERSE_"+foc2_rev+"            // reverse the direction of Axis5 focuser movement\n")
            
            if ((board != "Classic") and (board != "MiniPCB")):
                if dico["foc2_disable"] == "OFF":                  
                    config_file.write("#define AXIS5_DISABLE_"+dico["foc2_disable"]+"            // Pin 36 (Aux3.)  Default _OFF, use HIGH for common stepper drivers.\n")
                else:
                    config_file.write("#define AXIS5_DISABLE "+dico["foc2_disable"]+"            // Pin 36 (Aux3.)  Default _OFF, use HIGH for common stepper drivers.\n")
                config_file.write("#define AXIS5_AUTO_POWER_DOWN_"+dico["foc2_autodi"]+"            // use _ON if you want to power down the motor at stand-still.  Default _OFF.\n")                  
            config_file.write("\n")
            
    #==========================================================================
    #==========================================================================
    # END
    #==========================================================================
    #==========================================================================
    
    
    config_file.write("// THAT'S IT FOR USER CONFIGURATION!\n")
    config_file.write("\n")
    config_file.write("// -------------------------------------------------------------------------------------------------------------------------\n")
    config_file.write("#define FileVersionConfig 2\n")
    config_file.write("#include \"src/pinmaps/Pins."+board+".h\"\n")
    config_file.write("#endif\n")
    
    config_file.close()
