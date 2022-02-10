/***********************************************************************************************************
 *  SIGNAL.H     - Parameters & Constants   -NOTES at the Bottom
 *  
 ***********************************************************************************************************/

                                          /////////////////////////////////////////////////////////////////
                                          ////  GENERAL  CONSTANTS  &  PARAMETERS
                                          /////////////////////////////////////////////////////////////////
#define FIRST_DIG_PIN               2
#define LAST_DIG_PIN                53                                     
#define ONBOARD_LED_PIN             13  
#define LAST_LWR_DIG_PIN            13
#define FIRST_UPR_DIG_PIN           22
#define NUM_DIG_IO                  54            // array size, even tho [0],[1] never used
#define NUM_AI                      16  

#define PRINT                       Serial.print
#define PRINTLN                     Serial.println
#define prtEQ()                     Serial.print("=") 
#define OR                          ||
#define AND                         &&

#define WRKSTR_LEN                  40
#define MAX_FLD_LEN                 10
#define RULESTR_LEN                 80    // must be long enough for a RULE String
#define CMDSTR_LEN                  100   // must be long enough for a RULE String + command words
#define ID_STR_LEN                  16
#define NAME_LEN                    12
#define TAGNAME_LEN                 18
#define NETNAME_LEN                 32
#define EXPLSTR_LEN                 20
#define INTERFACE_BFR_LEN           74  
                                          /////////////////////////////////////////////////////////////////
                                          ////  APP SPECIFIC PARAMETERS _ SIGNAL ASPECTS
                                          /////////////////////////////////////////////////////////////////  
#define NUM_BLOCKS                  4     // Max # = 4: Left & Right on Double Main
#define MAX_BLOCK_TURNOUTS          2     // allow up to 2 Turnouts onto main within a block for a passing siding
#define NUM_DETECTORS               6     // End Point for Every Block. Adjacent Blocks share
#define NUM_SENSORS                 12
#define NUM_SIGNALS                 (NUM_BLOCKS+NUM_BLOCKS) 

#define NUM_TAGS                    10
#define BOOL_TAG_TYP                0x01 
#define TIMER_TAG_TYP               0x02 
#define INT_TAG_TYP                 0x04 

#define LF_INPUT_NDX                0
#define RT_INPUT_NDX                1

#define MIN_SENSOR_DEADBAND         250     // ms
#define MAX_SENSOR_DEADBAND         1500    // ms
#define DFLT_SENSOR_DEADBAND        750

#define MAX_VALID_MILEPOST          999.9

#define BLOCK_UNOCC                 0
#define BLOCK_OCC_LF_RT             1
#define BLOCK_OCC_RT_LF             0x10
#define DFLT_BLOCK_TO_SEC           120
#define MIN_BLOCK_TO_SEC            60
#define MAX_BLOCK_TO_SEC            600 
#define OPEN_ENDED_BLK_TO           15      // sec

#define TURNOUT_FACING_ACTV_STS     1
#define TURNOUT_RVS_ACTV_STS        2

#define MAX_ENTRY_PERMIT_LVL        3
#define MED_ENTRY_PERMIT_LVL        2         // TURNOUT IN NEXT BLOCK
#define LOW_ENTRY_PERMIT_LVL        1         // TURNOUT IN NEXT BLOCK
#define NO_ENTRY_PERMIT_LVL         0
#define UNKNOWN_PERMIT_LVL          99        // NEXT BLOCK ENTRY PERMIT LEVELS ARE UNKNOWN

#define MAINLINE_TRACK_TYP          0
#define SIDING_TRACK_TYP            1
#define SHORT_SIDING_TRACK_TYP      2

#define DETECTOR_UNOCC              0x00
#define DETECT_ENTER_LFT            0x01
#define DETECTOR_OCC_LFT_RT         0x03
#define DETECTOR_EXIT_LFT           0x02
#define DETECT_ENTER_RT             0x10
#define DETECTOR_OCC_RT_LFT         0x30 
#define DETECTOR_EXIT_RT            0x20
#define DETECTOR_OCC_DIR_UNKNOWN    0x33 

#define SIGNAL_RGY_TYP              0x00
#define SIGNAL_RG_ONLY_OPT          0x01
#define SIGNAL_RG_BIPOLAR_OPT       0x02
#define SIGNAL_HEAD_PIN_OFFSET      3    

#define RED_NDX                     0       // Colour Value Lookup Indexes
#define GRN_NDX                     1
#define YLW_NDX                     2   

#define RED                         1
#define GRN                         2
#define YLW                         4
#define FLASH                       8

#define FLIP_LIST_LEN               5     // number bipolar LEDS that can be YELLOW at same time

      // Add 400 to these (Byte) constants to get CROR RULE NUMBERS. SO CLEAR = Rule 405 = 5 + 400. 
#define CLEAR                        5     // GRN, RED, RED - Proceed
#define CLEAR_TO_LIMITED             6     // YLW, GRN-FL, RED - Proceed approached next at LIMITED
#define CLEAR_TO_MEDIUM              7     // YLW, GRN, RED - Proceed approached next at MEDIUM
#define CLEAR_TO_SLOW                9     // YLW, YLW, RED - Proceed approaching next at DIVERGING speed
#define CLEAR_TO_STOP               11     // YLW, RED, RED -Proceed preparing to stop
#define ADVANCE_CLEAR_TO_MEDIUM     13     // YLW-FL, GRN , RED
#define ADVANCE_CLEAR_TO_SLOW       14     // YLW-FL, YLW, RED
#define ADVANCE_CLEAR_TO_STOP       15     // YLW-FL, RED, RED
#define SLOW_TO_CLEAR               31     // RED, GRN, or RED, RED, GRN - Proceed Slow thru Turnout
#define SLOW_TO_STOP                35     // RED, YLW-FL or RED,RED,YLW-FL
#define RESTRICTING                 36     // RED, YLW or RED,RED,YLW
#define STOP_SIGNAL                 39     // RED, RED, RED 
#define SIG_OFF_ASPECT              99     // SIGNAL OFF
 
#define TOP_PSN_NDX                 0
#define BELOW_TOP_PSN_NDX           1
#define BTM_3_PSN_NDX               2                         

#define NUM_STATS                   4
#define DETECT_EVENT_STAT_X         0
                                          ////////////////////////////////////////////////////////////
                                          // NETWORK INTERFACE
                                          ////////////////////////////////////////////////////////////
#define OK_STS_EXPECTED             1
#define UNKNOWN_REPLY_EXPECTED      2
#define CONNECT_STS_EXPECTED        4
#define NETWORK_MODE_CHG            8

#define TEST_NET_CONNECT_STS        16
#define TAG_UPDATE_ACTN             32
                                          ////////////////////////////////////////////////////////////
                                          //  TAGS
                                          ////////////////////////////////////////////////////////////
#define INT_TAG_TYP                 0
#define FLOAT_TAG_TYP               1
                                          ////////////////////////////////////////////////////////////
                                          //  FLAG (Semaphore) HEX VALUES - 1 bit per unsigned value
                                          ////////////////////////////////////////////////////////////
#define SPARE1_FLAG                 0x0001
#define SPARE2_FLAG                 0x0002
#define SPARE3_FLAG                 0x0004
#define SPARE4_FLAG                 0x0008
#define SPARE5_FLAG                 0x0010
#define FWD_CMD_RDY_FLAG            0x0020
#define SPARE6_FLAG                 0x0040
#define SIGNAL_TST_FLAG             0x0080
#define SIMULATION_ACTV_FLAG        0x0100
#define SAVE_CFG_FLAG               0x0200
#define VERBOSE_FLAG                0x0400
#define MONITOR_INPUT_FLAG          0x0800
#define SHOW_STS_FLAG               0x1000
#define SHOW_CFG_FLAG               0x2000
#define SHOW_HLP_FLAG               0x4000
#define DEBUG_FLAG                  0x8000
                                          ////////////////////////////////////////////////////////////
                                          //  TIMING  PARAMS
                                          ////////////////////////////////////////////////////////////
#define NUM_TIMERS                  3
#define SENS_TMRX                   0       // index   
#define BLOCK_TMRX                  1       // index   
#define SIM_TMRX                    2

#define SENS_PACE                   4       // ms   
#define BLOCK_OCC_PACE              250     // ms   
#define NUM_ET_TIMERS               0                                
                                          ////////////////////////////////////////////////////////////
                                          ////  CONFIGURATION EEPROM PARAMS
                                          ////////////////////////////////////////////////////////////
#define SENSOR_EEPROM_ADDR          0     //  168
#define DETECTORS_EEPROM_ADDR       300   //  49
#define BLOCKS_EEPROM_ADDR          400   //  241 
#define SIGNALS_EEPROM_ADDR         700   //  32
#define NETWORK_CFG_EEPROM_ADDR     800    
#define CFG_INTEGRITY_CHK_VAL       43   // arbitrary number - to match when reading back
                                          ////////////////////////////////////////////////////////////
                                          ////  PARSING PARAMS
                                          ////////////////////////////////////////////////////////////
#define UPPR_CASE           1
#define INCL_DEC_PT_OPT     2
#define PASS_FLT_FLD_OPT    4   // Treats 123.45 as single field as it passes it looking for following flds
#define ALLOW_SLASH_OPT     8   // Example  km/hr
