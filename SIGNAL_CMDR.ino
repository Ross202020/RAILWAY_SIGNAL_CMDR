/**********************************************************************************
 *  SIGNAL_CMDR     -Railway Signal Commander senses Train Movements, 
 *                    Arduino MEGA Platform   Tools > Board > Arduino AVR Boards > Arduino Mega 2560
 *                    Uses about 9% of Pgm Mem, 16% of Dynamic. This is a Plug in and Go tool
 *                    >>On Serial Monitor just press <Enter> to get your initial Help - ther is lots there
 *                      Then if you need more, type HELP ADD or any other command 
 *                      
 *  >> Reads Aanalog SENSORS to Detect Trains. These are assumed to be UV or Photo with proper Resistor to Vcc
 *    (more detail in Help)
 *    
 *  You Configure DETECTORS. Each Detector has a Pair of Sensors physically separated by about 3 cm so it can detect
 *   Train with Direction. A Compass Direction is associated with teh Detector Left vs Right
 *   
 *  You Configure up to 4 BLOCKs. A Block has a Detector at Each End. Block Logic first determines if the BLOCK is OCCUPIED
 *   based on a Train Entering. It remains Occupied until it sees an EXIT. A Compass Direction is associated with 
 *   the BLOCK so it logically understands which way the train is going, and which way the Last Train Exited.
 *   After Occupancy has been determined as UNOCC Then it sets the ENTRY PERMISSIVES for each end
 *   You can Add Internal Turnouts, which will modify Entry Permissive Values.
 *  There are Hooks in place to support Networking to link distant BLOCKS to exchange ENTRY PERMISSIVES, 
 *    but the actual Networking part has not been applied. The TAGS module will be the 'Post Office' for this 
 *    exchange if implemented. ->I use a simple ESP01 to get onto the network and publish Tags, 1 Tag + Value per line.
 *    The ESP app also goes  out and reads other Web Pages to fetch their data. Way simpler IoT mechanism than MQTT or other UDP, 
 *    and you can even watch it happen on a PC to debug. Let me know if you need that software, work great with Arduino
 *   
 *  SIGNALS are associated with a BLOCK ENTRY and have Aspects selected by the ENTRY Permissive Value of that Block
 *  Each Signal can have 1,2,or 3 Heads (Lamps) and each has 3 outputs (R,G,Y) reserved for it in consecutive DigOuts
 *  THe MEGA is a beautiful beast, but even this could run out of Outputs if you go to a full compliment of Signals.
 *  But Outputs are not reserved if unused, so if your signal heads are single or double you will not reserve 9 
 *  consecutive outputs. In fact there is no assignment mapping so if you don't plan your pin assignment then more than
 *  1 action could map to the same output pin.
 *  
 *  The Vocabulary mecahnism is crude but cures compiler issues and is very efficient
 *  
 *  Newer (2022) version of BMRC (Belleville Model Railroad Club) SIGNAL_CMDR, Programmer: RossCameron.ca@gmail.com
 *  
 *       NOTE: Make use of byte types wherever possible because this is an 8-bit machine so byte integer
 *             operations are way more efficient (single fetch) than unsigned (16-bit) operations that
 *             require multiple fetches and other steps to handle every integer, even to test for 0
 *                    
 *  Feb04 0300  -Have most of the boiler plate done, just need to add Rule Logic to make it all work
 *       Basic configuration seems to work well with a small footprint
 *         >>Sketch uses 15056 bytes (5%) of PgmMem, Globals 1308 bytes (15%) of DynMem.
 *  Feb5  0957 -More changes to DETECTOR & BLOCK Configuration -check Block again for getting correct detector indexes
 *  Feb6 1250   -More improvements - simplified SENSOR actions, Added DEADBAND to Sensors, Paced to 4 ms
 *  1.04 6FE22  -Simulation works ! It found a bug in DETECTOR Logic as Train Leaves Detector, but that is for
 *                 another day. Quite pleased with the performance
 * Feb 7  -Fixed up DETECTORS, dflt=4 Sensors. Tested with SIM and that works Well
 * Feb 7  1.07 Have Have completed up to BLOCKS for OCCUPANCY, Tested with the SIMULATION Feature - Works Well
 * FE0722 1.1   -WORKING !! Initial Bench Test with Simulation (START SIM) exercised SENSORS, DETECTORS, BLOCKS, & SIGNALS  
 *               and all worked well
 * Feb08  1.2   -Added Use of TAGS for exporting of BLOCK PERMIT Values, BLOCK OCC TIMEOUTS,
 *               Added OPEN ENDED BLOCKS for Spur or End of signaled territory, reordered args to getFld() to put fld num first
 * 1.4 9FE22    -Rmvd WEB SERVER INTERFACE ->Caused Hanging and was not required
 **********************************************************************************/
#include <EEPROM.h>
#include "SIGNAL.H" 

char cmdStr[CMDSTR_LEN+1], ch, expl[EXPLSTR_LEN+1], wrkStr[WRKSTR_LEN+1];
byte cmdStrLen, wrkStrLen, monINPUT, monSec, flipList[FLIP_LIST_LEN], flipSts, pinX, tagUpdtX,
    simState, simSensX, simSensX2, simDetX, simRL,
    sigTstET, sigTstNdx, flipYlwCnt, trapFlag;
unsigned gbSensorSts[NUM_SENSORS],        // Made Global for hi-speed efficieny and access by multiple modules
    simCnt,  sigTstAsp; 
byte gbDetectSts[NUM_DETECTORS];         // Made Global for hi-speed efficieny and access by multiple modules

/*******************************************************/
void prtAppNam() { PRINT(F("SIGNAL_CMDR "));}
void prtAppRev() { PRINT(F("1.4 ")); }

void setup() 
{
  Serial.begin(115200); delay(750); while(!Serial) delay(100);
  prtRtn(); prtAppNam();  prtAppRev(); prtRtn();
  setFlag(VERBOSE_FLAG);
  setFlag(SHOW_HLP_FLAG);
  setFlag(SHOW_STS_FLAG);
  setFlag(SHOW_CFG_FLAG);
  initSensors();
  initDetectors();
  initBlocks();
  initSignals();
  for(byte x=0; x<FLIP_LIST_LEN ;x++) flipList[x]=0;
}

/*******************************************************/
void loop() 
{
static unsigned long sysTick, nowTime, lastRdTime, stsLEDtime;  static byte heartBeat, newms, detectNxt;
  nowTime = millis();
  if (sysTick != millis())      // START BY MANAGING TIME  1 ms at a TIME
  {
    if(sysTick > nowTime)       // Detect milis() Overflow (wrap)
    { sysTick = nowTime; clockTick(); }
    else    // sysTick < nowTime   // CATCH UP TO CURR TIME - 50 max this pass
    { for(newms=1; newms<50 && sysTick<nowTime ;sysTick++, newms++) { clockTick(); } }   
    newms=1;
  } else newms=0;

        ///////////////////////////////////////////////////////////
        //  DO 1 TASK ONLY on every Pass .AND. ONLY on newms >>Keeps Time Synchronized 
        //  the else if(  is what keeps it to 1 task/ pass
        ///////////////////////////////////////////////////////////
  if(newms)
  {
                ////////////////////////////////////////////////////////////////
                // DECR ALL SENSOR DEADBAND COUNTS EVERY ms
                ////////////////////////////////////////////////////////////////
    for(byte x=0; x<NUM_SENSORS ;x++)          
    { if(gbSensorSts[x]) --gbSensorSts[x];
    }
                ////////////////////////////////////////////////////////////////
                //  TOGGLE BIPOLAR LED SIGNAL on Consecutive Outputs EVERY ms 
                //   TO MAKE THEM YELLOW. Their presence in this list is managed by SIGNALS 
                ////////////////////////////////////////////////////////////////
    if(flipYlwCnt)
    {
      flipSts = flipSts ? 0:1;
      for(byte x=0; x<FLIP_LIST_LEN ;x++)
      {
        pinX = flipList[x];
        if(pinX>=FIRST_DIG_PIN)
        {
          digitalWrite(pinX++,flipSts);
          digitalWrite(pinX,flipSts ? 0:1);
        }
      }
    }    
                ////////////////////////////////////////////////////////////////
                // RE-READ SENSORS at 4 ms PACE. doDetect next ms After
                ////////////////////////////////////////////////////////////////
    if(timerExpired(SENS_TMRX))     // TIME TO RE-READ SENSORS
    {
      readSensors();                // Read all Sensors  
      detectNxt = 1;                      // Run Detectors next ms at same pace
      startTimer(SENS_TMRX, SENS_PACE);   // restart SENS_TMRX  (4 ms)
    }
    else if(detectNxt)              // RUN DETECTORS LOGIC in ms immediately following Sensor Read
    {                               //  to catch Train Edges at Sensor Pair
      if(doDetect())
      {  //stat[DETECT_EVENT_STAT_X]++;
      }
      detectNxt = 0;                // cancel  detectNxt
    }
                ////////////////////////////////////////////////////////////////
                //  Update BLOCK Status every 250 ms - Set Entry Permits + Signal Aspect
                ////////////////////////////////////////////////////////////////
    else if(timerExpired(BLOCK_TMRX))   // TIME TO MAKE OCCUPANCY & SIGNAL DECISIONS - NOT TOO FAST - 250 ms should be plenty fast
    {
      setBlockOccSts();             // Set Block Status according to its Detectors at each end

      setBlockEntrySts();           // SET ENTRY PERMITS & SIGNAL ASPECTS 
      
      driveSignals();               // UPDATE SIGNAL OUTPUTS
      
      startTimer(BLOCK_TMRX, BLOCK_OCC_PACE);    // Restart Pace Timer
    } 
                ////////////////////////////////////////////////////////////////
                //  SIMULATE Train Passing a Detector by hitting 
                //   Sensor statuses. Downstream Logic handles the rest
                ////////////////////////////////////////////////////////////////
    else if(tstFlag(SIMULATION_ACTV_FLAG) && timerExpired(SIM_TMRX))
    {
      switch(simState)
      {
        case 0:         // ACTIVATE FIRST SENSOR FOR THIS DETECTOR
          simSensX = getDetectorSensNdx(simDetX, simRL);
          if(simSensX<NUM_SENSORS)
          {
            setSensorActv(simSensX);
            ++simState;
            simCnt = 0;
          }
          else clrFlag(SIMULATION_ACTV_FLAG);
          break;
        case 1:       // ACTIVATE 2nd SENSOR, KEEP FIRST ACTIVE
          simSensX2 = getDetectorSensNdx(simDetX, simRL ? 0:1);
          if(simSensX2<NUM_SENSORS)
          {
            setSensorActv(simSensX2);
            setSensorActv(simSensX);
            ++simState;   simCnt=0;
          }
          else clrFlag(SIMULATION_ACTV_FLAG);
          break;
          
        case 2:       // KEEP BOTH DETECTOR SENSORS ACTIVE FOR A WHILE (10 sec)
          setSensorActv(simSensX2);
          setSensorActv(simSensX);
          if(++simCnt>=200)          // after 10 Sec move on to next State
          {
            ++simState;
            simCnt = 0;
          }
          break;

        case 3:       // TRAIN BEGINS EXIT - DROP INITIAL SENSOR KEEP 2nd going for about 500 ms
          setSensorActv(simSensX2);
          if(++simCnt>=10)
          {
            ++simState;
            simCnt = 0;
          }
          break;
        case 4:     // TRAIN HAS EXITED THIS DETECTOR - SIMULATION OVER
          clrFlag(SIMULATION_ACTV_FLAG);
          PRINTLN(F("End SIM TEST"));
          ++simState;
          break;
          
        default:
          break;
      }
      startTimer(SIM_TMRX, 50);   // restart 50 ms Delay
    }

                ////////////////////////////////////////////////////////////////
                //  READ NEXT INCOMING DEBUG ch Every ms - this allows time
                //   for all to arrive, as they arrive slower than this
                //   computer can read them
                ////////////////////////////////////////////////////////////////
    else if(Serial.available())
    {
      ch = Serial.read();
      if(ch >= ' ')         // valid char
      {
        if(cmdStrLen || ch>' ')           // APPEND NEXT CHAR - Discard leading SP
        {
          if(cmdStrLen<CMDSTR_LEN) 
            cmdStr[cmdStrLen++] = ch;
        }
        cmdStr[cmdStrLen] = 0;
        lastRdTime = millis();
      }
      else    // non-printable char
      {
        if(!cmdStrLen && (millis()-lastRdTime) > 1000) { setFlag(SHOW_STS_FLAG + SHOW_HLP_FLAG); }
      }
    }
    else if(cmdStrLen && millis()-lastRdTime > 2)   // CMD STRING HAS STOPPED ARRIVING - PROCESS IT NOW
    {
      processCmd(cmdStr);
      cmdStrLen = 0;
    }    
                ////////////////////////////////////////////////////////////////
                //  DO Low Priority Housekeeping Task if did not do 
                //   any other Tasks above
                ////////////////////////////////////////////////////////////////
    else if (tstFlag(SHOW_HLP_FLAG)) { clrFlag(SHOW_HLP_FLAG);  showHelp(); }
    else if (tstFlag(SHOW_STS_FLAG)) { clrFlag(SHOW_STS_FLAG);  showSts();  }  
    else if (tstFlag(SHOW_CFG_FLAG)) { clrFlag(SHOW_CFG_FLAG);  showCfg();  }
    else if (tstFlag(SAVE_CFG_FLAG)) 
    { 
      clrFlag(SAVE_CFG_FLAG);  
      saveSensorCfg(); saveDetectorCfg(); saveBlocksCfg(); saveSignalsCfg(); 
    }
    else if(tstFlag(MONITOR_INPUT_FLAG) && monSec!=getClockSec())   // DEBUG Feature - monitor a single output every sec
    {
      if(monINPUT>100) { PRINT(F("DI"));prtNdx(monINPUT-100);prtEQ();PRINTLN(digitalRead(monINPUT-100)); }
      else if(monINPUT<=15) { PRINT(F("AI"));prtNdx(monINPUT);prtEQ();PRINTLN(analogRead(monINPUT)); }
      monSec=getClockSec();
    }
          ////  OUTPUT HEARTBEAT
    else if(millis()-stsLEDtime >= 250)       // 250 ms have passed since last LED Update
    {
      pinMode(ONBOARD_LED_PIN, OUTPUT); digitalWrite(ONBOARD_LED_PIN,heartBeat);
      heartBeat = heartBeat ? 0:1; stsLEDtime=millis();
    }
  }
}

/*******************************************************/
byte validDigPin(byte pin) { return (pin>=FIRST_DIG_PIN && (pin<13 || pin>=22) && pin<=LAST_DIG_PIN); }
