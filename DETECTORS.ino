/****************************************************************************************
 *  DETECTORS  Module   -Every Detector has a Left Sensor, Right Sensor, Milepost
 ****************************************************************************************/
#ifndef NUM_DETECTORS
#define NUM_DETECTORS         9     // Every Block has Left and Right Detector, 8 Adjacent Blocks require 9 Detectors
#endif

struct DETECTOR {
  byte sensorNdx[2],                // [0]==Left Sensor, [1]==Right
       leftCompass;                  
  float milePost;                // Mile Post Value X 100
} detectorDefn[NUM_DETECTORS]; 
char lastTrainDir[NUM_DETECTORS]; 
unsigned detectEventCnt[NUM_DETECTORS];
byte chgCnt, lstSts, newSts, stsLft, stsRt, onFlags;

/************************************************************
 *  doDetect()      -Call very rapidly (LTE 5 ms) to Detect Trains Passing Sensor
 *                  -Rtns Motion Event Count
 ***********************************************************/
byte doDetect()
{
  chgCnt=0;
  for(byte dx=0; dx<NUM_DETECTORS ;dx++)
  {
    if(detectorDefn[dx].leftCompass)    // Test for Configured DETECTOR
    {
      onFlags=0; 
      lstSts = gbDetectSts[dx]; 
      newSts=0;
      stsLft = gbSensorSts[detectorDefn[dx].sensorNdx[LF_INPUT_NDX]];
      stsRt  = gbSensorSts[detectorDefn[dx].sensorNdx[RT_INPUT_NDX]];
      
      if(stsLft)          // LEFT SENSOR IS OCCUPIED AT THIS MOMENT
      {  
        onFlags = 1;     
        if(stsRt)         // BOTH ARE OCCUPIED AT THIS MOMENT
        {  onFlags = 3;
          if(lstSts==DETECTOR_OCC_LFT_RT)       { newSts = lstSts; }
          else if(lstSts==DETECT_ENTER_LFT)     { newSts = DETECTOR_OCC_LFT_RT; }
          else if(lstSts==DETECT_ENTER_RT)      { newSts = DETECTOR_OCC_RT_LFT; }
          else if(lstSts==DETECTOR_OCC_RT_LFT)  { newSts = DETECTOR_OCC_RT_LFT; }
          else                                  { newSts = DETECTOR_OCC_DIR_UNKNOWN; }
        }
        else                    // ONLY LEFT SENSOR IS ACTIVE AT THIS MOMENT
        { 
          if(lstSts==DETECT_ENTER_LFT  || lstSts==DETECTOR_EXIT_LFT)    // No Change
          { newSts = lstSts;
          }
          else if(lstSts==DETECTOR_UNOCC)               // BOTH UNOCCUPIED LAST TIME - must be new entry
          { newSts = DETECT_ENTER_LFT;
          }
          else if(lstSts==DETECTOR_OCC_LFT_RT || lstSts==DETECTOR_OCC_RT_LFT || lstSts==DETECTOR_OCC_DIR_UNKNOWN)
          { newSts = DETECTOR_EXIT_LFT;
          }
        }
      }
      else if(stsRt)        // ONLY RT SENSOR IS OCCUPIED AT THIS MOMENT
      { 
        onFlags=2; 
        if(lstSts==DETECTOR_OCC_LFT_RT || lstSts==DETECTOR_OCC_RT_LFT || lstSts==DETECTOR_OCC_DIR_UNKNOWN)
        { newSts = DETECTOR_EXIT_RT;
        }
        else if(lstSts==DETECTOR_UNOCC)                 // BOTH UNOCCUPIED LAST TIME - new Entry
        { newSts = DETECT_ENTER_RT;
        }
        else if(lstSts==DETECTOR_EXIT_RT || lstSts==DETECT_ENTER_RT)
        { newSts = lstSts;
        }
      }
      else      // Neither Sensor is occupied at this moment
      { newSts = DETECTOR_UNOCC;
      }
  
                // DETECT CHANGE, UPDATE DETECTOR STATUS
      if(newSts != lstSts) 
      {
        gbDetectSts[dx] = newSts;       // UPDATE GLOBAL   gbDetectSts[dx]
                    ////  SET COMPASS DIRECTION FOR THIS TRAIN
        if(newSts==DETECT_ENTER_LFT || newSts==DETECTOR_OCC_LFT_RT || newSts==DETECTOR_EXIT_RT)
        {
          if(detectorDefn[dx].leftCompass=='W') lastTrainDir[dx]='E';
          else if(detectorDefn[dx].leftCompass=='E') lastTrainDir[dx]='W';
          else if(detectorDefn[dx].leftCompass=='N') lastTrainDir[dx]='S';
          else lastTrainDir[dx]='N';
        }
        else if(newSts==DETECT_ENTER_RT || newSts==DETECTOR_OCC_RT_LFT || newSts==DETECTOR_EXIT_LFT)
        {     // TRAIN MOVING TOWARDS LEFT
          lastTrainDir[dx] = detectorDefn[dx].leftCompass; 
        }
        ++chgCnt;   ++detectEventCnt[dx];
        if(tstFlag(VERBOSE_FLAG)) 
        { 
          PRINT(F("DETECTOR"));prtNdx(dx);prtSP(); PRINT(getDetectStsExpl(dx));prtSP();
            PRINT(F(" ON_flags>"));if(onFlags<=1)prtCh('0'); PRINT(onFlags,BIN);
            PRINT(F(" STS="));PRINT(newSts);PRINT(F(",Prev="));PRINT(lstSts);prtSP(); PRINTLN(timeStr());
        }
      } 
    }
  }
  return chgCnt;
}

/************************************************************/
byte getDetectorSts(byte detX) { return gbDetectSts[detX]; }

/************************************************************/
char getLastTrainDir(byte detX) { return lastTrainDir[detX]; }

/************************************************************
 *  getDetectStsExpl(byte detX)   -Txt Explanation of Current Detector Status 
 *        For Debug & VERBOSE Monitoring, Translates L-R into Compass Direction
 */
char *getDetectStsExpl(byte detX)
{
byte x=0;  
  switch(gbDetectSts[detX])
  {    
    case DETECT_ENTER_LFT:
    case DETECT_ENTER_RT:
      expl[x++]=lastTrainDir[detX]; expl[x++]='B'; 
      expl[x++]='-';expl[x++]='E';expl[x++]='N';expl[x++]='T';expl[x++]='R';
      break;
    case DETECTOR_OCC_LFT_RT:     
    case DETECTOR_OCC_RT_LFT:
      expl[x++]=lastTrainDir[detX]; expl[x++]='B'; 
      expl[x++]='-';expl[x++]='O';expl[x++]='C';expl[x++]='C';
      break;
    case DETECTOR_EXIT_LFT:
    case DETECTOR_EXIT_RT:
      expl[x++]=lastTrainDir[detX]; expl[x++]='B'; 
      expl[x++]='-';expl[x++]='E';expl[x++]='X';expl[x++]='I';expl[x++]='T'; 
      break; 
    default: case DETECTOR_UNOCC:
      expl[x++]='U';expl[x++]='N';expl[x++]='O';expl[x++]='C';expl[x++]='C';
      break;
  }
  expl[x]=0;
  return expl;
}

/************************************************************/
unsigned getDetectEventCnt(byte dx) { return detectEventCnt[dx]; }

                            //////////////////////////////////////////////////////////////////////////
                            //  CONFIGURATION PARAMETER ACCESS
                            ////////////////////////////////////////////////////////////////////////// 

/************************************************************/
void initDetectors()
{
  for(byte x=0; x<NUM_DETECTORS ;x++) {  detectorDefn[x].leftCompass = 0; }
  readDetectorCfg(); 
  for(byte x=0; x<NUM_DETECTORS ;x++) { gbDetectSts[x]=0; lastTrainDir[x] = '?'; detectEventCnt[x] = 0; }
        // CREATE 2 DETECTORS for Demonstration if None Configured
  if(!detectorDefn[0].leftCompass && !detectorDefn[1].leftCompass)
  {
    addDetector(12.3, 0,1,'W');
    addDetector(29.4, 2,3,'W');
  }
}

/************************************************************
 *  addDetector
 ************************************************************/
byte addDetector(float mileP, byte lftSensX, byte rtSensX, byte compassCh)
{ 
byte dx;
  for(dx=0;dx<NUM_DETECTORS && detectorDefn[dx].leftCompass ;dx++);  // Skip Configured Entries to Find Empty Detector
  if(dx<NUM_DETECTORS) 
  {
    detectorDefn[dx].sensorNdx[LF_INPUT_NDX] = lftSensX;
    detectorDefn[dx].sensorNdx[RT_INPUT_NDX] = rtSensX;
    detectorDefn[dx].milePost = mileP;
    detectorDefn[dx].leftCompass = compassCh; 
    return 1;
  }
  return 0;
}

/************************************************************/
byte delDetector(byte detX)
{
  if(detX<NUM_DETECTORS) { detectorDefn[detX].leftCompass=0; return 1; }
  return 0;
}

/************************************************************/
byte detectorDefined(byte detX) { return (detX<NUM_DETECTORS && detectorDefn[detX].leftCompass); } 

/************************************************************/
float getDetectorMP(byte detX) { return detectorDefn[detX].milePost; }

/************************************************************/
byte getDetectorSensNdx(byte detX, byte rtNdx) 
{
  if(detX<NUM_DETECTORS && detectorDefn[detX].leftCompass)        // Valid Defined Sensor 
    return detectorDefn[detX].sensorNdx[rtNdx ? 1:0];  
  return 255;
} 

/************************************************************/
byte getDetectorOrientation(byte detX) { return detectorDefn[detX].leftCompass; }  

/************************************************************/
byte findDetector(float mp)    // LOOKUP DETECTOR Based on MilePost
{ 
  for(byte dx=0; dx<NUM_DETECTORS ;dx++) { if(detectorDefn[dx].milePost == mp) return dx; }
  return 255;
}

/************************************************************/
byte getNumDetectors() 
{ 
byte dx,cnt=0;
  for(dx=0; dx<NUM_DETECTORS ;dx++)
  { if(detectorDefn[dx].leftCompass) ++cnt;
  }
  return cnt;
}

/************************************************************/
byte getMaxDetectors() { return NUM_DETECTORS; }


/**************************************************
 * saveSensorCfg()      
 **************************************************/ 
void saveDetectorCfg()         
{
byte chk = CFG_INTEGRITY_CHK_VAL;
  EEPROM.write(DETECTORS_EEPROM_ADDR, chk); 
  EEPROM.put((int)(DETECTORS_EEPROM_ADDR+1), detectorDefn); 
  Serial.print(F("DETECTOR CFG SAVED @"));Serial.print(DETECTORS_EEPROM_ADDR);Serial.print(F(" - "));
    Serial.println(sizeof(detectorDefn)+DETECTORS_EEPROM_ADDR);
}

/**************************************************
 * readSensorCfg()      -Rtns 1 if REad OK, else 0
 **************************************************/ 
byte readDetectorCfg()
{ 
byte chk = EEPROM.read(DETECTORS_EEPROM_ADDR);
  if(chk != CFG_INTEGRITY_CHK_VAL) return 0;
  EEPROM.get((int)(DETECTORS_EEPROM_ADDR+1), detectorDefn);
  return 1;
}
