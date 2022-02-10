/****************************************************************************************
 *  SENSORS  Module   -Reads Optical Sensors via analog port, converts to Digital Results
 *                     Sensor[0] is assigned AI[0]...Sensor[15] to AI[15]
 ****************************************************************************************/
#ifndef NUM_SENSORS
#define NUM_SENSORS    4
#endif

#define SENSOR_IN_USE_STS     1
#define SENSOR_CAL_LO_STS     2
#define SENSOR_CAL_HI_STS     4
#define SENSOR_INVERT_CAL_STS 8
#define LOW_CAL_NDX           0
#define HI_CAL_NDX            1
#define MIDPT_NDX             2

struct TRAIN_SENSOR {
  byte flags,spare; 
  unsigned calMidPt;   
} sensorDefn[NUM_SENSORS]; 
unsigned sensRaw[NUM_SENSORS],      // RT values - don't need to be Saved to EEPROM
  calLow[NUM_SENSORS],
  calHi[NUM_SENSORS];
static unsigned sensorDB;

/************************************************************
 *  readSensors       -Call every ms to maintain consistent timing
 ************************************************************/
void readSensors()
{  unsigned aiV;
  for(byte sx=0; sx<NUM_SENSORS ;sx++)
  {
    if(sensorDefn[sx].flags & SENSOR_IN_USE_STS)              // Configured Sensor - Read it
    {
      aiV = analogRead(sx);
      if(aiV > sensorDefn[sx].calMidPt)      // Curr input = Hi - Reload Deadband Count
      { gbSensorSts[sx] = sensorDB; 
      } 
      sensRaw[sx]=aiV;
    }
  }
}

/***********************************************************
 *  getSensorSts      -Rtns 0 if Inactive, Else >0 if Active
 *                     Active if raw input has been above Trigger within deadband count down
 */
byte getSensorSts(byte sensX) { return gbSensorSts[sensX]; }
 
/***********************************************************  FOR DISPLAY ONLY */
unsigned getSensorInputVal(byte sensX) { return sensRaw[sensX]; }

/***********************************************************
 * setSensorActv    -Used for Simulation only
 */
void setSensorActv(byte sx) { if(sx<NUM_SENSORS) gbSensorSts[sx]=sensorDB; }
                            //////////////////////////////////////////////////////////////////////////
                            //  CALIBRATION PARAMETER ACCESS
                            //////////////////////////////////////////////////////////////////////////

/************************************************************/
void calSensor(byte sensX, byte hiVal)
{
  if(sensX<NUM_SENSORS)
  {
    if(hiVal) 
    { calHi[sensX] = sensRaw[sensX]; sensorDefn[sensX].flags |= SENSOR_CAL_HI_STS;
    }
    else
    { calLow[sensX] = sensRaw[sensX]; sensorDefn[sensX].flags |= SENSOR_CAL_LO_STS;
    }
          // When Both Cals hae been done Calc MidPt ->Halfway in between
    if(sensorDefn[sensX].flags & SENSOR_CAL_HI_STS && sensorDefn[sensX].flags & SENSOR_CAL_LO_STS)    // detect both limits entered
    { sensorDefn[sensX].calMidPt = calLow[sensX] + ((calHi[sensX] - calLow[sensX])/2); 
    }
  }
}

/************************************************************/
unsigned getSensorMidptVal(byte sensX) { return sensorDefn[sensX].calMidPt; } 

                            //////////////////////////////////////////////////////////////////////////
                            //  CONFIGURATION PARAMETER ACCESS
                            //////////////////////////////////////////////////////////////////////////

/************************************************************/
void initSensors()
{
  for(byte x=0; x<4 ;x++) addSensor(x); // Make the minimum 4 - they can be deleted or overwritten by Cfg
  readSensorCfg();                      // Read Sensor Cfg from EEPROM
  if(sensorDB < MIN_SENSOR_DEADBAND || sensorDB > MAX_SENSOR_DEADBAND)
    setSensorDB(DFLT_SENSOR_DEADBAND);  // use DFLT_SENSOR_DEADBAND 
}

/************************************************************/
byte addSensor(byte sx)
{
  if(sx<NUM_SENSORS && sensorDefn[sx].flags==0)
  {
    sensorDefn[sx].flags = SENSOR_IN_USE_STS;                // set active status, clr other flags
    sensorDefn[sx].calMidPt = 510;                           // set Dflt MidPt
    return 1;
  }
  return 0;
}

/************************************************************/
void delSensor(byte sx) { if(sx<NUM_SENSORS) sensorDefn[sx].flags=0; }

/************************************************************/ 
void setSensorDB(unsigned ms) { sensorDB = boundsChk(ms, MIN_SENSOR_DEADBAND, MAX_SENSOR_DEADBAND); }

/************************************************************/ 
unsigned getSeneorDB() { return sensorDB; }

/************************************************************/
byte getSensorDefined(byte sensX) { return sensorDefn[sensX].flags & SENSOR_IN_USE_STS; } 

/************************************************************/
byte sensorCalibrated(byte sx) 
{ 
  if(sx<NUM_SENSORS && sensorDefn[sx].flags & SENSOR_IN_USE_STS && sensorDefn[sx].flags & SENSOR_CAL_HI_STS &&
      sensorDefn[sx].flags & SENSOR_CAL_LO_STS) return 1;
  return 0;
} 

/************************************************************/
byte getNumSensors() 
{ 
byte sx,cnt=0;
  for(sx=0; sx<NUM_SENSORS ;sx++)
  { if(sensorDefn[sx].flags & SENSOR_IN_USE_STS) ++cnt;
  }
  return cnt;
}

/************************************************************/
byte getMaxSensors() { return NUM_SENSORS; }


/**************************************************
 * saveSensorCfg()      
 **************************************************/ 
void saveSensorCfg()         
{
byte chk = CFG_INTEGRITY_CHK_VAL;
int eeAddr = SENSOR_EEPROM_ADDR;
  EEPROM.write(eeAddr++,chk);
  EEPROM.put(eeAddr, sensorDefn); 
  eeAddr += sizeof(sensorDefn);
  EEPROM.put(eeAddr, sensorDB); 
  eeAddr += sizeof(sensorDB);  
  Serial.print(F("SENSOR CFG SAVED @"));Serial.print(SENSOR_EEPROM_ADDR);Serial.print(F(" - "));
    Serial.println(eeAddr-1);
}

/**************************************************
 * readSensorCfg()      -Rtns 1 if REad OK, else 0
 **************************************************/ 
byte readSensorCfg()
{ 
int eeAddr = SENSOR_EEPROM_ADDR;
byte chk = EEPROM.read(eeAddr);
  if(chk != CFG_INTEGRITY_CHK_VAL) return 0;
  ++eeAddr;
  EEPROM.get(eeAddr, sensorDefn); 
  eeAddr += sizeof(sensorDefn);
  EEPROM.get(eeAddr, sensorDB); 
  return 1;
}
