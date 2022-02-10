/****************************************************************************************
 *  SIGNALS   Module      -Performs Outputs to Signals
 ****************************************************************************************/
#ifndef NUM_SIGNALS
#define NUM_SIGNALS    10
#endif                                      
#define BLK_NDX_MASK  0x0F
#define RT_END_FLAG   0x80

struct SIGNAL_DEFN {
  byte numHeads,              // number of Lamps on this signal
      firstPin,               // Signal Outputs (R,G,Y) for each Lamp
      blkLocator,             // low 4 bits hold Block Index, 0x80 Flags Right End of Block 
      flags;                  // OPTION FLAGS
} signalDefn[NUM_SIGNALS];

unsigned asp, sigAspect[NUM_SIGNALS];

/************************************************************/
void driveSignals() 
{ 
static byte sx, hx, blkX, rtX, rpin, hdCnt, colorFlg, r,g,y, sigSec;  

  for(sx=0; sx<NUM_SIGNALS ;sx++)
  {
    hdCnt = signalDefn[sx].numHeads & 3;    
    if(hdCnt)                   // Configured Signal - Drive it
    {
      blkX = signalDefn[sx].blkLocator & BLK_NDX_MASK;
      rtX  = (signalDefn[sx].blkLocator & RT_END_FLAG) ? 1:0;
      asp  = getBlockEntryApsect(blkX, rtX);
      if(asp != sigAspect[sx])
      {
        if(tstFlag(VERBOSE_FLAG)) 
        { 
          PRINT(F("SIGNAL"));prtNdx(sx);PRINT(F(" for BLOCK"));prtNdx(blkX);PRINT(getBlockID(blkX)); PRINT(F(" at Mile "));
            PRINT(getBlockEndMP(blkX, rtX)); PRINT(F(" =Rule #")); PRINT(asp+400); prtSP();prtCh('>');
            PRINT(getAspectDesc(asp));PRINT(F(" = ")); PRINT(getAspectClrDesc(asp));prtSP();PRINTLN(timeStr());
        }
        sigAspect[sx] = asp;
      }
              // INTERCEPT asp WITH sigTstAsp IF  SIGNAL_TST_FLAG  ACTIVE
      if(tstFlag(SIGNAL_TST_FLAG) && sx==sigTstNdx && sigTstET<30)
      {
        if(sigSec != getClockSec())      // MANAGE TEST ET
        { 
          sigSec = getClockSec(); 
          if(++sigTstET >= 30)
          {
            clrFlag(SIGNAL_TST_FLAG);
            PRINT(F("End Signal Test"));PRINTLN(timeStr());
          }
        }
        if(sigTstAsp>=400) sigTstAsp-=400; 
        asp = sigTstAsp;
      }
              //////////////////////////////////////////////////////////
              // DRIVE EACH SIGNAL LAMP according to Signal Type
              // 0) R,G,Y     1) R,G     2) R,G BIPOLAR
              //////////////////////////////////////////////////////////
      rpin = signalDefn[sx].firstPin;     // GET 1st DIG OUTPUT INDEX - gets INCR at each Output
      if(validDigPin(rpin))
      {
        for(hx=0; hx<hdCnt ;hx++) // Drive each of the Signal Heads - each will have 1 color set
        {
          colorFlg = getSignalColor(asp, hdCnt, hx);      // Get Color selection for this Lamp (Head)
          r = g = y = 0;
          if(!(colorFlg & FLASH) || getHalfSec())         // If FLASH Flag then ALL OFF for first Half of this Second
          {                                               // IF NOT Flashing OFF Then Set the Colors
            r = colorFlg & RED ? 1:0;                     // Set each of the Color Outputs
            g = colorFlg & GRN ? 1:0;
            y = colorFlg & YLW ? 1:0;
          }
                  
          if(signalDefn[sx].flags & SIGNAL_RG_BIPOLAR_OPT)
          {
            if(y)       // YELLOW = FLIP QUICKLY >>Find empty location in flipList
            {
              for(byte fx=0; fx<FLIP_LIST_LEN ;fx++)
              {
                if(flipList[fx]==rpin) break;           // already in the list - quit
                if(flipList[fx]==0)                     // Found available locn
                {
                  flipList[fx] = rpin;                  // insert this rpin, then quit search
                  ++flipYlwCnt;                         // incr # of actv Bipolar YLW
                  break;
                }
              }
            }
            else if(flipYlwCnt)       // NOT YELLOW & IT MAY BE IN THE LIST
            {
              for(byte fx=0; fx<FLIP_LIST_LEN ;fx++)    // Make sure this rpin is NOT in flipList[]
              {
                if(flipList[fx]==rpin)                  // Found rpin in flipList[] locn
                {
                  flipList[fx] = 0;                     // Rmv rpin from flpList, then quit
                  --flipYlwCnt;
                  break;
                }
              }
            }
                  // NOW WRITE R,G OUTPUTS               
            pinMode(rpin, OUTPUT); digitalWrite(rpin, r);   // WRITE RED
            pinMode(++rpin,OUTPUT); digitalWrite(rpin, g);  // WRITE GRN     
            ++rpin;   // INCR BEFORE NEXT LAMP
          }
          else if(signalDefn[sx].flags & SIGNAL_RG_ONLY_OPT)  // 2 Outputs
          {
            if(y) r=g=1;     // YELLOW = BOTH RED & GRN ON
            pinMode(rpin, OUTPUT); digitalWrite(rpin, r);   // WRITE RED
            pinMode(++rpin,OUTPUT); digitalWrite(rpin, g);  // WRITE GRN       
            ++rpin;   // INCR BEFORE NEXT LAMP
          }
          else    // SIGNAL_RGY_TYP - dedicated output for each color ->3 Outputs
          { 
            pinMode(rpin, OUTPUT); digitalWrite(rpin, r);   // WRITE RED
            pinMode(++rpin,OUTPUT); digitalWrite(rpin, g);  // WRITE GRN
            pinMode(++rpin,OUTPUT); digitalWrite(rpin, y);  // WRITE YLW      
            ++rpin;   // INCR BEFORE NEXT LAMP
          }
        }
      }
    }
  } 
}

/************************************************************/
byte getSignalColor(unsigned aspect, byte numHeads, byte headNdx)
{
  switch(aspect)
  {
    case CLEAR: if(!headNdx) return GRN; return RED;    //405
    case CLEAR_TO_LIMITED: if(!headNdx) return YLW; if(headNdx==1) return (GRN + FLASH); return RED; //406
    case CLEAR_TO_MEDIUM: if(!headNdx) return YLW; if(headNdx==1) return GRN; return RED; // 407
    case CLEAR_TO_SLOW: if(headNdx<=1) return YLW; return RED;  // 409
    case CLEAR_TO_STOP: if(!headNdx) return YLW; return RED;    // 411
    case ADVANCE_CLEAR_TO_MEDIUM: if(!headNdx) return (YLW + FLASH); if(headNdx==1) return GRN; return RED; // 413
    case ADVANCE_CLEAR_TO_SLOW: if(!headNdx) return (YLW + FLASH); if(headNdx==1) return YLW; return RED;   // 414
    case ADVANCE_CLEAR_TO_STOP: if(!headNdx) return (YLW + FLASH); return RED;    // 415
    case SLOW_TO_CLEAR: if(!headNdx || (numHeads==3 && headNdx==1)) return RED; return GRN; // 431
    case SLOW_TO_STOP: if(!headNdx || (numHeads==3 && headNdx==1)) return RED; return (YLW + FLASH); // 435
    case RESTRICTING: if(!headNdx || (numHeads==3 && headNdx==1)) return RED; return YLW; // 436
    case SIG_OFF_ASPECT:  return 0; // 99
    case STOP_SIGNAL: default: return RED;   // 439
  }
} 

/************************************************************/
unsigned getSignalAspect(byte sigX) { return sigAspect[sigX]; }

                            //////////////////////////////////////////////////////////////////////////
                            //  CONFIGURATION PARAMETER ACCESS
                            //////////////////////////////////////////////////////////////////////////
/************************************************************/
void initSignals()
{
byte x;  
  for(x=0; x<NUM_SIGNALS ;x++) { signalDefn[x].numHeads = 0; }
  if(!readSignalsCfg()) { }
  for(x=0; x<NUM_SIGNALS ;x++) { sigAspect[x] = STOP_SIGNAL; }
      // CREATE 2 SIGNALS for DEMONSTRATION if none are configured
  if(!signalDefn[0].numHeads && !signalDefn[1].numHeads)
  {
    addSignal(12.3, "BMRC_123", 22, 2, 0);
    addSignal(29.4, "BMRC_123", 26, 2, 0);
  }
}

/************************************************************/
byte getSignalFirstOutput(byte sigX) { return signalDefn[sigX].firstPin; }

/************************************************************/
byte getNumSignalOutputs(byte sigX) 
{
  byte cnt = (signalDefn[sigX].flags & SIGNAL_RG_BIPOLAR_OPT || signalDefn[sigX].flags & SIGNAL_RG_ONLY_OPT) ? 2:3;
  cnt *= signalDefn[sigX].numHeads;
  return cnt; 
} 

/************************************************************
 *  addSignal
 ************************************************************/
byte addSignal(float mp, char *blkID, byte firstDigOut, byte numLamps, byte optFlags)
{
byte rtEnd, sigX, blkX = findBlockNdx(blkID); 
  if(blkX < getMaxBlocks() && validDigPin(firstDigOut))    // Located Block OK & DigOutput is valid
  {    
    for(sigX=0; sigX<NUM_SIGNALS ;sigX++)                 // Scan for Next available Table Entry
    { if(!signalDefn[sigX].numHeads) break;               // No Heads == Undefined
    }
    if(sigX<NUM_SIGNALS)                                  // Found empty table entry to fill
    {
      rtEnd = selectBlkEnd(blkX, mp);
      if(rtEnd > 10) return 0;                            // Abort if Error
      signalDefn[sigX].blkLocator = blkX & BLK_NDX_MASK;
      if(rtEnd)
        signalDefn[sigX].blkLocator |= RT_END_FLAG;
      signalDefn[sigX].numHeads = numLamps;
      signalDefn[sigX].firstPin = firstDigOut;
      signalDefn[sigX].flags    = optFlags;
      return 1;
    }
  }
  return 0;
} 

/************************************************************/
byte signalDefined(byte sigX) { return signalDefn[sigX].numHeads; } 

/************************************************************/
byte getSignalBlockNdx(byte sigX) { return signalDefn[sigX].blkLocator & BLK_NDX_MASK; }

/************************************************************/
byte getSignalBlockRtEnd(byte sigX) { return (signalDefn[sigX].blkLocator & RT_END_FLAG) ? 1:0; }

/************************************************************/
byte getSignalTyp(byte sigX) { return signalDefn[sigX].flags; }

/************************************************************/
byte numSignalLamps(byte sigX) { return signalDefn[sigX].numHeads & 3; } 

/************************************************************/
void delSignal(byte sigX) { if(sigX<NUM_SIGNALS) signalDefn[sigX].numHeads=0; } 

/************************************************************/
byte getNumSignals() 
{ 
byte sx,cnt=0;
  for(sx=0; sx<NUM_SIGNALS ;sx++)
  { if(signalDefn[sx].numHeads) ++cnt;
  }
  return cnt;
}

/************************************************************/
byte getMaxSignals() { return NUM_SIGNALS; }


/************************************************************/
char *getAspectDesc(unsigned aspect)
{
static char adesc[20];
byte x=0; 
  switch(aspect)
  {
    case SLOW_TO_STOP: adesc[x++]='S';adesc[x++]='L';adesc[x++]='O';adesc[x++]='W';adesc[x++]='-';adesc[x++]='S';adesc[x++]='T';adesc[x++]='O';adesc[x++]='P'; break;
    case RESTRICTING: adesc[x++]='R';adesc[x++]='E';adesc[x++]='S';adesc[x++]='T';adesc[x++]='R';adesc[x++]='I';adesc[x++]='C';adesc[x++]='T';adesc[x++]='I';adesc[x++]='N';adesc[x++]='G'; break;
    case SLOW_TO_CLEAR: adesc[x++]='S';adesc[x++]='L';adesc[x++]='O';adesc[x++]='W';adesc[x++]='-';adesc[x++]='C';adesc[x++]='L';adesc[x++]='E';adesc[x++]='A';adesc[x++]='R'; break;
    case CLEAR_TO_STOP: adesc[x++]='C';adesc[x++]='L';adesc[x++]='E';adesc[x++]='A';adesc[x++]='R';adesc[x++]='-';adesc[x++]='S';adesc[x++]='T';adesc[x++]='O';adesc[x++]='P'; break;
    case CLEAR_TO_SLOW: adesc[x++]='C';adesc[x++]='L';adesc[x++]='E';adesc[x++]='A';adesc[x++]='R';adesc[x++]='-';adesc[x++]='S';adesc[x++]='L';adesc[x++]='O';adesc[x++]='W'; break;
    case CLEAR_TO_MEDIUM: adesc[x++]='C';adesc[x++]='L';adesc[x++]='E';adesc[x++]='A';adesc[x++]='R';adesc[x++]='-';adesc[x++]='M';adesc[x++]='E';adesc[x++]='D';adesc[x++]='I';adesc[x++]='U';adesc[x++]='M'; break;
    case CLEAR_TO_LIMITED: adesc[x++]='C';adesc[x++]='L';adesc[x++]='E';adesc[x++]='A';adesc[x++]='R';adesc[x++]='-';adesc[x++]='L';adesc[x++]='I';adesc[x++]='M';adesc[x++]='I';adesc[x++]='T';adesc[x++]='D'; break;
    case ADVANCE_CLEAR_TO_STOP: adesc[x++]='A';adesc[x++]='D';adesc[x++]='V';adesc[x++]='-';adesc[x++]='C';adesc[x++]='L';adesc[x++]='R';adesc[x++]='-';adesc[x++]='S';adesc[x++]='T';adesc[x++]='O';adesc[x++]='P'; break;
    case ADVANCE_CLEAR_TO_SLOW: adesc[x++]='A';adesc[x++]='D';adesc[x++]='V';adesc[x++]='-';adesc[x++]='C';adesc[x++]='L';adesc[x++]='R';adesc[x++]='-';adesc[x++]='S';adesc[x++]='L';adesc[x++]='O';adesc[x++]='W'; break;
    case ADVANCE_CLEAR_TO_MEDIUM: adesc[x++]='A';adesc[x++]='D';adesc[x++]='V';adesc[x++]='-';adesc[x++]='C';adesc[x++]='L';adesc[x++]='R';adesc[x++]='-';adesc[x++]='M';adesc[x++]='E';adesc[x++]='D'; break;
    case CLEAR: adesc[x++]='C';adesc[x++]='L';adesc[x++]='E';adesc[x++]='A';adesc[x++]='R';break;
    case SIG_OFF_ASPECT: adesc[x++]='O';adesc[x++]='F';adesc[x++]='F';break;
    case STOP_SIGNAL: adesc[x++]='S';adesc[x++]='T';adesc[x++]='O';adesc[x++]='P';break;
  }
  adesc[x]=0;
//PRINT("91. x=");PRINTLN(x);
  return adesc;
}

/************************************************************/
char *getAspectClrDesc(unsigned aspect)
{
static char a[20];byte x=0; 
  switch(aspect)
  {
    case SLOW_TO_STOP: a[x++]='R';a[x++]='/';a[x++]='Y';a[x++]='-';a[x++]='F';a[x++]='L';a[x++]='A';a[x++]='S';a[x++]='H'; break;
    case RESTRICTING: a[x++]='R';a[x++]='/';a[x++]='Y'; break;
    case SLOW_TO_CLEAR: a[x++]='R';a[x++]='/';a[x++]='G'; break;
    case CLEAR_TO_STOP: a[x++]='Y';a[x++]='/';a[x++]='R';  break;
    case CLEAR_TO_SLOW: a[x++]='Y';a[x++]='/';a[x++]='Y';  break;
    case CLEAR_TO_MEDIUM: a[x++]='Y';a[x++]='/';a[x++]='G'; break;
    case CLEAR_TO_LIMITED: a[x++]='Y';a[x++]='/';a[x++]='G';a[x++]='-';a[x++]='F';a[x++]='L';a[x++]='A';a[x++]='S';a[x++]='H';break;
    case ADVANCE_CLEAR_TO_STOP: a[x++]='Y';a[x++]='-';a[x++]='F';a[x++]='L';a[x++]='A';a[x++]='S';a[x++]='H';a[x++]='/';a[x++]='R'; break;
    case ADVANCE_CLEAR_TO_SLOW: a[x++]='Y';a[x++]='-';a[x++]='F';a[x++]='L';a[x++]='A';a[x++]='S';a[x++]='H';a[x++]='/';a[x++]='Y'; break;  
    case ADVANCE_CLEAR_TO_MEDIUM: a[x++]='Y';a[x++]='-';a[x++]='F';a[x++]='L';a[x++]='A';a[x++]='S';a[x++]='H';a[x++]='/';a[x++]='G'; break;  
    case CLEAR: a[x++]='G';a[x++]='/';a[x++]='R';break;
    case SIG_OFF_ASPECT: a[x++]='-'; break;
    case STOP_SIGNAL: default: a[x++]='R';a[x++]='/';a[x++]='R';break;
  }
  a[x]=0;
  return a;
}

/**************************************************
 * saveSensorCfg()      
 **************************************************/ 
void saveSignalsCfg()         
{
byte chk = CFG_INTEGRITY_CHK_VAL;
  EEPROM.write(SIGNALS_EEPROM_ADDR,chk); 
  EEPROM.put((int)(SIGNALS_EEPROM_ADDR+1), signalDefn); 
  Serial.print(F("SIGNALS CFG SAVED @"));Serial.print(SIGNALS_EEPROM_ADDR);Serial.print(F(" - "));
    Serial.println((sizeof(signalDefn))+SIGNALS_EEPROM_ADDR);
}

/**************************************************
 * readSensorCfg()      -Rtns 1 if REad OK, else 0
 **************************************************/ 
byte readSignalsCfg()
{ 
byte chk = EEPROM.read(SIGNALS_EEPROM_ADDR);  
  if(chk!=CFG_INTEGRITY_CHK_VAL) return 0;
  EEPROM.get((int)(SIGNALS_EEPROM_ADDR+1), signalDefn); 
  return 1;
}
