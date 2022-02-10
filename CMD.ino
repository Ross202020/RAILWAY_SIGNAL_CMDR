/**********************************************************************************************************
 *  CMD  Module     -Processing services for Debug Port and, potentially, external Serial Link
 *                  -Processes TEXT COMMANDS to translate them into actions for COnfiguration and Program Control
 *                    or Monitoring.
 *     
 *  Command Parsing: The first word on a line is the Primary COMMAND - These keywords are in the top of the
 *  Vocab Table. The second field(word) is the primary OBJECT. There can be more modifiers,
 *  but the command key coupled with the object key selects the specific action for the command.
 *  This uses simple primitive tools (in UTILS module) to getFld, and strCmpr to process 1 field at a time, 
 *  progressing through the command.
 **********************************************************************************************************/ 
 
#define VOCAB_KEY_LEN     8
 
#define HELP_VX           0
#define GET_VX            1
#define SET_VX            2
#define START_VX          3
#define STOP_VX           4
#define ADD_VX            5
#define SAVE_VX           6
#define SEND_VX           7
#define CAL_VX            8
#define UPDATE_VX         9
#define SELECT_VX         10
#define DEL_VX            11     // Last Cmd
#define SENSOR_VX         12    // First Object 
#define DETECTOR_VX       13
#define BLOCK_VX          14
#define SIGNAL_VX         15
#define ASPECT_VX         16
#define ID_VX             17
#define INPUT_VX          18
#define OUTPUT_VX         19 
#define TAG_VX            20 
#define NAME_VX           21
#define EXPORT_VX         22
#define IMPORT_VX         23
#define BAUD_VX           24
#define RATE_VX           25
#define RANGE_VX          26
#define LIMIT_VX          27
#define HI_VX             28
#define LOW_VX            29
#define TURNOUT_VX        30
#define MAIN_VX           31
#define SIDING_VX         32
#define REMOTE_VX         33
#define SIM_VX            34
#define NETWORK_VX        35
#define PASSWORD_VX       36
#define ADDRESS_VX        37
#define TIME_VX           38

#define FIRST_CMD_NDX     HELP_VX
#define LAST_CMD_NDX      DEL_VX
#define FIRST_OBJ_NDX     DEL_VX+1
#define LAST_OBJ_NDX      TIME_VX
#define LAST_KEY_NDX      LAST_OBJ_NDX

#ifndef MAX_FLD_LEN
#define MAX_FLD_LEN       10
#endif
 
char vocabStr[VOCAB_KEY_LEN+2], fldBfr[MAX_FLD_LEN+2];
/***************************************************
 *  processCmd()    -Parse contents of cmdBfr[]
 ***************************************************/
void processCmd(char *cmdBfr)
{
static byte cmdKey, objKey, replyCode, fldLen, vocabLen, x; char ch; byte lftX, rtX; float mp;

  if(cmdBfr[0] < 0x20)    // DETECT EMPTY COMMAND, OR BINNARY COMM FROM MODBUS
    return;

  replyCode = 99;         // Dflt Reply = "??" 

  if(cmdBfr[1]<=' ')                        // Single Char - ShortCut Cmd
  {
    ch = toupper(cmdBfr[0]);
    if(ch=='S')        { setFlag(SHOW_STS_FLAG); }
    else if(ch=='C')   { setFlag(SHOW_CFG_FLAG); }
    else if(ch=='D')   { setFlag(SHOW_STS_FLAG); toggleFlag(DEBUG_FLAG); } 
    else if(ch=='V')   { setFlag(SHOW_STS_FLAG); toggleFlag(VERBOSE_FLAG); } 
    else if(ch=='H' || ch=='?') { setFlag(SHOW_HLP_FLAG); }
    else if(ch=='M')   
    { 
      if(tstFlag(MONITOR_INPUT_FLAG))    // WAS MONITORING AI - So Turn it Off
        clrFlag(MONITOR_INPUT_FLAG);
      else                            // Not previously monitoring so turn it on
      {
        setFlag(MONITOR_INPUT_FLAG);
        monINPUT = 0;                                          // Dflt = AI[0]
        if(getFld(1, &cmdBfr[2], fldBfr, MAX_FLD_LEN, 0))   // Get AI ndx if preovided
        {  monINPUT = (byte)atoi(fldBfr);
        }
      }
    } 
    else if(ch=='T')   { ++trapFlag; }
    return;
  }
   
        // FIRST GET THE PRIMARY CMD - Its First Field
  fldLen = getFld(1, cmdBfr, fldBfr, MAX_FLD_LEN, UPPR_CASE);     // Extract COMMAND Keyword in UPPR_CASE from 1st fld in cmdBfr[]  
  if(!fldLen) return;               // QUIT IF DID NOT GET A CMD FIELD -> cmdBfr[] must be Empty
  for(cmdKey=FIRST_CMD_NDX; cmdKey <= LAST_CMD_NDX ;cmdKey++)     // VOCAB LOOKUP for COMMAND KEYWORD
  {
    vocabLen = getVocab(cmdKey); 
    if(vocabLen==fldLen)
    { if(strCmpr(fldBfr,vocabStr)) break;     // Break out of search loop when match is found
    }
  }  
      //  Have CMD Figured - if unknown cmdKey will be > LAST_CMD_NDX.  
      //  NOW GET OBJECT
  fldLen = getFld(2, cmdBfr, fldBfr, MAX_FLD_LEN, UPPR_CASE);     // Fetch OBJECT - 2nd fld
  for(objKey=FIRST_OBJ_NDX; objKey <= LAST_OBJ_NDX ;objKey++)     // VOCAB LOOKUP for OBJECT KEYWORD
  {
    vocabLen = getVocab(objKey);
    if(vocabLen==fldLen)
    { if(strCmpr(fldBfr,vocabStr)) break;     // Break out of search loop when match is found
    }
  }    
    if(tstFlag(DEBUG_FLAG)) { PRINT("CMD:");PRINT(cmdKey);PRINT(',');PRINT(objKey); PRINT(" >"); PRINTLN(cmdBfr); }

                  ////////////////////////////////////////////////////////////
                  //  HELP 
                  ////////////////////////////////////////////////////////////
  if(cmdKey==HELP_VX)                                       
  {
    fldLen = getFld(2, cmdBfr, fldBfr, MAX_FLD_LEN, UPPR_CASE);   // got an Argument ->The Help Topic will be a CMD
    if(fldLen)
    {
      for(objKey=FIRST_CMD_NDX; objKey <= LAST_KEY_NDX ;objKey++)     // VOCAB LOOKUP for COMMAND KEYWORD
      {
        vocabLen = getVocab(objKey); 
        if(vocabLen==fldLen)
        { if(strCmpr(fldBfr,vocabStr)) break;     // Break out of search loop when match is found
        }
      }  
            // Now got and get CMD specific Help
      showCmdHelp(objKey); 
    }
    else
      setFlag(SHOW_HLP_FLAG);
    replyCode=0;
  }
                  ////////////////////////////////////////////////////////////
                  //  CAL DETECTOR    "CAL DETECTOR ndx(0-5)  (OCC,UNOCC)
                  ////////////////////////////////////////////////////////////
  else if(cmdKey==CAL_VX)  
  {
    if(objKey==DETECTOR_VX) 
    {
      if(getFld(3, cmdBfr, fldBfr, 2, 0))
      {
        x = (byte)atoi(fldBfr);
        if(x<NUM_DETECTORS)
        {
          if(getFld(4, cmdBfr, fldBfr, MAX_FLD_LEN, UPPR_CASE))
          {
            if(fldBfr[0]=='O') fldLen=1;
            else fldLen=0;
          }
          lftX = getDetectorSensNdx(x, 0); 
          rtX  = getDetectorSensNdx(x, 1); 
          calSensor(lftX, fldLen);
          calSensor(rtX, fldLen);
          replyCode = 104;
        }
      } 
    }
  }
                  ////////////////////////////////////////////////////////////
                  //  ADD     ADD SENSOR,  ADD DETECTOR,  ADD BLOCK,  ADD SIGNAL,  ADD TURNOUT
                  ////////////////////////////////////////////////////////////
  else if(cmdKey==ADD_VX)                                     // "ADD"
  {
    if(objKey==SENSOR_VX)                                    // ADD SENSOR n 
    {
      if(getFld(3, cmdBfr, fldBfr, 2, 0))
      {
        if(addSensor((byte)atoi(fldBfr)))
          replyCode = 104;
      } 
      setFlag(SHOW_CFG_FLAG);
    }
    
    else if(objKey==DETECTOR_VX)                                         // ADD DETECTOR  mile.post leftSensX  rtSensX compass(W,E,N,S)
    { 
      if(getFld(3, cmdBfr, fldBfr, NAME_LEN, INCL_DEC_PT_OPT))                  // get mile fld  
      {
        mp =atof(fldBfr);        
        if(getFld(4, cmdBfr, fldBfr, NAME_LEN, PASS_FLT_FLD_OPT))                // get leftSensX fld
        {
          lftX = (byte)atoi(fldBfr);
          if(getFld(5, cmdBfr, fldBfr, NAME_LEN, PASS_FLT_FLD_OPT))              // get leftSensX fld
          {
            rtX = (byte)atoi(fldBfr);
            if(getFld(6, cmdBfr, fldBfr, NAME_LEN, UPPR_CASE+PASS_FLT_FLD_OPT))      // get Compass fld(W,E,N,S) 
            {   // byte addDetector(float milePost, byte lftSensX, byte rtSensX, byte compassCh)
              if(addDetector(mp, lftX, rtX, fldBfr[0]))
                replyCode = 104;
            }
          } 
          setFlag(SHOW_CFG_FLAG);
        }
      } 
    }
                                    /******************************************************************
                                     *  ADD BLOCK "blkID" leftEndMile  rtEndMile  compassLeft  >>Double Ended Block
                                     *  ADD BLOCK "blkID" leftEndMile  compassLeft     >>Single, Open Ended BLOCK
                                     ******************************************************************/
    else if(objKey==BLOCK_VX)                                        // ADD BLOCK n leftDetX  rtDetX  blkID (N,E,S,W)
    { 
      if(getFld(3, cmdBfr, wrkStr, MAX_FLD_LEN, 0))     // get blkID (3rd) fld into wrkStr[]
      {
        if(getFld(4, cmdBfr, fldBfr, MAX_FLD_LEN, INCL_DEC_PT_OPT) && isdigit(fldBfr[0]))     // get Left (first) MilePost
        {
          mp =atof(fldBfr);        
          lftX = findDetector(mp);
          if(lftX < getMaxDetectors())        // LOCATED LEFT DETECTOR OK
          {
            if(getFld(5, cmdBfr, fldBfr, MAX_FLD_LEN, INCL_DEC_PT_OPT+PASS_FLT_FLD_OPT))    // get 5th fld - could be RT mile or Compass if open ended Block
            {
              if(isdigit(fldBfr[0]))     // FULL BLOCK with BOTH ENDS MAPPED TO DETECTORS AT MILE POSTS
              {                
                mp =atof(fldBfr);  
                rtX = findDetector(mp); 
                if(rtX < getMaxDetectors())        // LOCATED RIGHT DETECTOR OK
                {
                  if(getFld(6, cmdBfr, fldBfr, MAX_FLD_LEN, INCL_DEC_PT_OPT+PASS_FLT_FLD_OPT))    // get 6th fld - Compass
                  {       //  byte addBlock(byte lfDetX, byte rtDetX, byte leftCompass, char *blkID)                    
                    if(addBlock(lftX, rtX, fldBfr[0], wrkStr))
                      replyCode = 104;
                  }
                }                
              }
              else if(fldBfr[0]=='W' || fldBfr[0]=='E' || fldBfr[0]=='N' || fldBfr[0]=='S')   // Compass ->Single Ended Block
              {                  
                rtX = 255;    // Set Right Detector Index == Invalid to indicate Open Ended Block
                if(addBlock(lftX, rtX, fldBfr[0], wrkStr))
                  replyCode = 104;
              }
            }
          }          
        }
      }
      setFlag(SHOW_CFG_FLAG);
    }
    else if(objKey==SIGNAL_VX)                                        // ADD SIGNAL milePost \"blkID\"  num_lamps  firstDigOut  (RG,BIPOLAR)
    {
      if(getFld(3, cmdBfr, fldBfr, MAX_FLD_LEN, INCL_DEC_PT_OPT))     // get MilePost (3rd) fld as a FLoat
      {
        mp = atof(fldBfr);
        if(getFld(5, cmdBfr, fldBfr, MAX_FLD_LEN, PASS_FLT_FLD_OPT))  // get 5th fld holding # Lamps
        {
          fldLen = (byte)atoi(fldBfr);                                  // use fldLen for # Lamps here
          if(getFld(6, cmdBfr, fldBfr, MAX_FLD_LEN, PASS_FLT_FLD_OPT))  // get 6th fld holding DigOutputNdx
          {
            rtX = (byte)atoi(fldBfr);                                   // use rtX  for DigOutputNdx
            lftX = 0;                                                   // use lftX for Option Flags
            if(getFld(7, cmdBfr, fldBfr, MAX_FLD_LEN, UPPR_CASE+PASS_FLT_FLD_OPT))  // get 7th fld holding OPTs - this field is optional
            {
              if(fldBfr[0]=='B')      lftX = SIGNAL_RG_BIPOLAR_OPT;
              else if(fldBfr[0]=='R') lftX = SIGNAL_RG_ONLY_OPT;
            }
            
            if(getFld(4, cmdBfr, fldBfr, MAX_FLD_LEN, PASS_FLT_FLD_OPT))  // get blkID (4th) fld  
            {       // byte addSignal(float mp, char *blkID, byte firstDigOut, byte numLamps, byte optFlags)
              if(addSignal(mp, fldBfr, rtX, fldLen, lftX))
                replyCode = 104;
            }
          }
        }
        setFlag(SHOW_CFG_FLAG);
      }      
    }     
    else if(objKey==TURNOUT_VX)   // ADD TURNOUT  \"blkID\"  digInput_ndx  facingDir_compass(W,E,N,S)  (LOW_ACTV)
    {
      if(getFld(3, cmdBfr, fldBfr, MAX_FLD_LEN, 0))   // Get blkID string in 3rd fld
      {
        x = findBlockNdx(fldBfr);
        if( x < getMaxBlocks())        // LOCATED BLOCK OK
        {
          ch = getBlockEndCompass(x, LF_INPUT_NDX);         // use ch to hold Left Compass Direction of Block Left 
          if(getFld(4, cmdBfr, fldBfr, MAX_FLD_LEN, 0))     // get turnout digital input num (4th) fld
          {
            lftX = (byte)atoi(fldBfr);                      // reuse lftX to convert DigInput Ndx
            if(getFld(5, cmdBfr, fldBfr, MAX_FLD_LEN, UPPR_CASE))   // get Compasss (5th) fld in Upper Case
            {
              rtX = findSubStr("LOW", cmdBfr);                // reuse rtX to detect "LOW_ACTV" . rtn==255 if Not found
                  // byte addTurnout(byte blkX, byte inputX, byte lowActv, byte rtFacing)
              if(addTurnout(x, lftX, rtX<CMDSTR_LEN ? 1:0, ch==fldBfr[0] ? 0:1))
                replyCode = 104;
              setFlag(SHOW_CFG_FLAG); 
            }
          }
        }
      }
    }
  }
                  ////////////////////////////////////////////////////////////
                  //  SET
                  ////////////////////////////////////////////////////////////
  else if(cmdKey==SET_VX)                                     // "SET"
  {
    if(objKey==TIME_VX)                                       // SET TIME
    {
      byte h=0,m=0,s=0;
      if(getFld(3, cmdBfr, fldBfr, MAX_FLD_LEN, 0))
      {
        h = (byte)atoi(fldBfr);
        if(getFld(4, cmdBfr, fldBfr, MAX_FLD_LEN, 0))
          m = (byte)atoi(fldBfr);
        if(getFld(5, cmdBfr, fldBfr, MAX_FLD_LEN, 0))
          s = (byte)atoi(fldBfr);
        set_Time(h,m,s);
        setFlag(SHOW_STS_FLAG);
        replyCode=104; 
      }
    }
    else if(objKey==SENSOR_VX)                                  // SET SENSOR
    {
      if(getFld(3, cmdBfr, fldBfr, MAX_FLD_LEN, UPPR_CASE))     // Fetch 3rd Fld
      {
        if(findSubStr("DEAD",fldBfr) < 2)                       // SET SENSOR DEADBAND
        {
          if(getFld(4, cmdBfr, fldBfr, MAX_FLD_LEN, 0))
          {
            setSensorDB((byte)atoi(fldBfr));
            replyCode=104;
          }
          setFlag(SHOW_CFG_FLAG);
        }
      }      
    }
    else if(objKey==SIDING_VX)                                  // SET SIDING blkID
    {
      if(getFld(3, cmdBfr, fldBfr, MAX_FLD_LEN, 0))             // Fetch blkID 3rd Fld
      { 
        x = findBlockNdx(fldBfr); 
        if(x < getMaxBlocks())                              // found Block
        {
          setBlockOpt(x, SIDING_TRACK_TYP);
          replyCode = 104;
        }
        setFlag(SHOW_CFG_FLAG);
      }      
    }
/****
    else if(objKey==NETWORK_VX)                                  // SET NETWORK
    {
      if(getFld(3, cmdBfr, fldBfr, MAX_FLD_LEN, UPPR_CASE))             // Fetch blkID 3rd Fld
      { 
        if(fldBfr[0]=='N')                                      // SET NETWORK NAME ssid
        {
          if(getFld(4, cmdBfr, wrkStr, WRKSTR_LEN, 0))         // get ssid (4th) fld
          {
            setNetworkName(wrkStr);
            replyCode = 104;
          }
        }
        else if(fldBfr[0]=='P')                                 // SET NETWORK PASSWORD
        {
          if(getFld(4, cmdBfr, wrkStr, WRKSTR_LEN, 0))         // get pwd (4th) fld
          {
            setNetworkPwd(wrkStr);
            replyCode = 104;
          }
        }
        else if(fldBfr[0]=='A')                                 // SET NETWORK ADDRESS 192.168.3.123
        {
          byte ip[4],fldX;
          for(fldX=0;fldX<4;fldX++)
          {
            if(getFld(4+fldX, cmdBfr, fldBfr, MAX_FLD_LEN, 0))
              ip[fldX] = (byte)atoi(fldBfr);
            else
              break;
          }
          if(fldX==4)         // get pwd (4th) fld
          {
            setStaticIP(ip);
            replyCode = 104;
          }
        }
        setFlag(SHOW_CFG_FLAG);
      }      
    }
****/    
    else if(objKey>LAST_OBJ_NDX)                                // UNKNOWN OBJECT
    {
      if(getFld(2, cmdBfr, fldBfr, MAX_FLD_LEN, UPPR_CASE))     // Fetch Object (2nd) field
      {
        if(findSubStr("OCC", fldBfr) < 10)                      // "SET OCC   "
        {
          if(getFld(3, cmdBfr, fldBfr, MAX_FLD_LEN, UPPR_CASE))     // Fetch 2nd Object (3rd) field
          {
            if(fldBfr[0]=='T')                                  // SET OCC TIMEOUT sec
            {
              if(getFld(4, cmdBfr, fldBfr, MAX_FLD_LEN, UPPR_CASE))   // fetch 4th fld - this is the seconds
              {
                unsigned unsV = (unsigned)atoi(fldBfr);         // convert ASCII to int
                setBlockTimeout(unsV);
                setFlag(SHOW_CFG_FLAG);
                replyCode = 104;
              }
            }
          }
        }
        else if(findSubStr("ADJACENT", fldBfr) < 10) // SET ADJACENT \"blkID\"  compass(W,E,N,S) \"adjacentBlkID\"
        {
          if(getFld(3, cmdBfr, fldBfr, MAX_FLD_LEN, 0))   // Get blkID string in 3rd fld
          {
            x = findBlockNdx(fldBfr);
            if( x < getMaxBlocks())        // LOCATED BLOCK OK
            {
              if(getFld(4, cmdBfr, fldBfr, MAX_FLD_LEN, UPPR_CASE))   // Get 4th fld with COMPASS DIR OF ADJACENT BLOCK
              {
                ch = fldBfr[0];                                       // store adjacent block direction letter in ch 
                if(getFld(5, cmdBfr, fldBfr, MAX_FLD_LEN, 0))         // fetch adjacent blkID string from 5th fld - no case conversion
                {     // byte setAdjacentBlk(byte blkX, byte rtSel, char adjID[]) 
                  rtX = ch==getBlockEndCompass(x, RT_INPUT_NDX) ? 1:0;
                  if(setAdjacentBlk(x,  rtX , fldBfr)) 
                    replyCode = 104;
                  setFlag(SHOW_CFG_FLAG); 
                }
              }
            }
          }          
        }
      }
    }
  }
                  ////////////////////////////////////////////////////////////
                  //  DEL  SENSOR,  DEL DETECTOR,  DEL BLOCK, DEL SIGNAL
                  ////////////////////////////////////////////////////////////
  else if(cmdKey==DEL_VX) 
  {   
    if(objKey==SENSOR_VX)
    {
      if(getFld(3, cmdBfr, fldBfr, 2, 0))
      {
        delSensor((byte)atoi(fldBfr));  
        replyCode = 104;
      }      
      setFlag(SHOW_CFG_FLAG);
    }
    else if(objKey==DETECTOR_VX)
    {
      if(getFld(3, cmdBfr, fldBfr, MAX_FLD_LEN, INCL_DEC_PT_OPT))
      {
        float mp = atof(fldBfr);
        x = findDetector(mp);
        if(delDetector(x)) 
          replyCode = 104;
      }      
      setFlag(SHOW_CFG_FLAG);
    }
    else if(objKey==BLOCK_VX)                                 // DEL BLOCK "blkID"
    {
      if(getFld(3, cmdBfr, fldBfr, MAX_FLD_LEN, 0))
      { 
        if(delBlock(fldBfr))
          replyCode = 104;
      }      
      setFlag(SHOW_CFG_FLAG);
    }
    else if(objKey==SIGNAL_VX)                                // DEL SIGNAL n
    {
      if(getFld(3, cmdBfr, fldBfr, 2, 0))
      { 
        delSignal((byte)atoi(fldBfr)); 
        setFlag(SHOW_CFG_FLAG);
        replyCode = 104;
      }      
      setFlag(SHOW_CFG_FLAG);
    }
  } 
                  ////////////////////////////////////////////////////////////
                  //  SAVE
                  ////////////////////////////////////////////////////////////
  else if(cmdKey==SAVE_VX) 
  {  setFlag(SAVE_CFG_FLAG); replyCode = 104;
  } 
                  ////////////////////////////////////////////////////////////
                  //  START
                  ////////////////////////////////////////////////////////////
  else if(cmdKey==START_VX) 
  {  
    if(objKey==SIM_VX)                                    // START SIM detectX  rt_lft(1,0)
    {
      if(getFld(3, cmdBfr, fldBfr, MAX_FLD_LEN, 0))   // Get   3rd fld
      {
        simDetX = (byte)atoi(fldBfr);
        if(getFld(4, cmdBfr, fldBfr, MAX_FLD_LEN, 0))   // Get 4th fld
        {
          simRL = (byte)atoi(fldBfr);
          simState = 0;   setFlag(SIMULATION_ACTV_FLAG + VERBOSE_FLAG);
          replyCode = 104;
        }
      }
    } 
    else if(objKey==SIGNAL_VX) 
    {
      if(getFld(3, cmdBfr, fldBfr, MAX_FLD_LEN, 0))   // Get sigNdx 3rd fld
      {
        sigTstNdx = (byte)atoi(fldBfr); 
      }      
      if(getFld(4, cmdBfr, fldBfr, MAX_FLD_LEN, 0))   // Get ruleNum 4th fld
      {
        sigTstAsp = atoi(fldBfr);
      }
      if(sigTstAsp<100) sigTstAsp += 400;
      PRINT(F(" Begin 30 sec Test of SIGNAL"));prtNdx(sigTstNdx);prtEQ();PRINT(sigTstAsp); prtSP(); PRINTLN(timeStr());
      setFlag(SIGNAL_TST_FLAG);
      sigTstET=0;
      replyCode = 104;
    }
  } 
                        /////////////////////////////////////////////////////////////////////
                        //  ISSUE CMD PROCESS STATUS IF REQUIRED
                        /////////////////////////////////////////////////////////////////////
  if(replyCode==104) PRINTLN(F("OK "));
  else if(replyCode==99) PRINTLN(F("?? "));
}

////////////////////////////////////////////////////  
void showSts()
{ byte x, sts, cnt; unsigned asp;
  prtRtn();
  border(1); 
  prtSP(4); prtCh('@'); prtSP(); PRINTLN(timeStr()); 
  prtSP(2);PRINT(getNumSensors());PRINTLN(F(" SENSORS:"));
  for(x=0; x<getMaxSensors() ;x++)
  {
    if(getSensorDefined(x))
    {
      prtSP(6);prtNdx(x);
      sts = getSensorSts(x); if(sts) PRINT(F(" ACTV"));else PRINT(F(" Idle"));
      PRINT(F(", InputV="));PRINTLN(getSensorInputVal(x));
    }
  }
  prtSP(2);PRINT(getNumDetectors());PRINTLN(F(" DETECTORS:"));
  for(x=0; x<getMaxDetectors() ;x++)
  {
    if(detectorDefined(x))
    {
      prtSP(6);prtNdx(x);PRINT(F(" MILE_POST="));PRINT(getDetectorMP(x));PRINT(F(", STATUS="));
      sts = getDetectorSts(x);prtPin(sts);PRINT(getDetectStsExpl(x)); 
      PRINT(F(",LastTrainDir="));PRINT((char)getLastTrainDir(x));
      PRINT(F("  #Events="));PRINTLN(getDetectEventCnt(x));
    }
  }
  prtSP(2);PRINT(getNumBlocks());PRINTLN(F(" BLOCKS:"));
  for(x=0; x<getMaxBlocks() ;x++)
  {
    if(blockDefined(x))
    {
      prtSP(6);prtNdx(x);PRINT(getBlockID(x)); PRINT(getBlockOccSts(x) ? " =OCC" : " =UNOCC"); 
        PRINT(F("  ET=")); PRINT(getBlockET(x));
        PRINT(F(" sec, LstTrainDir="));PRINT((char)getBlkLastTrainDir(x));
        PRINT(F(", #Trains="));PRINTLN(getBlockNumTrains(x));
      cnt = (getNumTurnouts(x) & 7);
      if(cnt)
      {
        for(byte tx=0; tx<cnt ;tx++)
        {
          prtSP(20);PRINT(F("TURNOUT"));prtNdx(tx); prtEQ();PRINTLN(getTurnoutSts(x,tx));
        }
      }
    }
  }
  prtSP(2);PRINT(getNumSignals());PRINTLN(F(" SIGNALS:"));
  for(x=0; x<getMaxSignals() ;x++)
  {
    if(signalDefined(x))
    {
      asp = getSignalAspect(x);
      prtSP(6);prtNdx(x);PRINT(F(" Rule #"));PRINT(asp+400);prtSP();prtCh('>');PRINT(getAspectDesc(asp));prtSP();
        prtEQ();prtSP();PRINTLN(getAspectClrDesc(asp));
    }
  }
  sts = getNumTags();
  prtSP(2);PRINT(sts);PRINTLN(F(" TAGS:"));
  if(sts)
  {
    for(x=0; x<getTagTblSiz() ;x++)
    {
      if(tagDefined(x))
      {  prtSP(6);prtNdx(x);prtSP();PRINT(getTagName(x));PRINT(F(" = "));PRINTLN(getTagVal(x));
      }
    }
  }
  
  PRINT(F("  DEBUG="));PRINT(tstFlag(DEBUG_FLAG) ? 1:0); 
    PRINT(F(",  VERBOSE="));PRINT(tstFlag(VERBOSE_FLAG) ? 1:0); 
    PRINT(F(",  SIM="));PRINT(tstFlag(SIMULATION_ACTV_FLAG) ? 1:0); 
    PRINT(F(",  MONITOR="));PRINTLN(tstFlag(MONITOR_INPUT_FLAG) ? monINPUT : 0); 
  border(0);
}

////////////////////////////////////////////////////  
void showCfg()
{ byte x,y, blkX, optFlag; 
  prtRtn();
  border(2); 
  prtSP(2);PRINT(getNumSensors());PRINT(F(" SENSORS:   Deadband ms ="));PRINTLN(getSeneorDB());    
  for(x=0; x<getMaxSensors() ;x++)
  { 
    if(getSensorDefined(x)) 
    { 
      prtSP(6); PRINT(F("AI"));prtNdx(x);  
      if(sensorCalibrated(x))
      {
        PRINT(F(" CALIBRATED MidPt="));PRINTLN(getSensorMidptVal(x));
      }
      else
      {
        PRINTLN(F(" NOT CALIBRATED"));
      }
    }
  }
  
  prtSP(2);PRINT(getNumDetectors());PRINTLN(F(" DETECTORS:"));
  for(x=0; x<getMaxDetectors() ;x++)
  {
    if(detectorDefined(x))
    {  
      prtSP(6);prtNdx(x);PRINT(F(" MILE_POST="));PRINT(getDetectorMP(x));PRINT(F(", LEFT_SENSOR:"));
      PRINT((char)getDetectorOrientation(x)); prtNdx(getDetectorSensNdx(x,0));prtSP(); 
      PRINT(F(", RT_SENSOR:"));prtNdx(getDetectorSensNdx(x,1));  prtRtn();
    }
  }  
  prtSP(2);PRINT(getNumBlocks());PRINTLN(F(" BLOCKS:"));
  for(x=0; x<getMaxBlocks() ;x++)
  {
    if(blockDefined(x))
    { 
      char dirCh = getBlockEndCompass(x, LF_INPUT_NDX); 
      prtSP(6);prtNdx(x);PRINT(getBlockID(x));  
      if(getBlockOpt(x, SIDING_TRACK_TYP)) PRINT(F(" >SIDING ")); else PRINT(F(" >MAINLINE "));
      
      if(openEndedBlock(x))
      {
        PRINT(F(", ENTRY MILEPOST="));PRINT( getBlockEndMP(x,0)); 
        PRINT(F(", DETECTOR:")); prtNdx(getBlockDetNdx(x,0)); 
        PRINT(F(", NEXT BLOCK ID:"));PRINT(getAdjacentBlk(x,LF_INPUT_NDX));
      }
      else
      {
        PRINT(F(", MILEPOSTS: LEFT="));PRINT( getBlockEndMP(x,0)); 
        PRINT(F(", RT="));PRINT( getBlockEndMP(x,1));         
        PRINT(F(", DETECTORS: LF:")); prtNdx(getBlockDetNdx(x,0)); 
          PRINT(F(", RT:")); prtNdx(getBlockDetNdx(x,1)); 
          PRINT(F(", Compass LEFT=")); PRINT(dirCh);
          PRINT(F(", ADJACENT BLOCKS: LF="));PRINT(getAdjacentBlk(x,LF_INPUT_NDX));
          PRINT(F(", RT="));PRINT(getAdjacentBlk(x,RT_INPUT_NDX));
      }          
      byte cnt = getNumTurnouts(x);
      cnt = cnt<= MAX_BLOCK_TURNOUTS ? cnt:MAX_BLOCK_TURNOUTS;
      if(!cnt) PRINTLN(F(" , NO TURNOUTS"));
      else
      {
        prtRtn();prtSP(9);PRINT(cnt); PRINT(F(" TURNOUTS:")); 
        for(y=0; y<cnt ;y++)
        {  
          if(y)prtSP(20);
          prtNdx(y);prtSP();
          dirCh = getBlockEndCompass(x, LF_INPUT_NDX); 
          if(getTurnoutRightFacing(x, y))                     // If Right Facing, get opposite dir
            dirCh = getBlockEndCompass(x, RT_INPUT_NDX);             
          PRINT(F(", PtsFacing=")); PRINT(dirCh);
          PRINT(F(", Input=")); prtPin(getTurnoutInput(x,y));
          if(getTurnoutLowActv(x,y)) PRINT(F("(LOW_ACTV) "));
          prtRtn();
        }
        prtRtn();
      }
    }
  }
  PRINT(F("    BLOCK OCC TIMEOUT sec="));PRINTLN(getBlockTimeout());
  
  prtSP(2);PRINT(getNumSignals());PRINTLN(F(" SIGNALS:"));
  byte rtEnd, digX; char blkEndDir; float mp;
  for(x=0; x<getMaxSignals() ;x++)
  {
    if(signalDefined(x))
    { 
      y = numSignalLamps(x);
      optFlag = getSignalTyp(x);
      blkX = getSignalBlockNdx(x);
      rtEnd = getSignalBlockRtEnd(x);
      blkEndDir = getBlockEndCompass(blkX, rtEnd);
      mp = getBlockEndMP(blkX, rtEnd);
      digX = getSignalFirstOutput(x);
      
      prtSP(6);prtNdx(x);prtSP();prtCh(blkEndDir); PRINT(F(" Entry to Block"));prtNdx(blkX);
        PRINT(getBlockID(blkX)); PRINT(F(" at MP=")); PRINT(mp); PRINT(F("  #Lamps=")); PRINT(y);
        if(optFlag==SIGNAL_RGY_TYP) PRINT(F(", 3 Outputs(R,G,Y) for each Lamp "));
        else if(optFlag==SIGNAL_RG_ONLY_OPT) PRINT(F(", 2 Outputs(R,G) for each Lamp "));
        else if(optFlag==SIGNAL_RG_BIPOLAR_OPT) PRINT(F(", 2 Outputs(R,G Bipolar) for each Lamp "));
        PRINT(F(", Outputs"));prtPin(digX);prtCh('-');prtPin(digX+getNumSignalOutputs(x)-1);prtRtn();
    }
  } 
  y = getNumTags();
  prtSP(2);PRINT(y);PRINTLN(F(" TAGS:"));
  if(y)
  {
    for(x=0; x<getTagTblSiz() ;x++)
    {
      if(tagDefined(x))
      {  prtSP(6);prtNdx(x);prtSP();PRINTLN(getTagName(x));
      }
    }
  }
  
  border(0);
}

////////////////////////////////////////////////////  
void showHelp()
{
  prtRtn();
  border(3);
  PRINTLN(F(" DEBUG CMDS: S  ==ShowSts, C  ==Config,  D  ==Debug,  H  ==Help"));
  prtSP(13);     PRINT(F("V  ==Verbose, M  ==Monitor Input[n] ->Add 100 for DigInput")); prtRtn();
  PRINTLN(F(" CMDS: "));
  showCmdHelp(ADD_VX);  
  showCmdHelp(DEL_VX);  
  showCmdHelp(SET_VX);  
  // showCmdHelp(GET_VX);    // future
  showCmdHelp(SAVE_VX);  
  showCmdHelp(START_VX);  
  showCmdHelp(HELP_VX);  
  prtRtn();
  PRINTLN(F(">SENSOR INPUTS (photo or UV) are mapped to Anaolg Inputs with same Index."));
  PRINTLN(F(" Sensor Bool Status { ACTIVE, IDLE }. ACTIVE if it has been above Trigger Level within")); 
  PRINTLN(F(" DEADBAND ms (Dflt=250 ms). Deadband filters noise as train passes - gaps are expected."));
  PRINTLN(F(" CONNECTIONS: Sensor Low Leg = GND. Hi Leg = Resistor to Vcc (typical 220K). Hi Leg is also"));
  PRINTLN(F(" Analog Input connect point. Light lowers Sensor impedence==lower V drop across sensor"));
  PRINTLN(F(" Light Interruption increases V drop and Sensor Input value."));
  PRINTLN(F(" CAL DETECTOR in each state (OCCUPIED, UNOCCUPIED) to establish sensor specific levels")); 
  PRINTLN(F(" which sets MidPoint Level for ON-OFF translation.")); 
  PRINT  (F("  Max SENSORS=")); PRINTLN(NUM_SENSORS); prtRtn();
  PRINTLN(F(">DETECTOR:  Sensor Pair physically separated by 3 to 5 cm located at MilePost."));
  PRINTLN(F("  LEFT and RIGHT Sensor  +  Compasss Orientation of LEFT vs RT.  +  MilePost"));
  PRINTLN(F(" Detectors are polled fast (2 ms) enough to detect Start and End of Train with Direction."));
  PRINTLN(F(" Direction of End of Last Train is used to set Occupancy of Blocks."));
  PRINTLN(F(" CAL DETECTOR operations for both OCCUPIED & UNOCCUPIED State needs to be done once,"));
  PRINT(  F("  then SAVE.  Max DETECTORS="));PRINTLN(NUM_DETECTORS); prtRtn();
  PRINTLN(F(">BLOCK: Track Segment with 2 ENDS that are mapped to DETECTORS at MilePosts"));
  PRINTLN(F(" Block = OCC if Either End==Active. Block==OCC until EXIT detected."));
  PRINTLN(F(" 2 Trains in Block is illegal. Train can be entirely inside Block ends, or be longer than"));
  PRINTLN(F(" Block. Trains may ENTER then Reverse back out. All cases are handled. Any EXIT Clears OCC.")); 
  PRINTLN(F(" OCC will Timeout (Recover) if both ends are IDLE ->In case train is lifted off track in Block.")); 
  PRINTLN(F(" Block CONFIG: Compass Orientation is direction (N,E,S,W) from RT to LEFT End."));
  PRINTLN(F(" Configure Open Ended Block for Spur or Edge of Signaled Territory.")); 
  PRINTLN(F(" BLOCK status includes OCCUPIED Status, ENTRY PERMISSIVE Levels (0-3), Entry Signal ASPECT."));
  PRINTLN(F(" Entry Permit== OCC sts + TURNOUT sts + ADJ_BLK permit + Main/Siding Typ + Timeout"));
  PRINTLN(F(" >>OPEN ENDED BLOCK with only 1 End having an internal DETECTOR handles SPUR or edge of"));
  PRINTLN(F(" Signaled Territory. Trains entering an OPEN ENDED BLOCK ('Dark Territory') trigger it to OCCUPIED")); 
  PRINTLN(F(" and its Entry at the configured end operate the same. However OCC TIMEOUT back to UNOCC = 15 sec.")); 
  PRINTLN(F(" Block ENTRY PERMIT status is published and transferred via TAGS which are used internally")); 
  PRINTLN(F(" Undef Adjacent Blocks always assumed to have Full Entry Permit. "));
  PRINT  (F("  Max BLOCKS= "));PRINT(NUM_BLOCKS);PRINT(F(", Max Internal TURNOUTS/Blk= "));PRINTLN(MAX_BLOCK_TURNOUTS); prtRtn();
  PRINTLN(F(">SIGNAL:  Each Signal is associated with a BLOCK ENTRY. Each Signal has 1,2, or 3 Lamps. >> 3 SIGNAL TYPES:"));
  PRINTLN(F("   1) R,G.Y   with 3 consecutive Dig Outputs/Lamp,")); 
  PRINTLN(F("   2) R,G     with 2 consec Outputs/Lamp. Yellow is simulated with Both Active."));
  PRINTLN(F("   3) R,G BIPOLAR with 2 consec Outputs/Lamp. Yellow is simulated by reversing outputs every ms."));
  PRINTLN(F("   A Signal reserves Consecutive Outputs for all Lamps, so a 3 Lamp Typ 1 uses 9 consec Outputs, while a"));
  PRINTLN(F("   single head Type 2,or 3, only requires 2 consecutive Outputs. ALL Signal LAMPS Require a Resistor for EACH COLOR !"));
  PRINTLN(F("   Resistors should be 220 Ohm to 1K ->  1 K is best to avoid high total current draw on Arduino Outputs"));
  PRINTLN(F("  >>Can NOT use Common Resistor on GND for Types 2  ->only 1 clr will ever be on when driving YLW due to"));
  PRINTLN(F("   different LED trigger levels.    TEST Signal for 30 sec with Debug cmd >START SIGNAL sigNdx aspect_rule"));  
  PRINTLN(F("CROR SIGNAL Aspects in its librabry (only uses a subset  **): "));
  PRINTLN(F("    (405) CLEAR                   GRN,RED,RED            **"));
  PRINTLN(F("    (406) CLEAR-to-LIMITED        YLW,GRN_FLASH,RED      **"));
  PRINTLN(F("    (407) CLEAR-to-MEDIUM         YLW,GRN,RED            **"));
  PRINTLN(F("    (409) CLEAR-to-SLOW           YLW,YLW,RED"));
  PRINTLN(F("    (411) CLEAR-to-STOP           YLW,RED,RED            **"));
  PRINTLN(F("    (413) ADV_CLR-to-MEDIUM       YLW_FL,GRN,RED         **")); 
  PRINTLN(F("    (414) ADV_CLR-to-SLOW         YLW_FL,YLW,RED")); 
  PRINTLN(F("    (415) ADV_CLR-to-STOP         YLW_FL,RED,RED")); 
  PRINTLN(F("    (431) SLOW-to-CLEAR           RED,GRN  or  RED,RED,GRN  **")); 
  PRINTLN(F("    (435) SLOW-to-STOP            RED,YLW_FLASH or RED,RED,YLW_FLASH")); 
  PRINTLN(F("    (436) RESTRICTING             RED,YLW  or RED,RED,YLW   **")); 
  PRINTLN(F("    (439) STOP                    RED,RED,RED               **")); 
  PRINT  (F("  Max SIGNALS="));PRINTLN(NUM_SIGNALS);  prtRtn();
  PRINTLN(F("  Configuration Steps: 1)SENSORS, 2)DETECTOR, 3)BLOCKS, 4)ADJ_BLOCKS, 5)TURNOUTS, 6)SIGNALS,  7) SAVE"));
  PRINTLN(F("  Getting Started TEST:>If there is no previous configuration this app will create a single BLOCK with"));
  PRINTLN(F("   2 Detectors and 2 Signals. On the Serial Monitor (Debug) issue Cmd>START SIM 0 0     "));
  PRINTLN(F("     This will simulat a Train Passing Detector[0] from Left To Right ->Observe the actions for 30 sec."));
  PRINTLN(F("     At this point BLOCK = OCCUPIED. Now have Train pass out the otehr End with Cmd> START SIM 1 0")); 
  PRINTLN(F("  Make Train go opposite direction> START SIM 1 1      then 30 second later>START SIM 0 1")); 
  PRINTLN(F("  Observe messages - This demonstrate Block Occupancy + Entry Permit Signal Aspects taht progress from STOP"));
  PRINTLN(F("  to SLOW-to-CLEAR at 4 to 10 seconds after Block becomes UNOCC, then progress to CLEAR."));
  PRINTLN(F("  All 3 Signal LED Types have been Tested."));
//           0123456789 123456789 123456789 123456789 123456789 123456789 123456789 123456789 
  border(0);
} 

////////////////////////////////////////////////////  
void showCmdHelp(byte key)
{
  if(getVocab(key))
  { 
    for(byte x=0; x<10 ;x++) { PRINT(F("= ")); if(x==5) { PRINT(vocabStr);PRINT(F(" HELP ")); } } prtRtn();
  }
  
  if(key==ADD_VX)
  {
     prtSP(4);PRINT  (F("ADD SENSOR  input_n(0-"));PRINT(getMaxSensors()-1); PRINTLN(F(")"));
     prtSP(4);PRINTLN(F("ADD DETECTOR  milePost  leftSensorNdx  rtSensorNdx  dir_towardLeft(W,E,N,S)"));
      prtSP(8);PRINTLN(F(">>Sensor Pair using consec anal inputs. Required for Each Block End. 1 Detector can serve up to 3 Blocks"));
     prtSP(4);PRINTLN(F("ADD BLOCK  \"blkID\" left_endMile  rt_endMile dir_towardLeft(W,E,N,S)"));
     prtSP(4);PRINTLN(F("ADD BLOCK  \"blkID\" entryMilePost dir_towardEntry(W,E,N,S)"));
      prtSP(8);PRINTLN(F(">>Open Ended BLOCK For Spur or Edge of Signal Territory with no Far Detector"));
     prtSP(4);PRINTLN(F("ADD SIGNAL milePost \"blkID\"  num_lamps  firstDigOut (RG, BIPOLAR)"));
      prtSP(8);PRINTLN(F(">>Match BLOCK End Mile. Dflt is RGY. RG=>No Y Lamp, BIPOLAR=>R-G Lamp Type. Signals Mod to accomodate"));
     prtSP(4);PRINTLN(F("ADD TURNOUT  \"blkID\"  digInput_ndx  facingDir_compass(W,E,N,S)  (LOW_ACTV)"));
      prtSP(8);PRINTLN(F(">>Adds Turnout to named BLOCK"));
  }
  else if(key==DEL_VX)
  {
     prtSP(4);PRINTLN(F("DEL SENSOR  input_n"));
     prtSP(4);PRINTLN(F("DEL DETECTOR  milePost"));
     prtSP(4);PRINTLN(F("DEL BLOCK \"blkID\""));
     prtSP(4);PRINTLN(F("DEL SIGNAL milePost \"blkID\"")); 
  }
  else if(key==SET_VX)
  {
     prtSP(4);PRINTLN(F("SET ADJACENT \"blkID\"  compass(W,E,N,S) \"adjacentBlkID\"")); 
     prtSP(4);PRINTLN(F("SET SIDING   \"blkID\"    ->sets Track Type=SIDING for Block. Dflt=MAIN")); 
     prtSP(4);PRINTLN(F("SET SENSOR DEADBAND  ms(250 - 1500)        >>Filters Gap Noise as trains  pass Sensors")); 
     prtSP(4);PRINTLN(F("SET OCC TIMEOUT  sec(60 - 600)             >>OCCUPIED Block Timeout ->Resets to UNOCC")); 
     prtSP(4);PRINTLN(F("SET TIME  h m s")); 
  }
  else if(key==CAL_VX || key==DETECTOR_VX)
  {
    prtSP(4);PRINT(F("CAL DETECTOR ndx(0-5) (OCC, UNOCC)")); prtSP(8);PRINTLN(F(">>Saves Input Levels & sets Trigger Pt for Train Detection")); 
      prtSP(8); PRINT(F(">>CAL Every Detector, Block Light Source for OCC only. SAVE Afer all completed"));
  }
  else if(key==SAVE_VX)
  {
    prtSP(4);PRINT(F("SAVE")); prtSP(8);PRINTLN(F("Saves Configuration to EEPROM"));
  }
  else if(key==START_VX)
  {
    prtSP(4);PRINT(F("START SIM detect_n(0-5)  rt_left(1,0)")); 
      prtSP(8);PRINTLN(F(">>Simulates Train Passing selected Detector & Direction for about 10 sec"));
    prtSP(4);PRINT(F("START SIGNAL sigNdx aspect_ruleNum"));prtSP(8);PRINTLN(F(">>Tests indexed signal for CROR RuleNum for 30 sec"));
  }
  else if(key==HELP_VX)
  {
    prtSP(4); PRINTLN(F("Help Format >  HELP topic {ADD,SET,CAL,DEL,START,SAVE}")); 
  } 
}

////////////////////////////////////////////////////  
void border(byte wrdSel) 
{ 
byte x;  
  for(byte x=0;x< (wrdSel ? 20:30) ;x++) 
  { 
    vocabStr[0]='-';vocabStr[1]=' ';vocabStr[2]=0;
    PRINT(vocabStr);
    if(x==10 && wrdSel)
    {
      prtAppNam();prtAppRev(); 
      if(wrdSel==1) { vocabStr[0]='S';vocabStr[1]='T';vocabStr[2]='A';vocabStr[3]='T';vocabStr[4]='U';vocabStr[5]='S';vocabStr[6]=0; } 
      else if(wrdSel==2) { vocabStr[0]='C';vocabStr[1]='O';vocabStr[2]='N';vocabStr[3]='F';vocabStr[4]='I';vocabStr[5]='G';vocabStr[6]=0; } 
      else if(wrdSel==3) { vocabStr[0]='H';vocabStr[1]='E';vocabStr[2]='L';vocabStr[3]='P';vocabStr[4]=0; } 
      PRINT(vocabStr);
    }
  } prtRtn(); 
}

/**********************************************
 * getVocab()   -Fills global vocabStr[], Returns Len
 *    Avoids use of extra calls with string pointers 
 **********************************************/
byte getVocab(byte key)
{
byte vx=0;  
  if(key==GET_VX)
  { vocabStr[vx++]='G';vocabStr[vx++]='E';vocabStr[vx++]='T';
  }
  else if(key==SET_VX)
  { vocabStr[vx++]='S';vocabStr[vx++]='E';vocabStr[vx++]='T';
  }
  else if(key==START_VX)
  { vocabStr[vx++]='S';vocabStr[vx++]='T';vocabStr[vx++]='A';vocabStr[vx++]='R';vocabStr[vx++]='T'; 
  }
  else if(key==STOP_VX)
  { vocabStr[vx++]='S';vocabStr[vx++]='T';vocabStr[vx++]='O';vocabStr[vx++]='P'; 
  }
  else if(key==ADD_VX)
  { vocabStr[vx++]='A';vocabStr[vx++]='D';vocabStr[vx++]='D';
  }
  else if(key==SAVE_VX)
  { vocabStr[vx++]='S';vocabStr[vx++]='A';vocabStr[vx++]='V';vocabStr[vx++]='E';
  }
  else if(key==SEND_VX)
  { vocabStr[vx++]='S';vocabStr[vx++]='E';vocabStr[vx++]='N';vocabStr[vx++]='D';
  }
  else if(key==CAL_VX)
  { vocabStr[vx++]='C';vocabStr[vx++]='A';vocabStr[vx++]='L';
  }
  else if(key==UPDATE_VX)
  { vocabStr[vx++]='U';vocabStr[vx++]='P';vocabStr[vx++]='D';vocabStr[vx++]='A';vocabStr[vx++]='T';vocabStr[vx++]='E';
  }
  else if(key==SELECT_VX)
  { vocabStr[vx++]='S';vocabStr[vx++]='E';vocabStr[vx++]='L';vocabStr[vx++]='E';vocabStr[vx++]='C';vocabStr[vx++]='T';
  }
  else if(key==DEL_VX)
  { vocabStr[vx++]='D';vocabStr[vx++]='E';vocabStr[vx++]='L';
  }
      // = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = 
  else if(key==SENSOR_VX)
  { vocabStr[vx++]='S';vocabStr[vx++]='E';vocabStr[vx++]='N';vocabStr[vx++]='S';vocabStr[vx++]='O';vocabStr[vx++]='R';
  }
  else if(key==DETECTOR_VX)
  { vocabStr[vx++]='D';vocabStr[vx++]='E';vocabStr[vx++]='T';vocabStr[vx++]='E';vocabStr[vx++]='C';vocabStr[vx++]='T';vocabStr[vx++]='O';vocabStr[vx++]='R';
  }
  else if(key==BLOCK_VX)
  { vocabStr[vx++]='B';vocabStr[vx++]='L';vocabStr[vx++]='O';vocabStr[vx++]='C';vocabStr[vx++]='K'; 
  }
  else if(key==SIGNAL_VX)
  { vocabStr[vx++]='S';vocabStr[vx++]='I';vocabStr[vx++]='G';vocabStr[vx++]='N';vocabStr[vx++]='A';vocabStr[vx++]='L';
  }
  else if(key==ASPECT_VX)
  { vocabStr[vx++]='A';vocabStr[vx++]='S';vocabStr[vx++]='P';vocabStr[vx++]='E';vocabStr[vx++]='C';vocabStr[vx++]='T';
  }
  else if(key==SIGNAL_VX)
  { vocabStr[vx++]='S';vocabStr[vx++]='I';vocabStr[vx++]='G';vocabStr[vx++]='N';vocabStr[vx++]='A';vocabStr[vx++]='L';  
  }
  else if(key==ASPECT_VX)
  { vocabStr[vx++]='A';vocabStr[vx++]='S';vocabStr[vx++]='P';vocabStr[vx++]='E'; vocabStr[vx++]='C'; vocabStr[vx++]='T';  
  } 
  else if(key==ID_VX)
  { vocabStr[vx++]='I';vocabStr[vx++]='D'; 
  }
  else if(key==INPUT_VX)
  { vocabStr[vx++]='I';vocabStr[vx++]='N';vocabStr[vx++]='P';vocabStr[vx++]='U';vocabStr[vx++]='T';
  }
  else if(key==OUTPUT_VX)
  { vocabStr[vx++]='O';vocabStr[vx++]='U';vocabStr[vx++]='T';vocabStr[vx++]='P';vocabStr[vx++]='U';vocabStr[vx++]='T';
  }
  else if(key==TAG_VX)
  { vocabStr[vx++]='T';vocabStr[vx++]='A';vocabStr[vx++]='G';
  }
  else if(key==NAME_VX)
  { vocabStr[vx++]='N';vocabStr[vx++]='A';vocabStr[vx++]='M';vocabStr[vx++]='E';
  }
  else if(key==EXPORT_VX)
  { vocabStr[vx++]='E';vocabStr[vx++]='X';vocabStr[vx++]='P';vocabStr[vx++]='O';vocabStr[vx++]='R';vocabStr[vx++]='T'; 
  }
  else if(key==IMPORT_VX)
  { vocabStr[vx++]='I';vocabStr[vx++]='M';vocabStr[vx++]='P';vocabStr[vx++]='O';vocabStr[vx++]='R';vocabStr[vx++]='T'; 
  }
  else if(key==BAUD_VX)
  { vocabStr[vx++]='B';vocabStr[vx++]='A';vocabStr[vx++]='U';vocabStr[vx++]='D';  
  }
  else if(key==RATE_VX)
  { vocabStr[vx++]='R';vocabStr[vx++]='A';vocabStr[vx++]='T';vocabStr[vx++]='E';
  }
  else if(key==RANGE_VX)
  { vocabStr[vx++]='R';vocabStr[vx++]='A';vocabStr[vx++]='N';vocabStr[vx++]='G';vocabStr[vx++]='E';
  }
  else if(key==LIMIT_VX)
  { vocabStr[vx++]='L';vocabStr[vx++]='I';vocabStr[vx++]='M';vocabStr[vx++]='I';vocabStr[vx++]='T';
  }
  else if(key==HI_VX)
  { vocabStr[vx++]='H';vocabStr[vx++]='I';
  }
  else if(key==LOW_VX)
  { vocabStr[vx++]='L';vocabStr[vx++]='O';vocabStr[vx++]='W';
  }
  else if(key==TURNOUT_VX)
  { vocabStr[vx++]='T';vocabStr[vx++]='U';vocabStr[vx++]='R';vocabStr[vx++]='N';vocabStr[vx++]='O';vocabStr[vx++]='U';vocabStr[vx++]='T';
  }
  else if(key==MAIN_VX)
  { vocabStr[vx++]='M';vocabStr[vx++]='A';vocabStr[vx++]='I';vocabStr[vx++]='N';
  }
  else if(key==SIDING_VX)
  { vocabStr[vx++]='S';vocabStr[vx++]='I';vocabStr[vx++]='D';vocabStr[vx++]='I';vocabStr[vx++]='N';vocabStr[vx++]='G';
  }
  else if(key==REMOTE_VX)
  { vocabStr[vx++]='R';vocabStr[vx++]='E';vocabStr[vx++]='M';vocabStr[vx++]='O';vocabStr[vx++]='T';vocabStr[vx++]='E';
  }
  else if(key==SIM_VX)
  { vocabStr[vx++]='S';vocabStr[vx++]='I';vocabStr[vx++]='M';
  }
  else if(key==NETWORK_VX)
  { vocabStr[vx++]='N';vocabStr[vx++]='E';vocabStr[vx++]='T';vocabStr[vx++]='W';vocabStr[vx++]='O';vocabStr[vx++]='R';vocabStr[vx++]='K';
  }
  else if(key==PASSWORD_VX)
  { vocabStr[vx++]='P';vocabStr[vx++]='A';vocabStr[vx++]='S';vocabStr[vx++]='S';vocabStr[vx++]='W';
    vocabStr[vx++]='O';vocabStr[vx++]='R';vocabStr[vx++]='D';
  }
  else if(key==ADDRESS_VX)
  { vocabStr[vx++]='A';vocabStr[vx++]='D';vocabStr[vx++]='D';vocabStr[vx++]='R';vocabStr[vx++]='E';vocabStr[vx++]='S';vocabStr[vx++]='S';
  }
  else if(key==TIME_VX)
  { vocabStr[vx++]='T';vocabStr[vx++]='I';vocabStr[vx++]='M';vocabStr[vx++]='E';
  }
  else if(key==0)   // HELP_VX
  { vocabStr[vx++]='H';vocabStr[vx++]='E';vocabStr[vx++]='L';vocabStr[vx++]='P';
  }
  vocabStr[vx] = 0;
  return vx;
} 
