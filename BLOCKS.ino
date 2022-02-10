/****************************************************************************************
 *  BLOCKS  Module   -Every Block has a Left Detector, Right Detector, IDstr[]
 ****************************************************************************************/
#ifndef NUM_BLOCKS
#define NUM_BLOCKS          6     // Every Block has Left and Right Detector, 8 Adjacent Blocks require 9 Detectors
#endif
#define TURNOUT_NUM_MASK        0x3F
#define TURNOUT_LOW_ACTV_FLG    0x80
#define TURNOUT_RT_FACING_FLG   0x40 

struct BLOCK {
  byte detectNdx[2],                // [0]==Left, [1]==Right  DETECTOR
       flags,                       // holds # Internal Turnouts + SIDING Designation in Hi Bit
       turnoutNdx[MAX_BLOCK_TURNOUTS],  // Internal TURNOUTS DigInput Indexs. 2 Hi Bits are used as Flags
       turnoutCnt,
       compassLeft;                 // Holds ASCII (W,E,S,N) Compass Orientation of the Left End relative to RT
  char idStr[NAME_LEN+2],           // ID of THIS Block
    adjBlkID[2][NAME_LEN+2];        // ID of Adjacent Blocks 
  unsigned spare, numTrains;
} blockDefn[NUM_BLOCKS]; 

unsigned blkTOsec,
      blockET[NUM_BLOCKS],
      permitCycleCnt;
static byte blkOccSts[NUM_BLOCKS],                // RT Vars -Block Occ is primary
      lastDetSts[NUM_BLOCKS][2],
      entryPerm[NUM_BLOCKS][2],                   // Entry Permit for each end of Block
      entryAspect[NUM_BLOCKS][2],                 // Entry Signal Aspect for each end of Block
      adjEntryPerm[NUM_BLOCKS][2],                // Entry Permit for Adjacent Blocks
      turnoutSts[NUM_BLOCKS][MAX_BLOCK_TURNOUTS], // Internal Turnout Sts
      trainDir[NUM_BLOCKS],                       // Direction of Last Train
      blkSec;
      
/***********************************************************
 * setBlockOccSts()     
 */
byte setBlockOccSts()
{ static byte chgCnt, detX, dSts, blkSec, lstSts;

  if(blkSec != getClockSec())                       // UPDATE ET FOR ALL BLOCKS
  { blkSec=getClockSec();  for(detX=0; detX<NUM_BLOCKS; detX++) ++blockET[detX]; 
  }
  
  chgCnt=0;
  for(byte blkX=0; blkX<NUM_BLOCKS ;blkX++)
  {  
    if(blockDefn[blkX].idStr[0]>='A')             // only service configured BLOCKS
    {
            ///////////////////////////////////////////////////////////
            //// FIRST CHECK LEFT END
            ///////////////////////////////////////////////////////////
      detX = blockDefn[blkX].detectNdx[LF_INPUT_NDX];
      lstSts = lastDetSts[blkX][LF_INPUT_NDX];
      dSts   = gbDetectSts[detX];

      if(dSts != lstSts)        // LEFT DETECTOR STATUS HAS CHANGED
      {
        trainDir[blkX] = getLastTrainDir(detX);      // GET TRAIN DIRECTION AT THIS DETECTOR
        if(dSts)                // Its Occupied
        {                       //  so Block is OCC
          if(!blkOccSts[blkX])      // BLOCK was NOT OCC Last Pass
          {
            blkOccSts[blkX]=1;
            blockET[blkX] = 0;
            blockDefn[blkX].numTrains++;
            if(tstFlag(VERBOSE_FLAG+DEBUG_FLAG))
            {
              PRINT(F("BLOCK OCC:"));prtNdx(blkX);PRINT(blockDefn[blkX].idStr);
                PRINT(F(" Dir="));PRINT((char)trainDir[blkX]);prtSP();PRINTLN(timeStr());
            }
          }
        }
        else    // DETECTOR NOW IDLE: 1) EXIT COMPLETE, 2) ENTRY COMPLETED - Still in Block
        {
          if(trainDir[blkX]==blockDefn[blkX].compassLeft)   // TRAIN JUST EXITED LEFT END
          {
            if(blkOccSts[blkX])      // BLOCK was  OCC Last Pass
            {
              blkOccSts[blkX] = 0;
              blockET[blkX] = 0;
              if(tstFlag(VERBOSE_FLAG+DEBUG_FLAG))
              {
                PRINT(F("BLOCK UNOCC:"));prtNdx(blkX);PRINT(blockDefn[blkX].idStr);
                  PRINT(F(" Dir="));PRINT((char)trainDir[blkX]);prtSP();PRINTLN(timeStr());
              }
            }            
          }
          else {}   // TRAIN NOW FULLY INSIDE BLOCK          
        }
      }
      lastDetSts[blkX][LF_INPUT_NDX] = dSts;    // save current LF detector sts
      
            ///////////////////////////////////////////////////////////
            //// NEXT CHECK RT END
            ///////////////////////////////////////////////////////////
      detX = blockDefn[blkX].detectNdx[RT_INPUT_NDX];
      lstSts = lastDetSts[blkX][RT_INPUT_NDX];
      dSts   = gbDetectSts[detX];

      if(dSts != lstSts)        // RT DETECTOR STATUS HAS CHANGED
      {
        trainDir[blkX] = getLastTrainDir(detX);      // GET TRAIN DIRECTION AT THIS DETECTOR
        if(dSts)                // Its Occupied
        {                       //  so Block is OCC
          if(!blkOccSts[blkX])      // BLOCK was NOT OCC Last Pass
          {
            blkOccSts[blkX]=1;
            blockET[blkX] = 0;
            blockDefn[blkX].numTrains++;
            if(tstFlag(VERBOSE_FLAG+DEBUG_FLAG))
            {
              PRINT(F("BLOCK OCC:"));prtNdx(blkX);PRINT(blockDefn[blkX].idStr);
                PRINT(F(" Dir="));PRINT((char)trainDir[blkX]);prtSP();PRINTLN(timeStr());
            }
          }
        }
        else    // DETECTOR NOW IDLE: 1) EXIT COMPLETE, 2) ENTRY COMPLETED - Still in Block
        {
          if(trainDir[blkX] != blockDefn[blkX].compassLeft)   // TRAIN JUST EXITED RT END
          {
            if(blkOccSts[blkX])      // BLOCK was  OCC Last Pass
            {
              blkOccSts[blkX] = 0;
              blockET[blkX] = 0;
              if(tstFlag(VERBOSE_FLAG+DEBUG_FLAG))
              {
                PRINT(F("BLOCK UNOCC:"));prtNdx(blkX);PRINT(blockDefn[blkX].idStr);
                  PRINT(F(" Dir="));PRINT((char)trainDir[blkX]);prtSP();PRINTLN(timeStr());
              }
            }            
          }
          else {}   // TRAIN NOW FULLY INSIDE BLOCK          
        }
      }
      lastDetSts[blkX][RT_INPUT_NDX] = dSts;    // save current RT detector sts
      
            ///////////////////////////////////////////////////////////
            //  CHECK FOR OCC TIMEOUT if both Detectors are IDLE 
            ///////////////////////////////////////////////////////////
      if(blkOccSts[blkX] && !gbDetectSts[blockDefn[blkX].detectNdx[LF_INPUT_NDX]] && !gbDetectSts[blockDefn[blkX].detectNdx[RT_INPUT_NDX]])
      {       // OCCUPIED & DETECTORS ARE IDLE -->TEST FOR TIMEOUT
        if(blockET[blkX] > blkTOsec || (openEndedBlock(blkX) && blockET[blkX]>=OPEN_ENDED_BLK_TO))
        {
          PRINT(F("BLOCK OCC TIMEOUT ! "));PRINT(blockDefn[blkX].idStr); PRINT(F(" ->UNOCCUPIED "));PRINTLN(timeStr());
          blkOccSts[blkX] = 0;
          blockET[blkX] = 0;
        }
      }      
    } // end if block is Valid
  }
  return chgCnt;
}

/************************************************************
 *  setBlockEntrySts      -Lots of Code here and no big rush to get it done
 *                          so just do 1 BLOCK each time
 ************************************************************/
void setBlockEntrySts()
{
static byte blkX, toNdx, txx, nxtToSts, facingPtDir, toSts[2], adjPermit[2], tagX, trackTyp;
  if(blkX>=NUM_BLOCKS) 
  {
    blkX=0;                  //Wrap to start
    ++permitCycleCnt;
  }
  if(blockDefn[blkX].idStr[0]>='A')             // only service configured BLOCKS
  {
    if(blkOccSts[blkX])                         // BLOCK IS OCCUPIED
    {                                           //  NO Permissive, Aspect = STOP in both Dir
      entryAspect[blkX][LF_INPUT_NDX] = entryAspect[blkX][RT_INPUT_NDX] = STOP_SIGNAL;
      entryPerm[blkX][LF_INPUT_NDX] = entryPerm[blkX][RT_INPUT_NDX] = NO_ENTRY_PERMIT_LVL;
    }
    else      // UNOCC
    {
            ////////////////////////////////////////////////////////////////
            //  DETERMINE TURNOUT STATUS FOR EACH DIRECTION
            ////////////////////////////////////////////////////////////////
      toSts[0] = toSts[1] = 0;
      for(txx=0; txx<getNumTurnouts(blkX) && txx<MAX_BLOCK_TURNOUTS ;txx++)
      {
        toNdx = getTurnoutInput(blkX, txx);
        nxtToSts=0;
        if(validDigPin(toNdx)) 
        {
          nxtToSts = digitalRead(toNdx);      // READ TURNOUT INPUT  1=ACTV
          turnoutSts[blkX][txx] = nxtToSts;
        }
        if(nxtToSts)        // it is Turned Out
        {
          if(getTurnoutRightFacing(blkX, txx))
          {
            toSts[LF_INPUT_NDX] = TURNOUT_RVS_ACTV_STS;
            if(toSts[RT_INPUT_NDX]==0) 
              toSts[RT_INPUT_NDX] = TURNOUT_FACING_ACTV_STS;
          }
          else      // LEFT FACING
          {
            toSts[RT_INPUT_NDX] = TURNOUT_RVS_ACTV_STS;
            if(toSts[LF_INPUT_NDX]==0) 
              toSts[LF_INPUT_NDX] = TURNOUT_FACING_ACTV_STS;              
          }
        } 
      }
            ////////////////////////////////////////////////////////////////
            //  DETERMINE PERMISSIVE LEVELS OF ADJACENT BLOCK 
            //   If no id or Tag Not Found then Assume Max Permit
            //   Build TAG Name with -W or -E or -N or -S suffix from idStr
            ////////////////////////////////////////////////////////////////
      for(txx=0; txx<2 ;txx++)
      { 
        adjPermit[txx] = 0;
        if(blockDefn[blkX].adjBlkID[LF_INPUT_NDX][0] >= '0')    // This Adjacent Block has a defined ID
        {
              //BUILD TAG NAME TO FETCH THIS INFO IN TAGS:  idStr-W  or idStr-E ...-N, -S
          cpStr(blockDefn[blkX].adjBlkID[LF_INPUT_NDX], wrkStr, NAME_LEN);
          wrkStr[NAME_LEN]=0;
          wrkStrLen = strlen(wrkStr);   // Determine how long idStr is
          wrkStr[wrkStrLen++] = '-';    // append suffix starting with '-'
              // If Adjacent Block is to the West of this Block, then assume we are looking for its "-E" tag
              //  for Adjacent Blk to the Right of this Blk, we need its direction which is same as This Block Left
          if(txx==LF_INPUT_NDX) 
          {
            if(blockDefn[blkX].compassLeft == 'W') wrkStr[wrkStrLen++] = 'E';
            else if(blockDefn[blkX].compassLeft == 'N') wrkStr[wrkStrLen++] = 'S';
            else if(blockDefn[blkX].compassLeft == 'S') wrkStr[wrkStrLen++] = 'N';
            else wrkStr[wrkStrLen++] = 'W';
          }
          else
            wrkStr[wrkStrLen++] = blockDefn[blkX].compassLeft;
          wrkStr[wrkStrLen] = 0;      // TERMINATE TAG NAME STRING
              // NOW LOOK FOR IT IN TAGS
          tagX = lookupTag(wrkStr);
          if(tagX < getTagTblSiz())     // Tag was found in TAGS
          {
            adjPermit[txx] = (byte)getTagVal(tagX);
          }
          else    // COULD NOT FIND THIS NAME IN TAGS
          {
            adjPermit[txx] = MAX_ENTRY_PERMIT_LVL;
                //////////////////////////////////////////////////////////////////////////////////
                // IF ADJACENT BLOCK HAS A VALID ID BUT DOES NOT HAVE A TAG FOR IT
                //  AFTER SEVERAL CYCLES, THEN ASSUME ITS DATA WILL COME VIA NETWORK
                //  SO CREATE AN IMPORT TAG, AND SET IT TO MAX_ENTRY_PERMIT_LVL
                // IF NETWORK SERVER IS ABLE TO RETRIEVE THIS NAMED TAG VALUE FROM THE NETWORK
                // THEN THIS BLOCK WILL GET SYNCED WITH THE ACTUAL VALUE 
                // IF IT NEVER GETS LINKED, THEN THE INITIAL VALUE = MAX_ENTRY_PERMIT_LVL will persist
                //////////////////////////////////////////////////////////////////////////////////
            if(permitCycleCnt > 3)   // Its been around >3 times, so if this was a local Block the tag would be there
            {              
              tagX = createTag(wrkStr, INT_TAG_TYP);          // UPDATE TAGS - createTag returns Tag Index
              setTagVal(tagX, MAX_ENTRY_PERMIT_LVL);          // Write  MAX_ENTRY_PERMIT_LVL
            }
          }
        }
        else 
          adjPermit[txx] = MAX_ENTRY_PERMIT_LVL;
      }
            ////////////////////////////////////////////////////////////////
            // SET TRACK TYP: (MAINLINE, SIDING)
            ////////////////////////////////////////////////////////////////
      if(blockDefn[blkX].flags & SHORT_SIDING_TRACK_TYP)
        trackTyp = SHORT_SIDING_TRACK_TYP;
      else if(blockDefn[blkX].flags & SIDING_TRACK_TYP)
        trackTyp = SIDING_TRACK_TYP;
      else
        trackTyp = MAINLINE_TRACK_TYP;
        
            ////////////////////////////////////////////////////////////////
            //  SET ENTRY PERMISSIVE, SIGNAL ASPECT FOR EACH END of BLOCK
            //    Write TAG to inform other (Adjacent) Blocks
            ////////////////////////////////////////////////////////////////
      for(txx=0; txx<2 ;txx++)
      {
        if(blockET[blkX] >= 10)   // ITS BEEN UNOCC for at least 10 Sec
        {
          if(toSts[txx] == TURNOUT_RVS_ACTV_STS)
          {
            entryPerm[blkX][txx] = NO_ENTRY_PERMIT_LVL;
            entryAspect[blkX][txx] = STOP_SIGNAL;         // RED 
          }
          else if(toSts[txx] == TURNOUT_FACING_ACTV_STS)
          {
            entryPerm[blkX][txx] = LOW_ENTRY_PERMIT_LVL;
            entryAspect[blkX][txx] = CLEAR_TO_SLOW;         // YLW,YLW
          }
          else    // NO TURNOUT ACTIVE AHEAD
          {       // IF LIMITED ENTRY PERMIT ON BLOCK AHEAD, OR THIS IS SIDING THEN LIMIT PERMIT LEVEL
            if(adjPermit[txx] < MAX_ENTRY_PERMIT_LVL || trackTyp != MAINLINE_TRACK_TYP)
            {
              entryPerm[blkX][txx] = MED_ENTRY_PERMIT_LVL;
              entryAspect[blkX][txx] = CLEAR_TO_MEDIUM;         // YLW,GRN
            }
            else
            {
              entryPerm[blkX][txx] = MAX_ENTRY_PERMIT_LVL;
              entryAspect[blkX][txx] = CLEAR;         // GRN, RED
            }
          }            
        }
        else if(blockET[blkX] >= 4)   // ITS BEEN UNOCC for at least 4 Sec
        {
          if(toSts[txx] == TURNOUT_RVS_ACTV_STS)
          {
            entryPerm[blkX][txx] = LOW_ENTRY_PERMIT_LVL;
            entryAspect[blkX][txx] = RESTRICTING;         // RED,YLW
          }
          else 
          {
            entryPerm[blkX][txx] = MED_ENTRY_PERMIT_LVL;
            entryAspect[blkX][txx] = SLOW_TO_CLEAR;         // RED,GRN
          }
        }  
      } //  end for txx -loop to set permit & aspects for each end of this block
    }
    updatePermitTags(blkX);     // Create TAG Names & update with curr Blk Entry Permit values
  }
  ++blkX;     // INCR INDEX FOR NEXT TIME
}

/************************************************************/
byte getBlockOccSts(byte blkX) { return blkOccSts[blkX]; }

/************************************************************/
byte getBlkLastTrainDir(byte blkX) { return trainDir[blkX]; }

/************************************************************/
unsigned getBlockEntryApsect(byte blkX, byte rtEnd) { return entryAspect[blkX][rtEnd ? RT_INPUT_NDX : LF_INPUT_NDX]; }

/************************************************************/
unsigned getBlockEntryPermit(byte blkX, byte rtEnd) { return entryPerm[blkX][rtEnd ? RT_INPUT_NDX : LF_INPUT_NDX]; }

/************************************************************/
unsigned getBlockET(byte blkX) { return blockET[blkX]; }

/************************************************************/
unsigned getBlockNumTrains(byte blkX) { return blockDefn[blkX].numTrains; }

/************************************************************/
byte getTurnoutSts(byte blkX, byte toX) { return turnoutSts[blkX][toX]; }

/************************************************************/
void updatePermitTags(byte blkX)
{
byte dirX, tagX;  
  cpStr(blockDefn[blkX].idStr, wrkStr, NAME_LEN);
  wrkStr[NAME_LEN]=0;
  wrkStrLen = strlen(wrkStr);   // Determine how long idStr is
  wrkStr[wrkStrLen++] = '-';    // append suffix starting with '-'
  dirX = wrkStrLen;             // sav index where Compass Letter gets put
  for(byte dxx=0; dxx<2 ;dxx++)
  {
    wrkStrLen = dirX;           // reset to Compass letter index
    if(dxx)     // RT
    {
      if(blockDefn[blkX].compassLeft == 'W') wrkStr[wrkStrLen++] = 'E';
      else if(blockDefn[blkX].compassLeft == 'N') wrkStr[wrkStrLen++] = 'S';
      else if(blockDefn[blkX].compassLeft == 'S') wrkStr[wrkStrLen++] = 'N';
      else wrkStr[wrkStrLen++] = 'W';
    }
    else        // LF
      wrkStr[wrkStrLen++] = blockDefn[blkX].compassLeft;
    wrkStr[wrkStrLen] = 0;      // TERMINATE TAG NAME STRING
    
    tagX = createTag(wrkStr, INT_TAG_TYP);          // UPDATE TAGS - createTag returns Tag Index
    setTagVal(tagX, entryPerm[blkX][dxx]);           // Write current ENTRY PERMISSIVE TAG
  }
}
                            //////////////////////////////////////////////////////////////////////////
                            //  CONFIGURATION PARAMETER ACCESS
                            ////////////////////////////////////////////////////////////////////////// 
/************************************************************/
void initBlocks()
{ byte x,xx;
  for(x=0; x<NUM_BLOCKS ;x++)    // initialize all blocks as Empty
  {
    blockDefn[x].idStr[0]=0; 
    blockDefn[x].compassLeft=0;
    blockDefn[x].flags=0;
    blockDefn[x].turnoutCnt=0;
    blockDefn[x].adjBlkID[0][0]=0;
    blockDefn[x].adjBlkID[1][0]=0;
  }
  blkTOsec = DFLT_BLOCK_TO_SEC;
  if(readBlocksCfg())                // FETCH CONFIG from EEPROM
  {
    blkTOsec = blockDefn[0].spare;
  } 
  for(x=0; x<NUM_BLOCKS ;x++)     // initialize RT vars
  { 
    blkOccSts[x]=0; entryPerm[x][0]=0; entryPerm[x][1]=0; trainDir[x]=0;
        // INIT NEXT BLOCK PERMIT = MAX. GETS OVERRIDDEN AT RT, BUT IF NOT CONNECTION
        //   THEN SIGNALS INTO THAT TERRITORY STILL COME OFF STOP
    adjEntryPerm[x][0] = adjEntryPerm[x][1] = MAX_ENTRY_PERMIT_LVL; 
    lastDetSts[x][0]=0; lastDetSts[x][1]=0;
    for(xx=0; xx<MAX_BLOCK_TURNOUTS ;xx++) turnoutSts[x][xx]=0; 
  }
  permitCycleCnt = 0;
      // CREATE 1 BLOCK for Demonstration if None is configured
  if(blockDefn[0].idStr[0]<'A')     // BLOCK [0] Undefined
  { addBlock(0, 1, 'W', "BMRC_123");
  }
}

/************************************************************
 *  addBlock
 ************************************************************/
byte addBlock(byte lfDetX, byte rtDetX, byte leftCompass, char *blkID)
{    
byte bx;
  for(bx=0; bx<NUM_BLOCKS && blockDefn[bx].idStr[0]>' ' ;bx++);
  if(bx<NUM_BLOCKS && blkID[0]>' ') 
  {
    blockDefn[bx].detectNdx[LF_INPUT_NDX] = lfDetX;
    blockDefn[bx].detectNdx[RT_INPUT_NDX] = rtDetX;
    blockDefn[bx].flags = 0;
    blockDefn[bx].turnoutCnt=0;
    blockDefn[bx].adjBlkID[LF_INPUT_NDX][0] = 0;
    blockDefn[bx].adjBlkID[RT_INPUT_NDX][0] = 0;
    if(leftCompass=='E' || leftCompass=='N' || leftCompass=='S' || leftCompass=='W')
      blockDefn[bx].compassLeft = leftCompass;      // should be a letter {W,E,S,N}
    else 
      blockDefn[bx].compassLeft = 'W';
    cpStr(blkID, blockDefn[bx].idStr, NAME_LEN); 
    return 1;
  } 
  return 0;
} 

/************************************************************/
byte blockDefined(byte blkX) { return (blockDefn[blkX].idStr[0]>='A'); }  

/************************************************************/
char *getBlockID(byte blkX) { return blockDefn[blkX].idStr; } 

/************************************************************/
byte getBlockDetNdx(byte blkX, byte rtNdx) { return blockDefn[blkX].detectNdx[rtNdx ? RT_INPUT_NDX:LF_INPUT_NDX]; }  

/************************************************************/
char *getAdjacentBlk(byte blkX, byte rtSel) { return blockDefn[blkX].adjBlkID[rtSel?1:0]; }

/************************************************************/
byte openEndedBlock(byte bx) { return (blockDefn[bx].detectNdx[RT_INPUT_NDX] > NUM_DETECTORS); }

/************************************************************/
void setBlockOpt(byte bx, byte optFlag) { if(bx<NUM_BLOCKS) blockDefn[bx].flags |= optFlag; }

/************************************************************/
void clrBlockOpt(byte bx, byte optFlag) { if(bx<NUM_BLOCKS) blockDefn[bx].flags &= ~optFlag; }

/************************************************************/
byte getBlockOpt(byte bx, byte optFlag) { return blockDefn[bx].flags & optFlag; }

/************************************************************/
char getBlockEndCompass(byte blkX, byte rtEnd) 
{
  if(rtEnd)
  {
    if(blockDefn[blkX].compassLeft == 'E') return 'W';
    if(blockDefn[blkX].compassLeft == 'N') return 'S';
    if(blockDefn[blkX].compassLeft == 'S') return 'N';
    return 'E';
  } 
  return blockDefn[blkX].compassLeft;  
}

/************************************************************/
float getBlockEndMP(byte blkX, byte rtEnd) 
{
byte dx = blockDefn[blkX].detectNdx[rtEnd ? 1:0];
  return (getDetectorMP(dx));
}

/************************************************************/
byte findBlockNdx(char *idStr) 
{
byte bx;
  for(bx=0; bx<NUM_BLOCKS ;bx++)
  {
    if(strCmpr(idStr,  blockDefn[bx].idStr))   return bx; 
  }
  return 255;
} 

/***********************************************************
 *  selectBlkEnd(byte blkX, float mp)     -Signals may not be exactly at Detector Mile Post
 *        so this func determines which end it is closest to.
 *        Rtns 1 if closes to Right End, 0 if closest to Left End, 255 if Error
 */
byte selectBlkEnd(byte blkX, float mp)
{
float diffL, diffR, dMP;
byte dx;
  if(blkX<NUM_BLOCKS && blockDefn[blkX].idStr[0]>' ')   // valid block
  {
    dMP = getDetectorMP(blockDefn[blkX].detectNdx[LF_INPUT_NDX]);
    if(dMP>=mp) diffL = dMP-mp; else diffL = mp-dMP;
    dMP = getDetectorMP(blockDefn[blkX].detectNdx[RT_INPUT_NDX]);
    if(dMP>=mp) diffR = dMP-mp; else diffR = mp-dMP;
    if(diffL > diffR) return 1;       // its closest to Right End
    return 0;
  }
  return 255;
} 

/************************************************************/
byte setAdjacentBlk(byte blkX, byte rtSel, char *adjID) 
{ 
  if(blkX<NUM_BLOCKS)
  {  
    cpStr(adjID, blockDefn[blkX].adjBlkID[rtSel?1:0], NAME_LEN); 
    return 1;
  }
  return 0;
}

/************************************************************/
byte addTurnout(byte blkX, byte inputX, byte lowActv, byte rtFacing)
{
byte tx;  
  if(blkX<NUM_BLOCKS)     // valid Block Index
  { 
    tx = blockDefn[blkX].turnoutCnt;            // get curr count in lower 4 bits
    if(tx < MAX_BLOCK_TURNOUTS)                               // limit test
    {
      blockDefn[blkX].turnoutNdx[tx] = inputX;      // Configure Input this new Turnout
      if(lowActv)  blockDefn[blkX].turnoutNdx[tx] |= TURNOUT_LOW_ACTV_FLG;  // Set Low Actv Flag in thie Index Hi Bits
      if(rtFacing) blockDefn[blkX].turnoutNdx[tx] |= TURNOUT_RT_FACING_FLG; // Set RT-FACING Flag in Hi Order Bits
      blockDefn[blkX].turnoutCnt = (tx+1);                           // update Cnt
      return 1;
    }
  }
  return 0;
}

/************************************************************/
byte getNumTurnouts(byte blkX) { return blockDefn[blkX].turnoutCnt; }

/************************************************************/
byte getTurnoutInput(byte blkX, byte tx) { return (blockDefn[blkX].turnoutNdx[tx] & TURNOUT_NUM_MASK); }

/************************************************************/
byte getTurnoutRightFacing(byte blkX, byte tx) { return (blockDefn[blkX].turnoutNdx[tx] & TURNOUT_RT_FACING_FLG); }

/************************************************************/
byte getTurnoutLowActv(byte blkX, byte tx) { return (blockDefn[blkX].turnoutNdx[tx] & TURNOUT_LOW_ACTV_FLG); } 

/************************************************************/
byte delBlock(char *idStr) 
{
byte bx;
  for(bx=0; bx<NUM_BLOCKS ;bx++)
  {
    if(strCmpr(idStr,  blockDefn[bx].idStr))   
    {
      blockDefn[bx].idStr[0]=0;
      return 1;
    }
  }
  return 0;
} 

/************************************************************/
byte getNumBlocks() 
{ 
byte cnt=0;
  for(byte bx=0; bx<NUM_BLOCKS ;bx++)
  { if(blockDefn[bx].idStr[0]>' ') ++cnt;
  }
  return cnt;
}

/************************************************************/
byte getMaxBlocks() { return NUM_BLOCKS; }

/************************************************************/
void setBlockTimeout(unsigned toSec) { blkTOsec = boundsChk(toSec, MIN_BLOCK_TO_SEC, MAX_BLOCK_TO_SEC); }
unsigned getBlockTimeout() { return blkTOsec; }

/**************************************************
 * saveSensorCfg()      
 **************************************************/ 
void saveBlocksCfg()         
{
byte chk = CFG_INTEGRITY_CHK_VAL;
  EEPROM.write(BLOCKS_EEPROM_ADDR, chk);   
  blockDefn[0].spare = blkTOsec;
  EEPROM.put((int)(BLOCKS_EEPROM_ADDR+1), blockDefn); 
  Serial.print(F("BLOCK CFG SAVED @"));Serial.print(BLOCKS_EEPROM_ADDR);Serial.print(F(" - "));
    Serial.println(sizeof(blockDefn)+BLOCKS_EEPROM_ADDR);
}

/**************************************************
 * readSensorCfg()      -Rtns 1 if REad OK, else 0
 **************************************************/ 
byte readBlocksCfg()
{ 
byte chk = EEPROM.read(BLOCKS_EEPROM_ADDR);
  if(chk != CFG_INTEGRITY_CHK_VAL) return 0;
  EEPROM.get((int)(BLOCKS_EEPROM_ADDR+1), blockDefn);
  return 1;
}
