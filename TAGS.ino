/*******************************************************************************************
 *  TAGS  Module      -Post Office for Tagged Data. Tag Data does not need to be saved
 *                      All tags get created at RT as needed
 *******************************************************************************************/
#ifndef NUM_TAGS
#define NUM_TAGS          10
#endif  


#define TAG_TYP_MASK      0x0F 

static struct TAG_TABLE {
  char tagNam[TAGNAME_LEN];
  byte flags,
      hr,mn,sec; 
  int tagV; 
} tagTable[NUM_TAGS];
byte numTags;

/*********************************************
 *  initTags()    -Clear Tag Table
 */ 
void initTags() 
{ 
  numTags = 0;
  for(byte x=0; x<NUM_TAGS ;x++) 
  {
    tagTable[x].tagNam[0]=0; 
    tagTable[x].flags = 0; 
  } 
}

/*********************************************
 *  createTag()   -Adds TAG with tagName + Direction
 */ 
byte createTag(char *tagName, byte tagTyp)
{
byte tx, cx;
  for(tx=0; tx<NUM_TAGS ;tx++)                            // PREVENT DUPLICATE
  { if(strCmpr(tagName, tagTable[tx].tagNam)) return tx;
  }
  for(tx=0; tx<NUM_TAGS && tagName[0]>' ' ;tx++)
  { 
    if(tagTable[tx].tagNam[0]<=' ')         // Found unused Table Entry - put new tag here
    { 
      ++numTags;
      for(cx=0; cx<NAME_LEN && tagName[cx]>' ' ;cx++)     // copy tagName and terminate
      { tagTable[tx].tagNam[cx]=tagName[cx];
      }
      if(cx<NAME_LEN) tagTable[tx].tagNam[cx]=0;  
      tagTable[tx].flags = tagTyp & TAG_TYP_MASK; 
      return tx;
    }
  }
  return 255;
}

/*********************************************
 *  lookupTag()   -Rtns tagTable[] Index of tagName
 *                 or 255 if not found
 */ 
byte lookupTag(char *tagName)
{
byte tx;
  for(tx=0; tx<NUM_TAGS ;tx++)
  { if(strCmpr(tagName, tagTable[tx].tagNam)) return tx;
  }
  return 255;
}

/**********************************************/ 
char *getTagName(byte tagNdx)   { return tagTable[tagNdx].tagNam; }

/**********************************************/ 
byte tagDefined(byte tagNdx) { return (tagNdx<NUM_TAGS && tagTable[tagNdx].tagNam[0]>' ') ? 1:0; }

/**********************************************/ 
byte getTagTyp(byte tagNdx) { return (tagTable[tagNdx].flags & TAG_TYP_MASK); } 

/**********************************************/ 
int  getTagVal(byte tagNdx)  { return tagTable[tagNdx].tagV; } 

/**********************************************/ 
byte getTagTimeStamp(byte tx, byte timSel) 
{
  if(timSel==2)  return tagTable[tx].hr;
  if(timSel==1)  return tagTable[tx].mn;
  return tagTable[tx].sec;
}

/**********************************************/ 
char *getTagTimeStr(byte tx)
{
static char tStr[16]; 
  sprintf(tStr,"%02u:%02u:%02u",tagTable[tx].hr,tagTable[tx].mn,tagTable[tx].sec);
  return tStr;
}

/**********************************************
 *  setTagVal()    Sets value  
 **********************************************/ 
byte setTagVal(byte tagNdx,int newV) 
{ 
  if(tagNdx<NUM_TAGS)
  {
    tagTable[tagNdx].tagV = newV; 
    tagTable[tagNdx].hr  = getClockHr(); 
    tagTable[tagNdx].mn  = getClockMin(); 
    tagTable[tagNdx].sec = getClockSec(); 
    return 1;
  }
  return 0;
} 

/**********************************************/ 
void deleteTag(byte tagNdx) 
{ 
  if(tagNdx<NUM_TAGS) 
  {
    if(tagTable[tagNdx].tagNam[0]>' ' && numTags)      // was a Valid Tag
      --numTags;
    tagTable[tagNdx].tagNam[0]=0; 
  }
}

/**********************************************/ 
byte getNumTags() { return numTags; }

/**********************************************/ 
byte getTagTblSiz() { return NUM_TAGS; }
