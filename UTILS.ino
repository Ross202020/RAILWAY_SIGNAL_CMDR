/*******************************************************************************************
 *  UTILS
 *******************************************************************************************/  

          /////////  TIMING
#ifndef NUM_TIMERS
#define NUM_TIMERS    4
#endif
#ifndef NUM_ET_TIMERS
#define NUM_ET_TIMERS    4
#endif
unsigned msTimer[NUM_TIMERS], clkms;
byte clkSec, clkMin, clkHr, _tx;

void startTimer(byte tx, unsigned  dly) { if (tx<NUM_TIMERS) msTimer[tx] = dly; }
byte timerExpired(byte tx) { return (msTimer[tx] ? 0:1); }    

///////////////////////////////////////////////////////////
// clockTick()    - Run every ms
///////////////////////////////////////////////////////////
void clockTick()
{ 
  if(++clkms>=1000) 
  { 
    clkms=0; 
    if(++clkSec>=60) { clkSec=0; if (++clkMin>=60) { clkMin=0; if(++clkHr>=24) { clkHr=0; } } }    
  }  
  for(_tx=0; _tx<NUM_TIMERS; _tx++) { if(msTimer[_tx])msTimer[_tx]--; }      // DECR All ms Timers every ms
} 
 
void set_Time(byte h, byte m, byte s)  
{ 
  if (h<24) clkHr=h; 
  if (m<60) clkMin=m; 
  if (s<60) clkSec=s; 
  clkms=0;
} 

byte getClockSec()    { return clkSec;    }
byte getClockMin()    { return clkMin;    }
byte getClockHr()     { return clkHr;     } 
byte getHalfSec()     { return (clkms>=500);}
unsigned getClockms() { return clkms;     }
static char tstr[16];
char *timeStr() { sprintf(tstr,"%02u:%02u:%02u.%03u",clkHr,clkMin,clkSec,clkms); return tstr; } 


          //////////////////////////////////////////////////////////////
          //  FLAG FUNCTIONS -16 Flags in small footprint (1 unsigned)
          //  EACH FLAG is a single bit value in a 16 bit register 
          // First 4 Examples
          //    #define APP_FLAG1 0x0001
          //    #define APP_FLAG2 0x0002
          //    #define APP_FLAG3 0x0004
          //    #define APP_FLAG4 0x0008  
          //////////////////////////////////////////////////////////////
static unsigned _Flgs;
void setFlag(unsigned flagV) { _Flgs |= flagV; } 
void clrFlag(unsigned flagV) { _Flgs &= ~flagV; } 
inline unsigned tstFlag(unsigned flagV) { return (_Flgs & flagV); }
void toggleFlag(unsigned flagV) { if (_Flgs & flagV) _Flgs &= ~flagV;  else  _Flgs |= flagV;  }

          //////////// DEBUG PRINT UTILS 
void prtCh(byte chv)    { Serial.print((char)chv); } 
void prtSP(byte nReps)  { for (byte cnt = 0; cnt < nReps ; cnt++) Serial.print(' '); }
void prtSP()            { Serial.print(' '); }
void prtNdx(int ndx)    { Serial.print('[');Serial.print(ndx);Serial.print(']'); }
void prtPin(int ndx)    { Serial.print('(');Serial.print(ndx);Serial.print(')'); } 
void prtRtn()           { Serial.println(); }

          //////////////////////////////////////////////////////////////
          // NUMERIC    -bounds check,
          //////////////////////////////////////////////////////////////
int boundsChk(int v, int loBnd, int hiBnd) { if(v<loBnd)return loBnd;if(v>hiBnd)return hiBnd;return v; }

///////////////////////////////////////////////////////
//  getFld()    -Extracts sub field,Rtns Length if successful
//               opst & 1 == UPRCASE,  opts & 2 == INC_DEC_PT for floats w decimal point
//               Treats everything within quotes (" ") as 1 field, regardless of content
///////////////////////////////////////////////////////
byte getFld(byte selFld, char *src, char *dest, byte maxLen, byte opts)
{
static byte inFld, fCnt, sx, fx, quoteCnt, 
  upCase,             // Converto to Upper Case Flag
  dnd,                // Dot Not a Delimiter Flag
  aslash,             // allow Slash in fld
  fltFldOpt;          // Include Dot in fld
char ch;          
  dest[0] = 0;    // Make sure previous fld content is erased
  upCase = opts & 1;
  fltFldOpt = opts & 2;
  dnd = opts & 4;
  aslash = opts & 8;  

  for (sx=inFld=fCnt=0; sx<80 ;sx++)          // SEARCH FOR START OF SELECTED FIELD
  {
    ch = src[sx];
    if (ch<' ') break;      // QUIT AT END OF LINE
    
    if (!inFld)             // NOT YET IN A FIELD
    {
      if((ch>='A' && ch<='Z') || (ch>='a' && ch<='z')) inFld = 1;
      else if((ch>='0' && ch<='9') || (ch=='-' && src[sx+1]<='9' && src[sx+1]>='0')) inFld=2;
      else if(ch=='\"') inFld=4;       

      if(inFld && ++fCnt==selFld)     // THIS IS THE ONE WE ARE LOOKING FOR - EXTRACT IT NOW
      {                               //  Copy it to dest[]
        if(inFld==4)                  // skip leading '"' for Quoted Field
          ++sx;
        
        for (fx=quoteCnt=0; fx<maxLen ; sx++)
        {
          ch = src[sx];                     // fetch next ch from src[]         
          if (ch<' ' || ch=='\"')           // QUIT AT END OF src[] or encountering a Quote
            break;
                  // TEST FOR END DELIMITER
          if(fx)                // 2nd and later char - 1st always gets in except for inFld==4
          {
            if(inFld==1)    // Name Field - can include numbers and optionally '/'
            {
              if(ch=='/')
              { if(!aslash) break;
              }
              else if(ch<'A' && (ch<'0' || ch>'9')) break; 
              else if(ch>'Z' && ch!='_' && (ch<'a' || ch>'z')) break;
            }
            else if(inFld==2)
            {
              if(ch=='.')     // can accept 1 Decimal if  fltFldOpt
              { if(!fltFldOpt || ++quoteCnt > 1) break;
              }
              else if(!isDigit(ch)) break;
            }
          }
          dest[fx++] = upCase ? toupper(ch) : ch;
        }  // end of copy loop
        dest[fx] = 0;
        return fx;        
      } //  end if(inFld && ++fCnt==selFld) 
    }  // end of not yet in fld
    else    // MUST BE IN A FIELD
    {       //  Test for field ending delimiter
      if (ch<=' ' || ch==',' || ch==':' || (ch=='.' && !dnd) || (ch=='/' && !aslash) || ch=='\\' || ch==')' || ch==']' || ch=='}')
        inFld = 0;
    }
  } 
  return 0;
} 

///////////////////////////////////////////////////////
byte strCmpr(char *str1, char *str2)
{
byte cmpx;
  for (cmpx=0; str1[cmpx]==str2[cmpx] && str1[cmpx]>=' ' && cmpx<80 ;cmpx++);
  if (cmpx>0 && str1[cmpx]<' ' && str2[cmpx]<' ')       // Compare reached end of both str
    return cmpx;
  return 0;  
}

///////////////////////////////////////////////////////
byte cpStr(char *src, char *dest, byte maxLen) 
{ byte x; for (x=0; x<maxLen && src[x]>=' ' ;x++) dest[x]=src[x]; if(x<maxLen)dest[x]=0; }

/*******************************************************
 *  findSubStr()     -Rtns Index of modelStr in bigStr, OR 255 if not found
 */
byte findSubStr(char *modelStr, char *bigStr)
{
byte sx,cx, fndX;
  for(sx=0; sx<100 && bigStr[sx]>=' ' ;sx++)
  {
    if(bigStr[sx]== modelStr[0])    // 1st char matches - proceed
    {
      fndX = sx;                            // NOTE LOCATION OF POTENTIAL MATCH
      ++sx;                                 // Move index to next char in bigStr[]
      for(cx=1 ; cx<32 && modelStr[cx]>=' ' && bigStr[sx]>=' ' ;cx++,sx++)     // Compare Loop
      { if(bigStr[sx] != modelStr[cx]) break;  }                                // Quit on miscompare
      if(modelStr[cx] < ' ')                // Test for Success - did it reach end of modelStr ?
        return fndX;
    }
  }
  return 255;
}

/*******************************************************
 *  getFldNdx     -Returns Offset Index where selected field begins, else 255
 *                 treats these same as SP {'=',',', '@','&','(',')','/','\\'}
 */
byte getFldNdx(byte fldNum,char *src)
{
byte sx, inFld, fldCnt;   char c;
  for(sx=inFld=fldCnt=0; sx<80 ;sx++)
  {
    c = src[sx];
    if(c<' ') break;        // Quit at end of src[] 
        
    if(c==' ' || c=='=' || c==',' || c=='@' || c=='(' || c==')' || c=='\\' ||c=='/' || c=='&') 
    {     // This is Delimiter - Not in a Fld
      inFld=0;
    }
    else     // Its a valid Field char
    { 
      if(!inFld)
      {
        if(++fldCnt==fldNum) return sx;
        inFld = 1;
      }
    }
  }
  return 255;
}
