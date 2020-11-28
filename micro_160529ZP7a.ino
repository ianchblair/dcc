/*
  DCC Decoder for Z gauge Ardiono Micro (German/two aspect signalling (no fours!))
  Encoder addresses are assumed to be 0 to 127.
  Points are in range 0 to 31
  Signals from 32 to 63 
  switches from  64 to 79 (less overlap with 2-aspect signals)
  Smartswitch drivers are 80 to 83 inclusive
  84 to 127 range is currently reserved
  Extended and basic acessories are assumed to overlap, although this might change!
*/
 
// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
int led = 13;
// Mappings to Arduino digital ports
//
// On the micro, Int4 is on D7 (used int is D2 on the Mega)
const int dcc_input = 7;
const int dcc_intrpt = 4;
// const int dcc_intr = digitalPinToInterrupt(dcc_input);
// switches are D4 to D7, then D18 and D18
const int sw0 = 3;
const int sw1 = 4;
const int sw2 = 5;
const int sw3 = 6;
const int sw4 = 18;
const int sw5 = 19;

//
// Signal types
//
const int st_sig2a = 2;


// 2-aspect signals are 
// sig2a1 14 (nearest bottom / USB) 
// sig2a2 15
// sig2a3 16
// sig2a4 33 (nearest top)
const int sig2a1 = 14;
const int sig2a2 = 15;
const int sig2a3 = 16;
const int sig2a4 = 17;


typedef struct sigstruct {
  int saddr;
  int stype; //simply number of aspects
  int saux1;
  int saux2;
} t_sigstruct;

// signal table
// for each signal we have the base port ID, number of aspects
// and aux pin IDs
t_sigstruct sigs[] = {
//  {sig4a1, st_sig4a, 0, 0},
//  {sig4a2, st_sig4a, sig4a2j, 0},
//  {sig4a3, st_sig4a, 0, 0},
//  {sig4a4, st_sig4a, 0, 0},  
  {sig2a1, st_sig2a, 0, 0},
  {sig2a2, st_sig2a, 0, 0},
  {sig2a3, st_sig2a, 0, 0},
  {sig2a4, st_sig2a, 0, 0}
};

int sigId;
int aspId;
int swId;


// point mux addresses are D8 to D11
int pmux0 = 8;
int pmux1 = 9;
int pmux2 = 10;
int pmux3 = 11;

// point mux selext is D12
int pmux = 12;
//
int pcount;
// Note that D5 is taken by the clock (OC3A)
//
// point mux lookup
int muxSeq[] = {7,6,5,4,3,1,2,0,13,12,15,14,9,8,11,10};

char serialchar;

// Here for device stuff
//
// Command tick is 10ms (prescaler = 64)
#define CMD_TICK_PERIOD 1250
byte cmdTickTimerReady;
// Actual timer count value
unsigned int cmdTickTimer;
unsigned int cmdTickTimeout;
//
// Timer values
// Point timer 200ms
#define T_POINT_PULSE 20
// Recharge timer 1 second
#define T_RECHARGE 100
// For smartswitch we will go for 250ms
#define T_SMARTSW_PULSE 25
//
//
typedef enum cmd_state
{
  C_IDLE = 0,
  C_POINT_PULSE,
  C_POINT_RECHARGE,
  C_SMARTSW_PULSE
} cmd_state;

cmd_state cmdState;

//
typedef enum dev_type
{
  D_UNKNOWN = 0,
  D_SIGNAL,
  D_POINT,
  D_SMARTSW,
  D_SWITCH
} dev_type;

typedef struct dec_msg
{
  int decAddress;
  int decValue;
} dec_msg;

typedef struct dec_msg_typed
{
  dev_type decDevType;
  int decAddress;
  int decValue;
} dec_msg_typed;

// Buffer and pointer for received messages for us
const byte DecBufLen = 16;
dec_msg decBuf[DecBufLen];
dec_msg * decBufIn;
dec_msg * decBufOut;
dec_msg_typed currentMsg;  // The current message that we are dealing with from buffer

// Here for DCC stuff

// Largest message we care about is an extended one (four bytes)
const byte MaxDccMsgLen = 4;

//
typedef enum dcc_rx_wait
{
  W_PREAMBLE = 0,
  W_START,
  W_DATA,
  W_END
} dcc_rx_wait;

typedef struct dcc_msg
{
  byte Data[MaxDccMsgLen];
  byte Size;
} dcc_msg;


// buffer for incoming command
// With state bits
volatile struct DccRx
{
  dcc_rx_wait rxState;
  byte  DataReady;
  byte  BitCount ;
  byte  TempByte ;
} DccRx;

volatile dcc_msg MsgIn;
volatile dcc_msg MsgOut;

volatile unsigned int timerval;
volatile unsigned int timerdiff;
volatile unsigned int timertmp;
volatile unsigned int timesince;
volatile boolean testbit;
volatile boolean parity;

// Stuff from outside



// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  pinMode(sw0, OUTPUT);
  pinMode(sw1, OUTPUT);
  pinMode(sw2, OUTPUT);
  pinMode(sw3, OUTPUT);
  pinMode(sw4, OUTPUT);
  pinMode(sw5, OUTPUT);
  pinMode(sig2a1, OUTPUT);
  pinMode(sig2a2, OUTPUT);
  pinMode(sig2a3, OUTPUT);
  pinMode(sig2a4, OUTPUT);
  pinMode(pmux0, OUTPUT);
  pinMode(pmux1, OUTPUT);
  pinMode(pmux2, OUTPUT);
  pinMode(pmux3, OUTPUT);
  pinMode(pmux, OUTPUT);
  pinMode(led, OUTPUT);
  pinMode(dcc_input, INPUT_PULLUP);
  digitalWrite(pmux,HIGH);
  // Clear data ready flag and count, and set start state to W_PREAMBLE before enabling dcc interrupt
  // (Arduino might pre-initialise, but let's not take chances!)
  DccRx.DataReady = 0;
  DccRx.BitCount = 0;
  DccRx.rxState = W_PREAMBLE;
  parity = LOW;
  
  // Buffer & fsm init
  decBufIn = &decBuf[0];
  decBufOut = &decBuf[0];
  cmdState = C_IDLE;
  cmdTickTimer = 0;
  cmdTickTimeout = 0;
  //
  // Enable timer 3 with prescale of 64
//  but first disable interrupts to prevent regsiter loads misbehaving
  cli();
// Note: Prescale of 64 is 100 for timer 2, 011 for all others (check for Genuino micro!)
// All are TCCRxB
  TCCR3A=0;
  TCCR3B=0;
  TCCR3C=0;  
  TCCR3A |= (1<< COM3A0);
  TCCR3A |= (1<< COM3A1);
// (Options for timer 2 instead...)  
//  TCCR2B &= ~(1<< CS20);
//  TCCR2B &= ~(1<< CS21);
//  TCCR2B |= (1<< CS22);
  TCCR3B |= (1<< CS30);
  TCCR3B |= (1<< CS31);
//  TCCR5B &= ~(1<< CS52);
//  TCCR5B |= (1<<WGM52);  // enables CTC mode
// Enable timer 3 compare regsister & interrupt
  OCR3A = TCNT3 + (CMD_TICK_PERIOD-1);
  TIMSK3 |= (1<<OCIE3A) ; // Enable timer 3 compare type 1 interrupt
  TIFR3 |=  (1<<OCF3A) ; // Clear Timer 3 Compare Match A flag
// Release interrupt mask
    sei();
// Initial Interrupt setting
// This will be changed in the ISR itself
//  attachInterrupt(digitalPinToInterrupt(dcc_input), dcc_interrupt_handler, CHANGE );
  attachInterrupt(dcc_intrpt, dcc_interrupt_handler, CHANGE );
  Serial.begin(9600);   
}

// Dcc decoder
// Requires two ineterrupts. A rising edge on the DCC line starts a timer
// for 80us.
// The inverse level after this time gives the bit value (then repeat for next bit)
// DCC frames are collected and passed to the decoder
//
// prescaler value is set to 64
#define DCC_BIT_SAMPLE_PERIOD 18
//
// Either edge triggered ISR
void dcc_interrupt_handler (void)
{
  timertmp = TCNT3;
  timerdiff = timertmp-timerval;
  timerval = timertmp;
  parity = !parity;
  
  if (parity)
  {
    boolean DccBitVal;
    byte i;
    if (timerdiff < DCC_BIT_SAMPLE_PERIOD) DccBitVal = HIGH;
    else DccBitVal = LOW;  
    DccRx.BitCount++; //
  testbit = DccBitVal;
//  if (testbit == LOW)  DccRx.DataReady = 1 ; //Debug  
  // Determine next state
    switch ( DccRx.rxState )
    {
      case W_PREAMBLE:
        if (DccBitVal)
        {
          if (DccRx.BitCount > 10)
          { 
            DccRx.rxState = W_START;
          }
        }
        else DccRx.BitCount = 0 ; 
        break;
      
     case W_START:
        if (!DccBitVal)
        {
          DccRx.rxState = W_DATA;
          MsgIn.Size = 0;
          for (i=0;(i< MaxDccMsgLen); i++) MsgIn.Data[i] = 0;
          DccRx.BitCount = 0;
          DccRx.TempByte = 0;
        }
        break;
      
     case W_DATA:
        DccRx.TempByte = (DccRx.TempByte<< 1);
        if (DccBitVal) DccRx.TempByte |= 1;
        if (DccRx.BitCount == 8)
        {
          MsgIn.Data[MsgIn.Size++] = DccRx.TempByte;        
          if (MsgIn.Size <= MaxDccMsgLen)
          {
             DccRx.rxState = W_END;
          }
          else
          {
          // message too long - ignore and abort it
             DccRx.rxState = W_PREAMBLE ;
          }
          DccRx.BitCount = 0;                    
        }
        break; 
      
      case W_END:    // End of octet
      if (DccBitVal) //if end of message
      {
    // Start looking for new preamble
        DccRx.rxState = W_PREAMBLE ;
      //Don't bother with glitches (size 1 psckets)
        if (MsgIn.Size>1)
        {
    // should do this with a pointer!?
          for (i=0;(i< MaxDccMsgLen); i++) MsgOut.Data[i] = MsgIn.Data[i];
          MsgOut.Size = MsgIn.Size;
          DccRx.DataReady = 1 ;
        }     
      }
      else //if end of octet, more to come
      {
        DccRx.rxState = W_DATA ;      // Else look for next byte
      }
      DccRx.BitCount = 0;
      DccRx.TempByte = 0;
      break;
    }
  }
  // End of DCC edge ISR
}

// We use the bit timer, timer 3, comparator A,
// to get command timer ticks (10ms)
ISR(TIMER3_COMPA_vect)
{
  cmdTickTimerReady = 1;
  OCR3A = TCNT3 + (CMD_TICK_PERIOD-1);
  TIMSK3 |= (1<<OCIE3A) ; // Enable timer 3 compare type 1 interrupt
  TIFR3 |=  (1<<OCF3A) ; // Clear Timer 3 Compare Match A flag
}


// the loop routine runs over and over again forever:
void loop()
{
  unsigned int j;
  struct dec_msg nextMsg;
  // digitalWrite(sw0, LOW);    // turn the LED off by making the voltage LOW
  digitalWrite(sig2a1, HIGH);
  digitalWrite(sig2a2, HIGH);
  digitalWrite(sig2a3, HIGH);
  digitalWrite(sig2a4, HIGH);
  
  //delay(1000);               // wait for a second
  //  Serial.write(66); // Debug marker
  Serial.println("DCC Decoder v 0.1 micro");
  for (;;)
  {
    if (cmdState != C_IDLE)
    {
    // timer still to do - when expired...
      if (cmdTickTimer > cmdTickTimeout)
      {
         switch (cmdState)
         {
            case C_POINT_PULSE:
            {
               docmd (&currentMsg, false);
               cmdTickTimer = 0;
               cmdTickTimeout = T_RECHARGE; //
               cmdState = C_POINT_RECHARGE;
            }
            break;
            case C_POINT_RECHARGE:
            {
               cmdTickTimer = 0;
               cmdTickTimeout = 0; //
               cmdState = C_IDLE;
            }
            break;
            case C_SMARTSW_PULSE:
            {
               docmd (&currentMsg, false);
               cmdTickTimer = 0;
               cmdTickTimeout = 0; //
               cmdState = C_IDLE;
            }
            break;
            default:
            {
            // assert error functionality to add
            }
         }         
      }
    }
    else   // if not timed out
    {
       if (decBufIn != decBufOut)
       {
            // get waiting message (if we can)
           if (retrieve_dec_msg(&nextMsg))
           {
//              Serial.println("Out");
//              Serial.print(nextMsg.decAddress,HEX);
//              Serial.write(32); // Debug marker 
//              Serial.println(nextMsg.decValue,HEX);
//              Serial.write(32); // Debug marker 
              // Only do it if we haven't just done it!
              if (!((currentMsg.decAddress == nextMsg.decAddress) && (currentMsg.decValue == nextMsg.decValue)))
              {
                currentMsg.decAddress = nextMsg.decAddress;
                currentMsg.decValue = nextMsg.decValue;
                currentMsg.decDevType = D_UNKNOWN;  // device is unknown until we decode it initially
                docmd (&currentMsg, true);
                switch (currentMsg.decDevType)
                {
                  case D_POINT:
                  {
                     cmdTickTimeout = T_POINT_PULSE; //
                     cmdState = C_POINT_PULSE;
                     cmdTickTimer = 0;
                  }
                  break;
                  case D_SMARTSW:
                  {
                     // cmdTickTimeout = T_SMARTSW_PULSE; //
                     // cmdState = C_SMARTSW_PULSE;
                     // cmdTickTimer = 0;
                  }
                  break;                                                                        
                  default:
                  {
                       // D_POINT and possibly D_SMARTSW are the only cases we care about                  
                  }       
                  break;
               }
            }
         }
      }
    }
    // Here regardless of state
    if (DccRx.DataReady != 0)
    {
       if (accessory_decode(&nextMsg))
       {
          store_dec_msg (&nextMsg);
       }
       DccRx.DataReady = 0;        
    }
    if (cmdTickTimerReady != 0)
    {
      cmdTickTimer ++;
      cmdTickTimerReady = 0;     
    }
  } // of forever
} // of loop

// Routines to store and retrieve messages
// returns TRUE if successful, else messgage is discarded and FALSE is returned
boolean store_dec_msg(struct dec_msg * decMsg)
{
  boolean rv = false;
  int fullness;
  // If space
  fullness = (decBufOut-decBufIn)+DecBufLen;
  fullness &= DecBufLen-1;
  if (fullness < DecBufLen)
  {
    decBufIn->decAddress=decMsg->decAddress;
    decBufIn->decValue=decMsg->decValue;
    decBufIn++;
    if ((decBufIn - &decBuf[0]) == DecBufLen)  decBufIn = &decBuf[0];
    rv = true;
  }
  return(rv);
}

// returns TRUE if successful, else null message and FALSE is returned
boolean retrieve_dec_msg(struct dec_msg * decMsg)
{
  boolean rv = false;
  if (decBufOut!=decBufIn)
  {
    decMsg->decAddress=decBufOut->decAddress;
    decMsg->decValue=decBufOut->decValue;
    decBufOut++;
    if ((decBufOut - &decBuf[0]) == DecBufLen)  decBufOut = &decBuf[0];
    rv = true;
  }
  return(rv);
}

// Routine to do commands
// called with ponter to command, and start/end indicator for pulsed outputs
// start_nend == TRUE to start, else end 
void docmd (struct dec_msg_typed * decMsg, boolean start_nend)
{
  int accessId = decMsg->decAddress;
  int accessValue = decMsg->decValue;
  dev_type devType = decMsg->decDevType;
//      Serial.write(66);
        Serial.print(accessId, HEX);
        Serial.println(accessValue, HEX);
        if (accessId > 63)
        {
           devType = D_SWITCH;
          //we will put "normal" swtchable accessories here
          swId = (accessId-63);
          swset(swId,accessValue);
        }
        else if (accessId>31)
        {
          // here for signals
          devType = D_SIGNAL;
          sigId = accessId-32 ;
          sigset (sigId,accessValue);        
        }
        else
        {
// here for points
          devType = D_POINT;
          pointset(accessId, start_nend);  
          Serial.println("Set");
//          Serial.print(accessId,HEX);
//          Serial.write(32); // Debug marker 
//          Serial.print(accessValue,HEX);
//          Serial.write(32); // Debug marker 
//          Serial.write(start_nend+48);
//          Serial.write(32); // Debug marker 
        }  
   decMsg->decDevType = devType;
}

// Routine for switch outputs
//
void swset(int swidn, int swstate) 
{
  int baseaddr;
  swidn = constrain(swidn,0,5);
  Serial.print("SW");
  Serial.println(swidn, HEX);
// First switch outputs are sw0 to sw3, then sw4 and sw5 on 18 and 19
  if (swidn < 4) baseaddr = (sw0 + swidn);
  else baseaddr = swidn-4+sw4;
  switch (swstate&1)
  {
    case 0:
    {
       digitalWrite(baseaddr, HIGH);
    }
    break;
    case 1:
    {
        digitalWrite(baseaddr, LOW);   
    }
    break;
    default:
    {
     // assert error functionality to add
    }
  }
}

// Routine to look up a signal and set a specific aspect
// aux entries are for e.g junction indicators and are zero if not used
void sigset(int sigidn, int aspect) 
{
  int sigtype;
  int baseaddr;
  int sigaux1;
  int sigaux2;
  //constrain(sigid,0,7);
  //constrain(aspect,0,31);
  sigtype = sigs[sigidn].stype;
  baseaddr = sigs[sigidn].saddr;
  sigaux1 = sigs[sigidn].saux1;
  sigaux2 = sigs[sigidn].saux2;
  if (sigtype==st_sig2a)
  {
    switch (aspect&1)
    {
      case 0:
      {
         digitalWrite(baseaddr, HIGH);
      }
      break;
      case 1:
      {
          digitalWrite(baseaddr, LOW);   
      }
      break;
      default:
      {
       // assert error functionality to add
      }
    }
  }

  // Here for junction indicator
  // On for aspects with bit 2 set
  if (sigaux1 != 0)
  {
    if ((aspect & 0x4) == 0)
      digitalWrite(sigaux1, LOW);
    else 
      digitalWrite(sigaux1, HIGH);
  }
 // Here for other accessory (e.g. calling on light)
 // On for aspects with bit 3 set  
   if (sigaux2 != 0)
   {
     if ((aspect & 0x8) == 0)
       digitalWrite(sigaux2, LOW);
     else 
       digitalWrite(sigaux2, HIGH);
   }  
}

void pointset(int pointid, boolean start_nend)
{
  int pointval;
  int paddr;
  int pmuxsel;
  boolean p0;
  boolean p1;
  boolean p2;
  boolean p3;
  boolean el;
  boolean eh;
  
// Assume for now that left most setting is first
//  pointval = (pointid*2)+leftright;
  pointval = pointid;
// Set mux value
  paddr = muxSeq[(pointval & 0xF)];
  if ((paddr & 0x1) != 0) p0 = HIGH;
  else p0 = LOW;
  if ((paddr & 0x2) != 0) p1 = HIGH;
  else p1 = LOW;
  if ((paddr & 0x4) != 0) p2 = HIGH;
  else p2 = LOW;
  if ((paddr & 0x8) != 0) p3 = HIGH;
  else p3 = LOW;
  digitalWrite(pmux0, p0);
  digitalWrite(pmux1, p1);
  digitalWrite(pmux2, p2);
  digitalWrite(pmux3, p3);
  
// Then enable for 200ms
  if (start_nend)
  {
     digitalWrite(pmux,LOW);
     digitalWrite(led, HIGH);
  }
//  delay(200);
  else
  {
     digitalWrite(pmux,HIGH); 
     digitalWrite(led, LOW);
  }
}


// Accessory decoder
// Decodes dcc accessory commands, and ignores all others
// Returns non zero if valid accessory command found, 
// with accId and acValue set according
// to message contents

boolean accessory_decode(dec_msg * decMsg)
{
  boolean rv = false;
  byte chksm = 0;
  byte i;
  unsigned int addr=0;
  unsigned int aval=0;
//  if(MsgOut.Size > 0)
//  {
//  Serial.println(MsgOut.Data[0],HEX);
//  Serial.write(32); // Debug marker
//  }  
//  Serial.print(MsgOut.Data[1],HEX);
//  Serial.write(32); // Debug marker  
//  Serial.print(MsgOut.Data[2],HEX);
//  Serial.write(32); // Debug marker  
//  Serial.print(MsgOut.Data[3],HEX);
//  Serial.write(32); // Debug marker  
//  Serial.println(MsgOut.Size);
//  }
//  if (MsgOut.Data[0] == 0x88)
//  {
//    for (i=0; i< (MsgOut.Size); i++)
//    {
//      Serial.print(MsgOut.Data[i],HEX);
//    }
//  }
  for (i=0; i< (MsgOut.Size-1); i++)
  {
    chksm = (chksm ^ MsgOut.Data[i]);
  }
  if (chksm ==  MsgOut.Data[(MsgOut.Size-1)])
  {
 //    Serial.print(MsgOut.Data[0],HEX);
 //    Serial.write(32); // Debug marker 
    // Accessory decoders begin 0b10....
    if ((MsgOut.Data[0] & 0xC0) == 0x80)
    {
      // MSBs from first two blocks of "A" bits
      addr = (MsgOut.Data[1] & 0x70) <<2;
      addr |= (MsgOut.Data[0] & 0x3F);
      // We invert the MS 3 bits, before further stuff (by "convention"!)   
      addr ^= 0x1C0; 
    
      // Next octet begins with 0 for extended address
      if ((MsgOut.Data[1] & 0x80) == 0x80)
      {
        Serial.println("Acc: "); // Debug marker
      // Here for Acc decoder - subtract 1 from board address, the append D bits
      // Use C for accVal, but this is ignores so far
      // Note that C might not be used later!
        addr = ((addr-1)<<3);
        addr |=(MsgOut.Data[1] & 0x7);
        aval = (MsgOut.Data[1] & 0x8) >>3;
      }
      else
      {
        Serial.println("Ext: "); // Debug marker    
      // Here for ext decoder - note that we are ignoring bit 3
        addr = ((addr<<3) | ((MsgOut.Data[1] & 0x6)));
        addr = (addr >> 1);
        aval = (MsgOut.Data[2] & 0x1F);    
     }
     // Finally range check address - lets use first 128 for ourself!
     // Ignoring zero
     // or look for broadcast address
     // 128 is temporary - we really want a range here (min to max address)
//     Serial.print(addr,HEX);
//     Serial.write(32); // Debug marker 
//     Serial.println(aval,HEX);
//     Serial.write(32); // Debug marker 
     // Finally if we this the address is for use we set rv true
     // Currently this is if it is from 0 to 128, but not yet if we think it is a broadcast
    // if ((addr>0) && ((addr < 128) || ((MsgOut.Size == 3) && ((addr>>3) == 0x7F)))) rv = true;
       if (addr < 128) rv = true;    
    } 
  } 
  decMsg->decAddress = addr;
  decMsg->decValue = aval;
  return (rv);
}


