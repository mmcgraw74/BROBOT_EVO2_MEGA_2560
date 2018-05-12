// Changes to allow xy1 to control the robot and to have TOUCH send bach centering (0.5) to xy1
// 9/27/2017  Added logic to detect a /pimg message from the device
//            After receiving the ping send the battery voltage to the fader3 and label4

// BROBOT EVO 2 by JJROBOTS
// SELF BALANCE ARDUINO ROBOT WITH STEPPER MOTORS
// License: GPL v2
// OSC functions  (OSC = Open Sound Control protocol)

// OSC Messages read:  OSC: /page/command parameters
//             FADER (1,2,3,4)  Ex: /1/fader1   f,  XXXX  => lenght:20, Param:  float (0.0-1.0)
//             XY (1,2)          Ex: /1/xy1  f,f,    XXXXXXXX => length: 24 Params: float,float (0.0-1.0)
//             PUSH (1,2,3,4)    Ex: /1/push1    f,  XXXX => length:20 Param: float
//             TOGGLE (1,2,3,4)  Ex: /1/toggle1  f,  XXXX => length:20 Param: float
//             MOVE              Ex: /1/m XXXX XXXX XXXX => length:16 Params: speed, steps1, steps2 (all float)
//
// OSC Message send:
//            string to send + param (float)[last 4 bytes]

// for DEBUG uncomment this lines...
//#define OSCDEBUG 0

//   Battery voltage per battery Alkaline = 1.5, NiCad/NiMH = 1.25, Lipo = 4.2
// If using other then the 6 alkaline batterys in the battery holder or a 9 volt battery
// comment out the next line and uncomment the line for the battery you are using.
float Battery_Normal_Voltage = 9.0;    // Value of 6 alkaline batteries or 9 volt battery
float Battery_Low_Voltage    = 6.7;    // Using 1.1 volts per AA alkaline battery for low voltage
//float Battery_Normal_Voltage = 7.5;     // Value of 6 NiCad/NiMh batteries
//float Battery_Normal_Voltage = 12.6;    // Value of a 3 call lipo battery
//float Battery_Normal_Voltage = 8.4;     // Value of a 2 call lipo battery

boolean Time_to_send_percent = true;   // Used to alternate sending battery messages. 
                                       // It get's a "twitch" if both are sent at the same time
boolean Low_Battery = false;           // Set this flag when battery falls to low voltage or below - and turn on OSC battery low LED 1/Led/1

char UDPBuffer[8]; // input message buffer

// OSC message internal variables
unsigned char OSCreadStatus;
unsigned char OSCreadCounter;
unsigned char OSCreadNumParams;
unsigned char OSCcommandType;
unsigned char OSCtouchMessage;


// ------- OSC functions -----------------------------------------

// Aux functions
float OSC_extractParamFloat(uint8_t pos) {
  union {
    unsigned char Buff[4];
    float d;
  } u;

  u.Buff[0] = (unsigned char)UDPBuffer[pos];
  u.Buff[1] = (unsigned char)UDPBuffer[pos + 1];
  u.Buff[2] = (unsigned char)UDPBuffer[pos + 2];
  u.Buff[3] = (unsigned char)UDPBuffer[pos + 3];
  return (u.d);
}

int16_t OSC_extractParamInt(uint8_t pos) {
  union {
    unsigned char Buff[2];
    int16_t d;
  } u;

  u.Buff[1] = (unsigned char)UDPBuffer[pos];
  u.Buff[0] = (unsigned char)UDPBuffer[pos + 1];
  return (u.d);
}


void OSC_init()
{
  OSCreadStatus = 0;
  OSCreadCounter = 0;
  OSCreadNumParams = 0;
  OSCcommandType = 0;
  OSCfader[0] = 0.5;
  OSCfader[1] = 0.5;
  OSCfader[2] = 0.5;
  OSCfader[3] = 0.5;
}

void OSC_MsgSend(char *c, unsigned char msgSize, float p)
{
  uint8_t i;
  union {
    unsigned char Buff[4];
    float d;
  } u;

  // We copy the param in the last 4 bytes
  u.d = p;
  c[msgSize - 4] = u.Buff[3];
  c[msgSize - 3] = u.Buff[2];
  c[msgSize - 2] = u.Buff[1];
  c[msgSize - 1] = u.Buff[0];
  for (i = 0; i < msgSize; i++)
  {
    Serial1.write((uint8_t)c[i]);
  }
}

void OSC_MsgSend_ff(char *c, unsigned char msgSize, float p1, float p2)
{
  uint8_t i;
  union {
    unsigned char Buff[4];
    float d;
  } u;
  u.d = p1;
  union {
    unsigned char Buff[4];
    float d;
  } v;
  v.d = p2;
  // We copy the param in the last 4 bytes
  c[msgSize - 8] = v.Buff[3];
  c[msgSize - 7] = v.Buff[2];
  c[msgSize - 6] = v.Buff[1];
  c[msgSize - 5] = v.Buff[0];
  
  c[msgSize - 12] = u.Buff[3];
  c[msgSize - 11] = u.Buff[2];
  c[msgSize - 10] = u.Buff[1];
  c[msgSize - 9] = u.Buff[0];
  for (i = 0; i < msgSize; i++)
  {
    Serial1.write((uint8_t)c[i]);
  }
}

void OSC_MsgRead()
{
  uint8_t i;
  uint8_t tmp;
  float value;
  float value2;

  // New bytes available to process?
  if (Serial1.available() > 0) {
    // We rotate the Buffer (we could implement a ring buffer in future)
    for (i = 7; i > 0; i--) {
      UDPBuffer[i] = UDPBuffer[i - 1];
    }
    UDPBuffer[0] = Serial1.read();
#ifdef OSCDEBUG3
    Serial.print(UDPBuffer[0]);
#endif
    // We look for an OSC message start like /x/    Find Page
    if ((UDPBuffer[0] == '/') && (UDPBuffer[2] == '/') && ((UDPBuffer[1] == '1') || (UDPBuffer[1] == '2'))) {
      if (OSCreadStatus == 0) {
        OSCpage = UDPBuffer[1] - '0';  // Convert page to int
        OSCreadStatus = 1;
        OSCtouchMessage = 0;
        //Serial.print("$");
#ifdef OSCDEBUG3
        Serial.println();
#endif
      }
      else {
        Serial.println("!ERR:osc");
        OSCreadStatus = 1;
      }
      return;
    } else if ((UDPBuffer[3] == 'i') && (UDPBuffer[2] == 'n') && (UDPBuffer[1] == 'g')) {
      // Ping sent from OSC device Send battery value 
      int battery = analogRead(5);
      float temp = (battery / 1024.0) * 5.0;
      float BatteryVoltage = (temp * 1.5)+ temp; 
      
      if (Time_to_send_percent) {
        Time_to_send_percent = false;
        if (BatteryVoltage < Battery_Low_Voltage) {
        Low_Battery = true;  
        OSC_MsgSend("/1/led1\0,f\0\0\0\0\0\0", 16, 1.0);   
      } else {
        Low_Battery = false;
      }
        float Percent_value = BatteryVoltage / Battery_Normal_Voltage;  // Gets the % of full charge
        OSC_MsgSend("/1/fader3\0\0\0,f\0\0\0\0\0\0", 20, Percent_value);
      } else {
        Time_to_send_percent = true;
      
        OSC_MsgSend("/1/label4\0\0\0,f\0\0\0\0\0\0", 20, BatteryVoltage);
      }
     return;
    } else if (OSCreadStatus == 1) { // looking for the message type
      // Fadder    /1/fader1 ,f  xxxx
      if ((UDPBuffer[3] == 'd') && (UDPBuffer[2] == 'e') && (UDPBuffer[1] == 'r')) {
        OSCreadStatus = 2;  // Message type detected
        OSCreadCounter = 11; // Bytes to read the parameter
        OSCreadNumParams = 1; // 1 parameters
        OSCcommandType = UDPBuffer[0] - '0';
#ifdef OSCDEBUG2
        Serial.print("$FAD1");
        Serial.print(OSCcommandType);
        Serial.print("$");
#endif
        return;
      } // end fadder
      // MOVE message
      if ((UDPBuffer[3] == 'o') && (UDPBuffer[2] == 'v') && (UDPBuffer[1] == 'e')) {
        OSCreadStatus = 2;  // Message type detected
        OSCreadCounter = 8; // Bytes to read the parameters
        OSCreadNumParams = 3; // 3 parameters
        OSCcommandType = 40;
#ifdef OSCDEBUG2
        Serial.print("$MOVE:");
#endif
        return;
      }  // End MOVE message
      // XY message
      if ((UDPBuffer[2] == 'x') && (UDPBuffer[1] == 'y')) {
        OSCreadStatus = 2;  // Message type detected
        OSCreadCounter = 14; // Bytes to read the parameters
        OSCreadNumParams = 2; // 2 parameters
        OSCcommandType = 10 + (UDPBuffer[0] - '0');
#ifdef OSCDEBUG2
        Serial.print("$xy");
        Serial.print(OSCcommandType);
        Serial.print("$");
#endif
        return;
      }  // End XY message
      // Push message
      if ((UDPBuffer[3] == 'u') && (UDPBuffer[2] == 's') && (UDPBuffer[1] == 'h')) {
        OSCreadStatus = 2;  // Message type detected
        OSCreadCounter = 10; // Bytes to read the parameter
        OSCreadNumParams = 1; // 1 parameters
        OSCcommandType = 20 + (UDPBuffer[0] - '1');
        //Serial.println(commandType);
#ifdef OSCDEBUG2
        Serial.print("$P");
        Serial.print(UDPBuffer[0] - '1');
        Serial.print(":");
#endif
          return;
        } // end push
        // Toggle message
        if ((UDPBuffer[3] == 'g') && (UDPBuffer[2] == 'l') && (UDPBuffer[1] == 'e')) {
        OSCreadStatus = 2;  // Message type detected
        OSCreadCounter = 10; // Bytes to read the parameter
        OSCreadNumParams = 1; // 1 parameters
        OSCcommandType = 30 + (UDPBuffer[0] - '1');
        //Serial.println(commandType);
#ifdef OSCDEBUG2
        Serial.print("$T");
        Serial.print(UDPBuffer[0] - '1');
        Serial.print(":");
#endif
          return;
        } // end toggle
      } else if (OSCreadStatus == 2) {
      if ((UDPBuffer[1] == '/') && (UDPBuffer[0] == 'z')) { // Touch up message? (/z) [only on page1]
        if ((OSCpage == 1) && ((OSCcommandType <= 2) || (OSCcommandType == 11))) { // Touchup message only on Fadder1 and Fadder2 and xy1
          OSCtouchMessage = 1;
        }
        else {
          OSCtouchMessage = 0;
          OSCreadStatus = 0; //Finish
        }
      } // Touch message(/z)
      OSCreadCounter--;   // Reading counter until we reach the Parameter position
      if (OSCreadCounter <= 0) {
        OSCreadStatus = 0;
        OSCnewMessage = 1;
        //Serial.println(value);
        switch (OSCcommandType) {
          case 1:
            value = OSC_extractParamFloat(0);
            OSCfader[0] = value;
            if ((OSCtouchMessage) && (value == 0)) {
              OSCfader[0] = 0.5;
              //Serial.println("TOUCH_X");
              OSC_MsgSend("/1/fader1\0\0\0,f\0\0\0\0\0\0", 20, 0.5);
            }
#ifdef OSCDEBUG
            Serial.print("$F1:");
            Serial.println(OSCfader[0]);
#endif
            break;
          case 2:
            value = OSC_extractParamFloat(0);        
            OSCfader[1] = value;
            if ((OSCtouchMessage) && (value == 0)) {
              OSCfader[1] = 0.5;
              //Serial.println("TOUCH_Y");
              OSC_MsgSend("/1/fader2\0\0\0,f\0\0\0\0\0\0", 20, 0.5);
            }
#ifdef OSCDEBUG
            Serial.print("$F2:");
            Serial.println(OSCfader[1]);
#endif
            break;
          case 3:
            OSCfader[2] = OSC_extractParamFloat(0);
#ifdef OSCDEBUG
            Serial.print("$F3:");
            Serial.println(OSCfader[2]);
#endif
            break;
          case 4:
            OSCfader[3] = OSC_extractParamFloat(0);
#ifdef OSCDEBUG
            Serial.print("$F4:");
            Serial.println(OSCfader[3]);
#endif
            break;
          case 11:
            OSCxy1_x = OSC_extractParamFloat(0);
            OSCxy1_y = OSC_extractParamFloat(4);
            //  Make Change to speed and direction.
            OSCfader[0] = OSCxy1_y;
            OSCfader[1] = OSCxy1_x;
#ifdef OSCDEBUG
            Serial.print("$XY1:");
            Serial.print(OSCxy1_x);
            Serial.print(",");
            Serial.println(OSCxy1_y);
#endif
            if ((OSCtouchMessage) && (value == 0)) {
              OSCxy1_x = 0.5;
              OSCxy1_y = 0.5;
              OSCfader[0] = 0.5;
              OSCfader[1] = 0.5;
//            Serial.println("TOUCH_xy");
              OSC_MsgSend_ff("/1/xy1\0\0,ff\0\0\0\0\0\0\0\0\0\0\0\0\0", 24, 0.5, 0.5);  // yx parameters
            }
            break;
          case 12:
            OSCxy2_x = OSC_extractParamFloat(0);
            OSCxy2_y = OSC_extractParamFloat(4);
#ifdef OSCDEBUG
            Serial.print("$XY2:");
            Serial.print(OSCxy2_x);
            Serial.print(",");
            Serial.println(OSCxy2_y);
#endif
            break;
          case 40:
            // MOVE
            OSCmove_mode = 1;
            OSCmove_speed = OSC_extractParamInt(4);
            OSCmove_steps1 = OSC_extractParamInt(2);
            OSCmove_steps2 = OSC_extractParamInt(0);
#ifdef OSCDEBUG
            Serial.print("$MOVE:");
            Serial.print(OSCmove_speed);
            Serial.print(",");
            Serial.print(OSCmove_steps1);
            Serial.print(",");
            Serial.println(OSCmove_steps2);
#endif
            break;

          default:
            // Push y toggle
            value = OSC_extractParamFloat(0);
            if ((OSCcommandType >= 20) && (OSCcommandType < 25))
            {
              if (value == 0)
                OSCpush[OSCcommandType - 20] = 0;
              else
                OSCpush[OSCcommandType - 20] = 1;
            }
            if ((OSCcommandType >= 30) && (OSCcommandType < 35))
            {
              if (value == 0)
                OSCtoggle[OSCcommandType - 30] = 0;
              else
                OSCtoggle[OSCcommandType - 30] = 1;
            }
            break;
        } // switch
      }  // if (OSCRead_counter<=0)
    }  // if (OSCread_status==2)
  }  // end Serial.available()
}

