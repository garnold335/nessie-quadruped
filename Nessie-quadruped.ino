/*************************************************** 
This code for a bluetooth gamepad-controlled quadruped robot is based on kasinatorthh's code:
https://github.com/kasinatorhh/BTSpidey
Following are the major modifications/enhancements in this code:
1. Runs on Arduino Nano 33 IoT rather than Ardunio Nano. The board includes an integrated bluetooth transceiver, so that a separate board is not needed to use a bluetooth gamepad to control the robot.
2. The Flexitimer2 library had to be replaced with the SAMDTimerInterrupt library.
3. Calibration values stored in flash rather than EEPROM (which is not available on this board)
4. Uses bluepad32 library and firmware for the gamepad controller: https://github.com/ricardoquesada/bluepad32/blob/main/docs/plat_nina.md
5. Servo calibration can be performed using the gamepad rather.
6. LiPo battery voltage monitor using analog pin A6 with a voltage divider: 82Kohm between Vin and A6 and 27kohm between A6 and Gnd (see below).
7. Voltage level is displayed on the gamepad LEDs: 4 LEDs inidcating full charge and 1 LED low voltage threshhold.  The gamepad vibrates continuously when voltage drops below the low threshhold.
8. Added additional moves from John Crombie's code: https://www.youtube.com/watch?v=wmmPD2v2RAA
9. Added modules for polar/cartesian tranformation using cordic library for future port to ESP32, which does not allow floating point code in an interrupt service routine.
*/
/*************************************************** 
  This is the Code for Arduino Spider with  PWM controller.
  3D Design Data and Buildup Guide is found here:
  https://www.thingiverse.com/thing:4070234
  The Spider uses pins see chapter pinmapping below
  
  BSD license, all text above must be included in any redistribution
 ****************************************************/
#define ENABLE_DEBUG_OUTPUT
// Use 0-2. Larger for more flash debugging messages
#define FLASH_DEBUG       2
#define USING_TIMER_TC3         true      // Only TC3 can be used for SAMD51
// These define's must be placed at the beginning before #include "SAMDTimerInterrupt.h"
// _TIMERINTERRUPT_LOGLEVEL_ from 0 to 4
// Don't define _TIMERINTERRUPT_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system.
// Don't define TIMER_INTERRUPT_DEBUG > 2. Only for special ISR debugging only. Can hang the system.
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
//#include "FlashStorage_SAMD.hpp"
//#include <FlashStorage_SAMD.h>
#include <FlashStorage.h>
#include "SAMDTimerInterrupt.h"
#include "Spider_Hardware.h"
#include "ServoHAL.h"
#include "MoveCore.h"
#include <Bluepad32.h>


/**********************************
 * Global Variables / Class inits
 *********************************/
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     1
#define SELECTED_TIMER      TIMER_TC3

#define UARTFAST 115200

#define DEBUG_COMMANDS 1
#define BTCALIB 0x06  //press + and - buttons simultaneously
  #define LF 0x0040
  #define LR 0x0010
  #define RF 0x0080
  #define RR 0x0020
  #define YCOXA 0x0004
  #define AFEMUR 0x0002
  #define BTIBIA 0x0001
  #define RCLICK 0x0200
  #define LCLICK 0x0100
  #define DPADUP 0x01
  #define DPADDN 0x02
  #define DPADL 0x08
  #define DPADR 0x04
  #define X 0x0008
  #define Y 0x0004
  #define A 0x0002
  #define B 0x0001
  #define LFC  LF + YCOXA
  #define LFF  LF + AFEMUR
  #define LFT  LF + BTIBIA
  #define LRC  LR + YCOXA
  #define LRF  LR + AFEMUR
  #define LRT  LR + BTIBIA
  #define RFC  RF + YCOXA
  #define RFF  RF + AFEMUR
  #define RFT  RF + BTIBIA
  #define RRC  RR + YCOXA
  #define RRF  RR + AFEMUR
  #define RRT  RR + BTIBIA
ServoHAL Srv;
SAMDTimer ServoTimer(SELECTED_TIMER);
// gamepad data

GamepadPtr myGamepads[BP32_MAX_GAMEPADS] = {};
int32_t RX_X;
int32_t RX_Y;
int32_t LX_X;
int32_t LX_Y;
uint8_t dpad;
uint16_t Buttons;
int32_t brake;
int32_t throttle;
uint16_t miscButtons;
byte _regGP[6];  //registered bluetooth address, read from flash. 0 if not set
uint32_t vmeasTime = 0;
float voltage;
uint32_t cal_request_time;
volatile bool EEPROMUPDATESENABLED = false;
const int WRITTEN_SIGNATURE = 0xBEEFDEED;

typedef struct {  //structure to hold updatable saved calibration values in flash memory
  boolean valid;
  byte regGP[6] = {0,0,0,0,0,0};
  Servo_Cal_T ServoParams[13];
} StoredCal;
  StoredCal _StoredCal;
FlashStorage(FlashCal, StoredCal); //StoredCal in flash

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  100 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  500 //600 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define INPUT_SIZE 30

/**********************************
 * Global Variables / Class inits
 *********************************/

enum POSITIONS{
  POS_INIT=255,
  POS_REF1=254,
  POS_REF2=253
};

void setup() {  
  // Open serial communications and wait for port to open:
  Serial.begin(UARTFAST);
 // while (!Serial) {
 //   ; // wait for serial port to connect. Needed for Native USB only
//  }
  delay(200);
  Serial.setTimeout(2);
  Serial.println(F("Welcome to Spider Bot!"));
  InitPWM();

  _StoredCal = FlashCal.read();  //read cal values in flash
  if (_StoredCal.valid==false) {
    Serial.println("No calibration values stored, storing default values");
       // Chan, Mode, Mini, Maxi, Ref-, Ref9, Ref+ (Note: Path param is calcualted)
     _StoredCal.ServoParams[0] = {0, 1, 75, 330, 65535, 250, 75};
    _StoredCal.ServoParams[1] = {1, 1, 100, 473, 65535, 260, 455};
    _StoredCal.ServoParams[2] = {2, 1, 180, 447, 65535, 340, 180};
    _StoredCal.ServoParams[3] = {3, 1, 238, 490, 65535, 299, 490};
    _StoredCal.ServoParams[4] = {4, 1, 100, 500, 65535, 340, 140};
    _StoredCal.ServoParams[5] = {5, 1, 177, 460, 65535, 290, 460};
    _StoredCal.ServoParams[6] = {6, 1, 100, 500, 65535, 280, 480};
    _StoredCal.ServoParams[7] = {7, 1, 100, 500, 65535, 320, 120};
    _StoredCal.ServoParams[8] = {8, 1, 150, 470, 65535, 325, 470};
    _StoredCal.ServoParams[9] = {9, 1, 110, 380, 65535, 320, 110};
    _StoredCal.ServoParams[10] = {10, 1, 100, 496,65535, 280, 480};
    _StoredCal.ServoParams[11] = {11, 1, 80, 475, 65535, 220, 80};
    _StoredCal.ServoParams[12] = {12, 1, 110, 360, 65535, 330, 360};
    _StoredCal.valid=true;
    _StoredCal.regGP[0]=0;_StoredCal.regGP[1]=0;_StoredCal.regGP[2]=0;_StoredCal.regGP[3]=0;_StoredCal.regGP[4]=0;_StoredCal.regGP[5]=0;
    FlashCal.write(_StoredCal); //store in flash memory
  }
  /*
      _StoredCal.ServoParams[0] = {0, 1, 75, 330, 65535, 250, 75};
    _StoredCal.ServoParams[1] = {1, 1, 100, 473, 65535, 260, 455};
    _StoredCal.ServoParams[2] = {2, 1, 180, 447, 65535, 340, 180};
    _StoredCal.ServoParams[3] = {3, 1, 238, 490, 65535, 299, 490};
    _StoredCal.ServoParams[4] = {4, 1, 100, 500, 65535, 340, 140};
    _StoredCal.ServoParams[5] = {5, 1, 177, 460, 65535, 290, 460};
    _StoredCal.ServoParams[6] = {6, 1, 100, 360, 65535, 300, 110};
    _StoredCal.ServoParams[7] = {7, 1, 100, 490, 65535, 283, 480};
    _StoredCal.ServoParams[8] = {8, 1, 80, 460, 65535, 220, 80};
    _StoredCal.ServoParams[9] = {9, 1, 125, 490, 65535, 290, 490};
    _StoredCal.ServoParams[10] = {10, 1, 100, 496,65535, 320, 120};
    _StoredCal.ServoParams[11] = {11, 1, 115, 475, 65535, 320, 475};
    _StoredCal.ServoParams[12] = {12, 1, 110, 360, 65535, 330, 360};
    */
  Serial.println("Loading values stored in flash");
  for (int i=0; i<NUMSERVOS; i++) {
    Srv.CalRestore(i,_StoredCal.ServoParams[i]);
  }
    _regGP[0]=_StoredCal.regGP[0];_regGP[1]=_StoredCal.regGP[1];_regGP[2]=_StoredCal.regGP[2];_regGP[3]=_StoredCal.regGP[3];_regGP[4]=_StoredCal.regGP[4];_regGP[5]=_StoredCal.regGP[5];
    Srv.CalPrint();
  


  if (UpdatesEnabled()){
    Serial.print(F("EEPROM Writes Enabled. Send PROTECT to switch off\n"));
  }else{
    PrintUpdateMessage();
  }
  //start servo service
  //FlexiTimer2::set(20, MoveCore::servo_service);
  // ServoTimer.attachInterruptInterval(20*1000, MoveCore::servo_service);
  //test timer
  if(ServoTimer.attachInterruptInterval_MS(20, MoveCore::servo_service)) Serial.println("Timer service started"); else Serial.println("Could not start timer service");
  ServoTimer.stopTimer();
  //initialize servos
  Serial.println(F("Init Leg positions"));
  Mover.Move(MOV_INIT,0);
  if (false){
    Srv.Attach(-1);
    Mover.Move(MOV_STAND,0);
    Mover.Move(MOV_STEPFORWARD,2);
    Mover.Move(MOV_HEADUP,20);
    Mover.Move(MOV_NO,2);
    Mover.Move(MOV_HANDSHAKE,3);
    delay(500);
    Mover.Move(MOV_STEPBACK,2);
    delay(500);
    Mover.Move(MOV_SIT,0);
  }
//  FlexiTimer2::start();
//  ServoTimer.restart();
  PrintHelp();
  Serial.println(F("Ready for Command:"));

  //gamepad setup

  String fv = BP32.firmwareVersion();
  Serial.print("Firmware version installed: ");
  Serial.println(fv);

  // BP32.pinMode(27, OUTPUT);
  // BP32.digitalWrite(27, 0);

  // This call is mandatory. It setups Bluepad32 and creates the callbacks.
  BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But might also fix some connection / re-connection issues.
  BP32.forgetBluetoothKeys();
}



bool UpdatesEnabled(){
  return EEPROMUPDATESENABLED;
}
void EnableUpdates(){
      EEPROMUPDATESENABLED = true;
}
void DisableUpdates(boolean writeFlash){
      EEPROMUPDATESENABLED = false;
      if(writeFlash) FlashCal.write(_StoredCal); //store in flash memory
}
void PrintUpdateMessage(){
    Serial.println(F("Updates are Disabled, send U to activate"));
}


void InitPWM(){
  Serial.println(F("Setup Servo Board"));
  pwm.begin();
  // In theory the internal oscillator is 25MHz but it really isn't
  // that precise. You can 'calibrate' by tweaking this number till
  // you get the frequency you're expecting!
  pwm.setOscillatorFrequency(27000000);  // The int.osc. is closer to 27MHz  
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates  
  for (int S=0;S<13;S++){
    Srv.SetHandler(S,&pwm,S);//All Servos have PWM control in latest wiring
  }
}

// This callback gets called any time a new gamepad is connected.
// Only 1 gamepad can be connected at the same time.
void onConnectedGamepad(GamepadPtr gp) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myGamepads[i] == nullptr) {
      Serial.print("CALLBACK: Gamepad is connected, index=");
      Serial.println(i);
      myGamepads[i] = gp;
      foundEmptySlot = true;
      // Optional, once the gamepad is connected, request further info about the
      // gamepad. 
      GamepadProperties properties = gp->getProperties();
      char buf[80];
      sprintf(buf,
              "BTAddr: %02x:%02x:%02x:%02x:%02x:%02x, VID/PID: %04x:%04x, "
              "flags: 0x%02x",
              properties.btaddr[0], properties.btaddr[1], properties.btaddr[2],
              properties.btaddr[3], properties.btaddr[4], properties.btaddr[5],
              properties.vendor_id, properties.product_id, properties.flags);
      Serial.println(buf);
      if (properties.btaddr[0]==_regGP[0]&&properties.btaddr[1]==_regGP[1]&&properties.btaddr[2]==_regGP[2]&&properties.btaddr[3]==_regGP[3]&&properties.btaddr[4]==_regGP[4]&&properties.btaddr[5]==_regGP[5]) {
        Serial.println("gamepad is registered, allowing connection"); break;//gamepad is registered, allow connection
      } else if (_regGP[0]==0 && _regGP[1]==0 && _regGP[2]==0 && _regGP[3]==0 && _regGP[4]==0 && _regGP[5]==0) {
        //no gamepad is registered, register this one
        _regGP[0]=properties.btaddr[0]; _regGP[1]=properties.btaddr[1]; _regGP[2]=properties.btaddr[2]; _regGP[3]=properties.btaddr[3]; _regGP[4]=properties.btaddr[4]; _regGP[5]=properties.btaddr[5]; 
        EnableUpdates(); 
        _StoredCal.regGP[0]=_regGP[0];_StoredCal.regGP[1]=_regGP[1];_StoredCal.regGP[2]=_regGP[2];_StoredCal.regGP[3]=_regGP[3];_StoredCal.regGP[4]=_regGP[4];_StoredCal.regGP[5]=_regGP[5];
        DisableUpdates(true);  //write bluetooth address to flash
        Serial.println("no gamepad was registered, registering this one");
        break;
      } else Serial.println(gp->isConnected()); Serial.println("gamepad not registered, connection not allowed");//do not allow connection
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println(
        "CALLBACK: Gamepad connected, but could not found empty slot");
  }
}

void onDisconnectedGamepad(GamepadPtr gp) {
  bool foundGamepad = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myGamepads[i] == gp) {
      Serial.print("CALLBACK: Gamepad is disconnected from index=");
      Serial.println(i);
      myGamepads[i] = nullptr;
      foundGamepad = true;
      break;
    }
  }

  if (!foundGamepad) {
    Serial.println(
        "CALLBACK: Gamepad disconnected, but not found in myGamepads");
  }
}

//calibrate from gamepad
boolean calibrate(GamepadPtr Gamepad) {
  delay(3000); //wait 3 sec to see if calibrate buttons are still pressed
  BP32.update();
  if (Gamepad->miscButtons()!=BTCALIB) return(false); //return if calibrate buttons were released
  Gamepad->setRumble(0xc0 /* force */, 0xc0 /* duration */);
  Serial.println("Entering calibration mode");
  // set all servos to 0 position
  for (int i=0; i<NUMSERVOS - 1; i++) {
  ServoTimer.stopTimer();pwm.setPWM(i,0,_StoredCal.ServoParams[i].Ref90);
  delay(20);
  }
  int servo_pos[3];
  int leg = -1;
  int joint = -1;
  while (Gamepad->buttons()!=X) {
    BP32.update();
    //select servo to calibrate
    // use front triggers to select leg
    switch (Gamepad->buttons()) {
    //set the leg or joint number
        case LF: leg = 0; joint = -1;servo_pos[0]=_StoredCal.ServoParams[leg*3].Ref90; servo_pos[1]=_StoredCal.ServoParams[leg*3+1].Ref90; servo_pos[2]=_StoredCal.ServoParams[leg*3+2].Ref90;Serial.println(leg);break;
        case LR: leg = 1; joint = -1;servo_pos[0]=_StoredCal.ServoParams[leg*3].Ref90; servo_pos[1]=_StoredCal.ServoParams[leg*3+1].Ref90; servo_pos[2]=_StoredCal.ServoParams[leg*3+2].Ref90;Serial.println(leg);break;
        case RF: leg = 2; joint = -1;servo_pos[0]=_StoredCal.ServoParams[leg*3].Ref90; servo_pos[1]=_StoredCal.ServoParams[leg*3+1].Ref90; servo_pos[2]=_StoredCal.ServoParams[leg*3+2].Ref90;Serial.println(leg);break;
        case RR: leg = 3; joint = -1;servo_pos[0]=_StoredCal.ServoParams[leg*3].Ref90; servo_pos[1]=_StoredCal.ServoParams[leg*3+1].Ref90; servo_pos[2]=_StoredCal.ServoParams[leg*3+2].Ref90;Serial.println(leg);break;
        case YCOXA: joint = 0; Serial.println(joint);break;
        case AFEMUR: joint = 1; Serial.println(joint);break;
        case BTIBIA: joint = 2; Serial.println(joint);break;
        default: ; 
      }
    if (leg>-1 && joint <0) {
      if ( Gamepad->axisX() > DEADZONE) { 
          servo_pos[0] = servo_pos[0] + 1; Serial.println(servo_pos[0]);
      } else if (Gamepad->axisX() < -1*DEADZONE) {
          servo_pos[0] = servo_pos[0] - 1; Serial.println(servo_pos[0]);
        };
      if (Gamepad->axisY() > DEADZONE) {
        servo_pos[1] = servo_pos[1] - 1; Serial.println(servo_pos[1]);
      } else if (Gamepad->axisY() < -1*DEADZONE) {
          servo_pos[1] = servo_pos[1] + 1; Serial.println(servo_pos[1]);
      };
      if (Gamepad->axisRY() > DEADZONE) {
        servo_pos[2] = servo_pos[2] - 1; Serial.println(servo_pos[2]);
      } else if (Gamepad->axisRY() < -1*DEADZONE) {
          servo_pos[2] = servo_pos[2] + 1; Serial.println(servo_pos[2]);
      };
       pwm.setPWM(leg*3,0,servo_pos[0]);
  //     delay(50);
       pwm.setPWM(leg*3+1,0,servo_pos[1]);
 //      delay(50);
       pwm.setPWM(leg*3+2,0,servo_pos[2]);
       Serial.print("Servo positions: "); Serial.print(leg*3); Serial.print(" ");Serial.print(servo_pos[0]); Serial.print(" "); Serial.print(servo_pos[1]); Serial.print(" "); Serial.println(servo_pos[2]);
       delay(50);
       
    } else if (leg>-1 && joint>-1) { //use + or - buttons to move selected servo joint
      if (Gamepad->miscButtons()==2) servo_pos[joint]=servo_pos[joint]+1; else if (Gamepad->miscButtons()==4) servo_pos[joint]=servo_pos[joint]-1;
      pwm.setPWM(leg*3+joint,0,servo_pos[joint]); delay(50);
    }

 //set 0 or 90 degree positions or min max positions
      switch (Gamepad->dpad()) {
        case DPADDN: if (leg>-1 && joint==-1) {EnableUpdates();SetValue(leg*3, SRV_REF90, F("Center Position"),servo_pos[0]);SetValue(leg*3+1, SRV_REF90, F("Center Position"),servo_pos[1]);SetValue(leg*3+2, SRV_REF90, F("Center Position"),servo_pos[2]);} else {
                       EnableUpdates();SetValue(leg*3+joint, SRV_REF90, F("Center Position"),servo_pos[joint]); 
                      }; break;
        case DPADUP: if (leg>-1 && joint==-1) {EnableUpdates();SetValue(leg*3, SRV_REF180, F("90 Degrees"),servo_pos[0]);SetValue(leg*3+1, SRV_REF180, F("90 Degrees"),servo_pos[1]);SetValue(leg*3+2, SRV_REF180, F("90 Degrees"),servo_pos[2]);
                        } else {
                      EnableUpdates();SetValue(leg*3+joint, SRV_REF180, F("90 Degrees"),servo_pos[joint]); SetValue(leg*3+joint, SRV_MAX, F("Max, 500?"),servo_pos[joint]);
                      }; break;
        case DPADL: if (leg>-1 && joint>-1) {EnableUpdates(); if(servo_pos[joint]<300) SetValue(leg*3+joint, SRV_MIN, F("Min, 300?"),servo_pos[joint]); else SetValue(leg*3+joint, SRV_MAX, F("Max, 500?"),servo_pos[joint]);} break;
        case DPADR: if (leg>-1 && joint>-1) {EnableUpdates(); if(servo_pos[joint]<300) SetValue(leg*3+joint, SRV_MIN, F("Min, 300?"),servo_pos[joint]); else SetValue(leg*3+joint, SRV_MAX, F("Max, 500?"),servo_pos[joint]);} break;
        default: ;
      }
  }
  DisableUpdates(true); //update flash
  Serial.println("Values saved, exiting calibration");
  Srv.CalPrint();
  Gamepad->setRumble(0xc0 /* force */, 0xc0 /* duration */); delay(1000); Gamepad->setRumble(0xc0 /* force */, 0xc0 /* duration */);
  return(true);
}

void PosServo(uint8_t ServoIdx, float Value){
  if (DEBUG_COMMANDS) Serial.println(F("PosServo"));
  Serial.println(Value);
  Srv.write(ServoIdx,Value);
}

void SyncServoMove(uint8_t ServoAxis, uint16_t Delay){
  if (DEBUG_COMMANDS) Serial.println(F("SyncServoMove"));
  for (float percent=0;percent>-90;percent=percent-3){
    for (int arm = 0;arm<4;arm++){
      Srv.write(ServoAxis+arm*3,percent);
    }
    delay(Delay);
  }
  for (float percent=-90;percent<100;percent=percent+1){
    for (int arm = 0;arm<4;arm++){
      Srv.write(ServoAxis+arm*3,percent);
    }
    delay(Delay);
  }
  for (float percent=100;percent>=0;percent=percent-2){
    for (int arm = 0;arm<4;arm++){
      Srv.write(ServoAxis+arm*3,percent);
    }
    delay(Delay);
  }
}

void SetValue(uint8_t ServoID, SrvCal Entry, const __FlashStringHelper * Name, uint16_t Value){
  if (UpdatesEnabled()){
    Serial.print(F("Servo "));
    Serial.print(ServoID);
    Serial.print(F(", Entered Value for "));
    Serial.print(Name);
    Serial.print(F(" was:"));
    Serial.println(Value);
    Srv.CalSet(ServoID,Entry,Value);
    switch (Entry) {  //store saved value in _StoredCql but do not write to flash until disableUpdates() is called
      case SRV_CHANNEL: _StoredCal.ServoParams[ServoID].Channel = Value; break;
      case SRV_MODE: _StoredCal.ServoParams[ServoID].Mode = Value; break;
      case SRV_MIN: _StoredCal.ServoParams[ServoID].Min = Value; break;
      case SRV_MAX: _StoredCal.ServoParams[ServoID].Max = Value; break;
      case SRV_REF0: _StoredCal.ServoParams[ServoID].Ref0 = Value; break;
      case SRV_REF90: _StoredCal.ServoParams[ServoID].Ref90 = Value; break;
      case SRV_REF180: _StoredCal.ServoParams[ServoID].Ref180 = Value; break;
      default: ;
    }
  }else{
    PrintUpdateMessage();
  }
}

void PrintHelp(void){
  Serial.println(F("Supported Commands, CMD ServoID Value:"));
  Serial.println(F("d:SetValue S SRV_MODE,N:SetValue S MIN,M:SetValue S MAX,0:SetValue S Ref0,r:SetValue S REF90,R:SetValue S REF180"));
  Serial.println(F("g:CountServo,G:PosServo,y:MoveIdenticServos,SweepIdenticServos,*:FormatServo"));
  Serial.println(F("Supported Stand alone commands:"));
  Serial.println(F("x:DetatchServos, X:AttachServos,*:FormatServo,^:Reset,?:PrintPos,#:PrintCal"));
  Serial.println(F("q:FormatBluetooth,Q:CheckBlueTooth"));
  Serial.println(F("Supported Mover commands:"));
  Serial.print((char)MOV_STAND);Serial.print(F(":Stand,"));
  Serial.print((char)MOV_SIT);Serial.print(F(":Sit,"));
  Serial.print((char)MOV_STEPFORWARD);Serial.print(F(":Forward,"));
  Serial.print((char)MOV_STEPBACK);Serial.println(F(":Backward"));
  Serial.print((char)MOV_TURNLEFT);Serial.print(F(":TurnLeft,"));
  Serial.print((char)MOV_TURNRIGHT);Serial.print(F(":TurnRight,"));
  Serial.print((char)MOV_HANDWAVE);Serial.print(F(":HandWave,"));
  Serial.print((char)MOV_HANDSHAKE);Serial.println(F(":HandShake"));
  Serial.print((char)MOV_HEADUP);Serial.print(F(":HeadUp,"));
  Serial.print((char)MOV_HEADDOWN);Serial.print(F(":HeadDown,"));
  Serial.print((char)MOV_NO);Serial.println(F(":No"));
}
  static int16_t IdleCounter;
  static bool Idle;

void idle(){
  if (IdleCounter++>65000){
    Serial.println(F("Saving Power"));
    ServoTimer.restartTimer();
    Mover.Move(MOV_SIT,0);
    ServoTimer.stopTimer();
    for (int i=0; i<13;i++){
      Srv.writeCount(i,0);
    }
    Idle=true;
  }
}

void notidle(){
  Mover.SetAllLegMode(0);
  Idle=false;  ServoTimer.restartTimer();
  IdleCounter=0;
}


void loop() {
  char input[INPUT_SIZE+1];
  char cmd;
  uint8_t ServoID; 
  uint16_t Value;
  String inp;
  if (!Idle){
    idle();
  }else{
    IdleCounter=0;
  }
//  Serial.println(IdleCounter);
//12v max, 27Kohm/82Kohm voltage divider, 1023 max resolution on ADC pin, 1023 = 3.06v
//if (millis()-vmeasTime > 10000) {
  voltage = (float)analogRead(VIN_SENSE)*3.3*4.037/1023.0; //(27+82)/27;
//  Serial.print("Voltage: "); Serial.println(voltage);
  if (voltage<LOWV) digitalWrite(LED_BUILTIN, HIGH);
 // vmeasTime = millis();
//}
//Clear Arrays
  for (int i = 0; i < sizeof(input);i++){
    input[i]=0;
  }
  //process hardware serial first
  cmd=0;
  ServoID=15;
  Value=65535;
//read hardware serial first
if (Serial.available()>0) {  //execute serial processing section only if there is serial input
  int ndx = 0;
  while(Serial.available()>0){
    input[ndx++] = Serial.read();
    if (ndx>INPUT_SIZE) break;
  }
  input[ndx] = 0;
    cmd=input[0];
    char *arg;
    arg = strtok(input, " ,");
    cmd = input[0];
    arg = strtok(NULL, " ,");
    if (arg!=NULL) ServoID = (uint8_t) atoi(arg);
    arg = strtok(NULL, " ,");
    if (arg!=NULL) Value=(uint16_t) atoi(arg);
  if (cmd>0) {
      Serial.print(F("Command:"));Serial.print(cmd, DEC);
      Serial.print(F(",ServoID:"));Serial.print(ServoID);
      Serial.print(F(",Value:"));Serial.println(Value);

      switch (cmd){
        case 'g': ServoTimer.stopTimer();pwm.setPWM(ServoID,0,Value);break;
        case 'G': PosServo(ServoID, Value);break;
        case 'x': ServoTimer.stopTimer();InitPWM();Srv.Detatch(-1);break;
        case 'X': InitPWM();Srv.Attach(-1);ServoTimer.restartTimer();break;
        case 'y': Srv.write(Mover.ServoIdx(0,ServoID),(float)(int)Value);
                  Srv.write(Mover.ServoIdx(1,ServoID),(float)(int)Value);
                  Srv.write(Mover.ServoIdx(2,ServoID),(float)(int)Value);
                  Srv.write(Mover.ServoIdx(3,ServoID),(float)(int)Value);
                  break;
        case 'Y': SyncServoMove(Value,20);break;
        case 'c': SetValue(ServoID, SRV_CHANNEL, F("Channel"),Value);break; // command to fix channel # if corrupted
        case 'd': SetValue(ServoID, SRV_MODE,F("Direction? 0=straight 1=invert"),Value);break;
        case 'N': SetValue(ServoID, SRV_MIN, F("Min, 300?"),Value);break;
        case 'M': SetValue(ServoID, SRV_MAX, F("Max, 500?"),Value);break;
        case 'r': SetValue(ServoID, SRV_REF90, F("Center Position"),Value);break;
        case 'L': SetValue(ServoID, SRV_REF0, F("-90 Degrees"),Value);break;
        case 'R': SetValue(ServoID, SRV_REF180, F("90 Degrees"),Value);break;
        case '?': Srv.PrintPos();break;
        case '!': Serial.println("PCA9685:"); Srv.PrintPCA9685();break;
   //     case '<': Srv.CalRestore(); break;
        case '>': Srv.CalBackup(4); break;
        case '#': Srv.CalPrint();break;
        case '*': Srv.InitServoParameters(ServoID,ServoID,SRV_MODE_INVALID,SERVOMIN,SERVOMAX,SERVOMIN,SERVOMAX-SERVOMIN/2,SERVOMAX);
                  if (UpdatesEnabled()){
                    Serial.println("Backing up servo values");
                    Srv.CalBackup(4);
                  }else{ PrintUpdateMessage();};
                  Srv.CalPrint();break;
        case 'U': Serial.println("Enable flash updates"); EnableUpdates(); break;
        case 'P': Serial.println("Update flash then disable flash updates"); DisableUpdates(true); break; // save calibration values to flash
  //      case 'A': SwipeServo(Value,20);break;
  //      case 's': Position(ServoID,300);break;
  //      case 'S': Position(ServoID,100);break;
        case 'V': Serial.print("Voltage ="); Serial.println(float(voltage)); break;
        case 'W': Mover.SetSpeedFactor(Value);break;
        case MOV_STEPFORWARD: 
        case MOV_STEPBACK:
        case MOV_TURNLEFT:
        case MOV_TURNRIGHT:
        case MOV_STAND:
        case MOV_TRANSPORT:
        case MOV_GYRATE:
        case MOV_TWIST:
        case MOV_SPRAWL:
        case MOV_STAGGER:
        case MOV_SIT: notidle(); Mover.Move((Moves) cmd,1);break;
        case MOV_HANDWAVE: Mover.Move(MOV_HANDWAVE,3);break;
        case MOV_HANDSHAKE: notidle(); Mover.Move(MOV_HANDSHAKE,3);break;
        case MOV_HEADUP: notidle(); Mover.Move(MOV_HEADUP,20);break;
        case MOV_HEADDOWN: notidle(); Mover.Move(MOV_HEADUP,-20);break;
        case MOV_NO: notidle(); Mover.Move(MOV_NO,3);break;
        case MOV_TALL: notidle(); Mover.Move(MOV_TALL, 2000); break;
        case MOV_INIT: notidle();Mover.Move(MOV_INIT,1); break;
        //case '^': asm volatile ("  jmp 0");break;
        //case 'q': FormatBlueTooth();break;
        //case 'Q': CheckBlueTooth();break;
 //       case 'D' : ServoDebug=Value;Serial.print(F("ServoDebug:"));
 //                 Serial.println(Value);break;
        default : Serial.println("Unknown command:");
                  PrintHelp();
      } 
  }
}
// Gamepad code
  // This call fetches all the gamepad info from the NINA (ESP32) module.
  // Just call this function in your main loop.
  // The gamepad pointers (the ones received in the callbacks) gets updated
  // automatically.
  BP32.update();

  // It is safe to always do this before using the gamepad API.
  // This guarantees that the gamepad is valid and connected.
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    GamepadPtr myGamepad = myGamepads[i];

    if (myGamepad && myGamepad->isConnected()) {
      //check if gamepad had been registered. If none registered, any connected GP can control, if one is registered only the registered one can control
      GamepadProperties properties = myGamepad->getProperties();
      if (!(properties.btaddr[0]==_regGP[0]&&properties.btaddr[1]==_regGP[1]&&properties.btaddr[2]==_regGP[2]&&properties.btaddr[3]==_regGP[3]&&properties.btaddr[4]==_regGP[4]&&properties.btaddr[5]==_regGP[5])) {myGamepad->disconnect(); break;}
      // There are different ways to query whether a button is pressed.
      // By query each button individually:
      //  a(), b(), x(), y(), l1(), etc...
      
      // Another way to query the buttons, is by calling buttons(), or
      // miscButtons() which return a bitmask.
      // Some gamepads also have DPAD, axis and more.
/*
      char buffer[120];
      snprintf(buffer, sizeof(buffer) - 1,
               "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4li, %4li, axis "
               "R: %4li, %4li, brake: %4ld, throttle: %4li, misc: 0x%02x",
               i,                      // Gamepad Index
               myGamepad->dpad(),      // DPAD
               myGamepad->buttons(),   // bitmask of pressed buttons
               myGamepad->axisX(),     // (-511 - 512) left X Axis
               myGamepad->axisY(),     // (-511 - 512) left Y axis
               myGamepad->axisRX(),    // (-511 - 512) right X axis
               myGamepad->axisRY(),    // (-511 - 512) right Y axis
               myGamepad->brake(),     // (0 - 1023): brake button
               myGamepad->throttle(),  // (0 - 1023): throttle (AKA gas) button
               myGamepad->miscButtons()  // bitmak of pressed "misc" buttons
      );
      Serial.println(buffer);
      */
      // read voltage first
          if (voltage>V4) myGamepad->setPlayerLEDs(LED4); else if (voltage>V3) myGamepad->setPlayerLEDs(LED3); else if (voltage>V2) myGamepad->setPlayerLEDs(LED2); else myGamepad->setPlayerLEDs(LED1);
          if (voltage<V1)  myGamepad->setRumble(0xc0 /* force */, 0xc0 /* duration */); //rumble gamepad if voltage gets low
          if (abs(myGamepad->axisX())>DEADZONE | abs(myGamepad->axisY())>DEADZONE) {   //process joystick first
            Value = pow(1.25,map(sqrt(pow(myGamepad->axisX(),2)+pow(myGamepad->axisY(),2)), 0,512,-4,4))*100;
//            Serial.print("Speed = "); Serial.println(Value);
            Mover.SetSpeedFactor(Value);
          if (abs(myGamepad->axisX()) > abs(myGamepad->axisY())) { //turn L or R
            if (myGamepad->axisX() < 0) { //right
                notidle();Mover.Move(MOV_TURNRIGHT,1);
              } else { //left
                notidle();Mover.Move(MOV_TURNLEFT,1);
              }
          } else {  //forward or backward           
            if (myGamepad->axisY() < 0) { //forward
                notidle();Mover.Move(MOV_STEPFORWARD,1);
            } else { //backward
                notidle();Mover.Move(MOV_STEPBACK,1);
            }
          }
          };
          //check dpad
 //         Serial.print("DPAD=");Serial.print(dpad);
          switch (myGamepad->dpad()) {
                case DPADUP: {Serial.println("Stand"); notidle();Mover.Move(MOV_STAND,1); break;}
                case DPADDN: {Serial.println("Sit"); notidle();Mover.Move(MOV_SIT,1); break;}
                case DPADR:  {Serial.println("Handwave");Mover.Move(MOV_HANDWAVE,3); break;}
                case DPADL:  {Serial.println("Handshake"); notidle(); Mover.Move(MOV_HANDSHAKE,3);break;}
                default: break;              
              };
          //right gamepad for body movements
          if (abs(myGamepad->axisRX())>DEADZONE | abs(myGamepad->axisRY())>DEADZONE) {   //process joystick first
          if (abs(myGamepad->axisRX()) > abs(myGamepad->axisRY())) { //lean L or R
            if (myGamepad->axisRX() < 0) { //right
                Serial.println("Body Right"); notidle();Mover.Move(MOV_BODY_RIGHT,1);
              } else { //left
                Serial.println("Body Left"); notidle();Mover.Move(MOV_BODY_LEFT,1);
              }
          } else {  //forward or backward           
            if (myGamepad->axisRY() < 0) { //lean forward
                notidle();Mover.Move(MOV_HEADUP,20);
            } else { //lean backward
                notidle();Mover.Move(MOV_HEADUP,-20);
            }
          }
          }; 
          switch (myGamepad->miscButtons()) {
            case BTCALIB: calibrate(myGamepad); break;
            default: ;
          }
          switch (myGamepad->buttons()) {
            case A: notidle(); Mover.Move(MOV_STAGGER,1); break;
            case B: notidle(); Mover.Move(MOV_GYRATE,1); break;
            case Y: notidle(); Mover.Move(MOV_TWIST,1); break;
            case X: notidle(); Mover.Move(MOV_SPRAWL, 1); break;
            case RR: notidle(); Mover.Move(MOV_TALL, 2000); break;
            case LF: notidle(); Mover.Move(MOV_BODY_SIDE_UPDOWN, 1); break;
            case RF: notidle(); Mover.Move(MOV_BODY_SIDE2SIDE, 1); break;
            case LR: notidle(); Mover.Move(MOV_BODY_FWD_BKWD, 1); break;
            /*
            case LFC: {Srv.write(Mover.ServoIdx(0,0), (float)(int) 0);
                  Srv.write(Mover.ServoIdx(1,0), (float)(int)0);
                  Srv.write(Mover.ServoIdx(2,0), (float)(int)0);
                  Srv.write(Mover.ServoIdx(3,0), (float)(int)0);}; break;
            case LFF: {Srv.write(Mover.ServoIdx(0,1),(float)(int)0);
                  Srv.write(Mover.ServoIdx(1,1),(float)(int)0);
                  Srv.write(Mover.ServoIdx(2,1),(float)(int)0);
                  Srv.write(Mover.ServoIdx(3,1),(float)(int)0);}; break;
            case LFT: {Srv.write(Mover.ServoIdx(0,2),(float)(int)0);
                  Srv.write(Mover.ServoIdx(1,2),(float)(int)0);
                  Srv.write(Mover.ServoIdx(2,2),(float)(int)0);
                  Srv.write(Mover.ServoIdx(3,2),(float)(int)0);}; break;
            case LRC: {Srv.write(Mover.ServoIdx(0,0),(float)(int)50);
                  Srv.write(Mover.ServoIdx(1,0),(float)(int)50);
                  Srv.write(Mover.ServoIdx(2,0),50);
                  Srv.write(Mover.ServoIdx(3,0),(float)(int)50);}; break;     
            case LRF: {Srv.write(Mover.ServoIdx(0,1),(float)(int)50);
                  Srv.write(Mover.ServoIdx(1,1),(float)(int)50);
                  Srv.write(Mover.ServoIdx(2,1),(float)(int)50);
                  Srv.write(Mover.ServoIdx(3,1),(float)(int)50);}; break;   
            case LRT: {Srv.write(Mover.ServoIdx(0,2),(float)(int)50);
                  Srv.write(Mover.ServoIdx(1,2),(float)(int)50);
                  Srv.write(Mover.ServoIdx(2,2),(float)(int)50);
                  Srv.write(Mover.ServoIdx(3,2),(float)(int)50);}; break;   
            case RFC: {Srv.write(Mover.ServoIdx(0,0),(float)(int)100);
                  Srv.write(Mover.ServoIdx(1,0),(float)(int)100);
                  Srv.write(Mover.ServoIdx(2,0),(float)(int)100);
                  Srv.write(Mover.ServoIdx(3,0),(float)(int)100);}; break;  
            case RFF: {Srv.write(Mover.ServoIdx(0,1),(float)(int)100);
                  Srv.write(Mover.ServoIdx(1,1),(float)(int)100);
                  Srv.write(Mover.ServoIdx(2,1),(float)(int)100);
                  Srv.write(Mover.ServoIdx(3,1),(float)(int)100);}; break;           
            case RFT: {Srv.write(Mover.ServoIdx(0,2),(float)(int)100);
                  Srv.write(Mover.ServoIdx(1,2),(float)(int)100);
                  Srv.write(Mover.ServoIdx(2,2),(float)(int)100);
                  Srv.write(Mover.ServoIdx(3,2),(float)(int)100);}; break;  
                  */                                                                                                 
            default: ;
          }
      // You can query the axis and other properties as well. See Gamepad.h
      // For all the available functions.
    } // end gamepad loop
  }
  delay(10);
}


/*****************************
 * Section deals with Servos
 */
 //void Position(uint8_t Value, uint16_t Speed){
//  if (DEBUG_COMMANDS) Serial.println(F("Position"));
//  switch (Value){
//    case 0:for (int arm=0;arm<4;arm++){
//               ServoNext[0+arm]=GetServoNext(0+arm,100);
//               ServoNext[4+arm]=GetServoNext(4+arm,250);
//               ServoNext[8+arm]=GetServoNext(8+arm,20);
//           }
//           break;
//    case 1:for (int arm=0;arm<4;arm++){
//           ServoNext[4+arm]=GetServoNext(4+arm,128);
//           ServoNext[8+arm]=GetServoNext(8+arm,128);
//           }
//           break;
//    case POS_INIT:
//           for (int servo=0;servo<8;servo++){
//               setPWM(servo,ServoSpec[servo].Reflen1);//Home Coxa+Thibia
//           }
//           for (int servo=8;servo<12;servo++){
//               setPWM(servo,ServoSpec[servo].Reflen2);//Home Femur
//           }
//           break;
//    case POS_REF1:
//           for (int servo=0;servo<8;servo++){
//               setPWM(servo,ServoSpec[servo].Reflen1);//Home Coxa+Thibia
//           }
//           for (int servo=8;servo<12;servo++){
//               setPWM(servo,ServoSpec[servo].Reflen1);//Home Femur
//           }
//           break;
//    case POS_REF2:
//           for (int servo=0;servo<8;servo++){
//               setPWM(servo,ServoSpec[servo].Reflen2);//Home Coxa+Thibia
//           }
//           for (int servo=8;servo<12;servo++){
//               setPWM(servo,ServoSpec[servo].Reflen2);//Home Femur
//           }
//           break;
//          
//  }
//}
//void SwipeServo(uint8_t ServoIdx, uint16_t Delay){
//  if (DEBUG_COMMANDS) Serial.println(F("SwipeServo"));
//  for (float percent=0;percent>-90;percent=percent-3){
//    ServoWrite(ServoIdx,percent);
//    delay(Delay);
//  }
//  for (float percent=-90;percent<100;percent=percent+1){
//    ServoWrite(ServoIdx,percent);
//    delay(Delay);
//  }
//  for (float percent=100;percent>=0;percent=percent-2){
//    Srv.ServoWrite(ServoIdx,percent);
//    delay(Delay);
//  }
//}
