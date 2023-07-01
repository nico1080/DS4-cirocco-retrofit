/*
DS4-cirocco-retrofit

New updated sketch see Cirocco_retrofit\Cirocco_retrofit_completev1.ino:



Sketch by Nico1080
This sketch was tested on a DS4, and should also work on any C4 gen 2 (hatchback, sedan, etc)

Look at my profile to see all modification I brought to the car:
https://www.drive2.ru/r/citroen/ds4/609447751477897832/

Many thanks to:
- Keryan (https://www.forum-peugeot.com/Forum/members/keryan.126947/) for code insipration
- Cesenate (https://www.drive2.ru/users/cesenate) for the HBA cluster CAN ID from DS5  (ID 0x217 Byte 3, value 0x80)
- Infizer for many items  (https://github.com/infizer91/can_extender  , https://www.drive2.ru/users/infizer/ , https://telecoder.ru/index.php )
And all I have forgotten

Feel free to use this code and improve it. In return I only ask that you share your improvement (and obviously that you don't sell it)

The function of this sketch:
- Make SAM (Surveillance Angles Mort, blind spots monitor) NAC toogle switch work
- AAS (Aide Au Stationement, parking sensor) need BSI to be coded to listen to NAC and not cluster anymore (some code could be wrote to have both working)
- Speed limit display from CVM and NAV data (see comment)
- Fix SAM light on cirocco (frame 0x321)  (Thanks to Infizer)
- Fix animation on cirocco (frame 0x236)  (Thanks to Infizer)
- Display green/orange line on cirrocco (AFIL and LKA need to be activated in cirocco) (Thanks to Infizer)
- Make AFIL indicator flash when lines are being crossed without turn indicator (see comment)(Thanks to Infizer)
- Deactivate Stop & Start 5s after engine start
- Fix MEM returning to default value for all setting (40km/h) (Thanks to Infizer)
- Double pressing MEM will set and activate cruise control/ speed limiter to a corrected speed value (offset+ rain, see comment)
- Physical connection with AAS/ECO/SAM button/led to make them work again and add other function (see comment)
- Display front/rear camera (VisioPark2) by pressing AAS/ECO button (see comment)
- Display front camera(VisioPark2) when front obstacle is detected by AAS (see comment)
- Change Cirocco ambiance/theme by pressing the ESC button on the wheel (see comment)

For speed limit display it follow this logic:
- show end of speed-limit sign for 3s when it is read by CVM
- show CVM speed when it is reliable (red sign)
- keep showing CVM speed in red sign after CVM loose reliability. Only if NAV speed have the same value and has not changed since it lost CVM reliability (CVM reliability is easily lost so I "extend" it)
- show NAV speed (grey sign)
- if no NAV speed is available it display last read sign (grey sign and remove 1km/h. Example last sign=70km/h display 69 in grey sign)
- If no previous sign (and no nav speed) it display nothing

For AFIL  (Alerte Franchissement Involontaire de Ligne, line detection):
- Display green orange line as soon as they are detected at any time (require LKA activated in cirocco)
- Flash AFIL light when one line become orange and if one(and not 2) turning indicator is ON  and when speed is over 50km/h (require AFIL activated in Cirocco)
- When left/right lines are both detected: margin calculation for AFIL threshold (large road= big margin small road= small margin) default value is 1300.

For Stop and start deactivation:
- It require NAC toggle key working for stop and start.  BSI need to be coded to telematic in engine menu (Type d'acquisition du contacteur stop and start) , other choices are BSI (original value on my DS4) and cluster.
- If stop & start is not already deactivated, it will send request (Id 1A9) to do it 5s after engine start (rpm>500) and check if it worked (if it didn't, it will try again)

For MEM FIX:
- sketch  will send 19b(limit) & 1DB (cruise) on CAN-DIV after ignition with speed value (BSI will send the same frame several times (6 for limit, 3 for cruise) sorted in ascending order
- Some code could be written to check if driver changed setting and store new values inside EEPROM

For setting cruise control/ speed limiter:
- the sketch will program the speed value into the MEM setting and emulate the needed button press (pause, MEM etc) to activate it.
- The wheels button need only to be set the cruise or limiter mode.
- The set speed is offseted by 2 or 3km/h (between 70/79 and above 80)
- If wiper are active (auto mode) the speed will also be decrease by 20 for 130 and 10 for 90/110 (French speed limit are lowered when raining)

For AAS/ECO/SAM button connection: 
- Button pin need to connected to arduino input pin (no need for resistor as INPUT_PULLUP is activated
- When button are pushed sketch will send request on ID 1A9 (ECO and AAS, NAC emulation, BSI need to be coded to listen to NAC) or 217 (SAM, Cluster emulation as BSI can not be coded for other source)
- For AAS/ECO/SAM led connection: a level shifter is required (led use 12v logic and arduino is only 5v) I used a UDN2981.
- Sketch listen to ID 227 (ECO and AAS) and 2D1 (SAM). It will turn on the led only if ignition is on.
- For button backlight I kept the original wire from car(265 generated by BSI, connected on button pin 4).  Some code could be written to integrate it on arduino and avoid extra wiring
- See https://www.drive2.ru/l/633915458608714452/

For VisioPark2 the sketch will:
- Show front video when front obstacle is detected by AAS
- Show/hide front video when AAS button is short pressed (<800ms)
- Show/hide rear video when ECO button is short pressed (<800ms)
- Long press (>800ms) on AAS/ECO button will make normal operation (ID 1A9 ECO and AAS NAC emulation)
- In any case video will automatically:
   - 	Disappear when going over 25km/h (setting built inside NAC, no way around it)
   - 	Rear video will show when rear gear is engaged 	


For changing Cirocco ambiance/theme:
- A short press on ESC button will toggle ambiance between: No ambiance/relax/boost ambiance,  This setting will disappear after car is shut off.  (It is a shortcut for Icockpit amplify in NAC menu)
- A long press (>1sec) on ESC button will change theme on Cirocco (blue or bronze) without changing NAC theme. However when restarting the car, NAC will change his theme to match the cirocco theme.

After many try on my car I figured the following logic:
- Cirocco have 2 themes activated: 1=Blue and 2=Bronze
- NAC have 3 themes: 1=purple (AMETHYST), 2=red (RUBY), 3=yellow(GOLD)  (+unlisted NACblue theme)

When changing theme on NAC it also change Cirocco theme: 1purple-->1blue, 2red-->2bronze and 3Yellow-->2bronze.

When changing theme on Cirocco with ESC button, it will change NAC theme after restart: 1blue--> 1purple and 2bronze-->2red  (3yellow is not accessible)


If I try activating more themes in cirocco/NAC I have weird behaviour: NAC reboot to unlisted NACblue theme, and NAC theme selection menu disappear.cluster CAN ID from DS5  (ID 0x217 Byte 3, value 0x80)


Feel free to use this code and improve it. In return I only ask that you share your improvement (and obviously that you don't sell it or try to make money from it)
*/

/////////////////////
//    Libraries    //
/////////////////////

#include <SPI.h>
#include <mcp2515.h>  // https://github.com/autowp/arduino-mcp2515 + https://github.com/watterott/Arduino-Libs/tree/master/digitalWriteFast

/////////////////////
//  Configuration  //
/////////////////////

#define CS_PIN_CAN0 10  //pin connected to NAC/cirocco CAN bus module
#define CS_PIN_CAN1 9   //pin connected to CVM CAN bus module
#define SERIAL_SPEED 115200
#define CAN_SPEED CAN_125KBPS  // Diagnostic CAN bus - High Speed
#define CAN_FREQ MCP_8MHZ      // Switch to 16MHZ if you have a 16Mhz module


MCP2515 CAN0(CS_PIN_CAN0);  // NAC/cirocco CAN bus
MCP2515 CAN1(CS_PIN_CAN1);  // CVM CAN bus


////////////////////
//   Variables    //
////////////////////

// My variables
bool debugCAN0 = false;  //Debug for NAC/cirocco CAN bus
bool debugCAN1 = false;  //Debug CVM CAN bus
bool SerialEnabled = true;

// CAN-BUS Messages
struct can_frame canMsgSnd;
struct can_frame canMsgRcv;
bool IDChanged = false;

//Bool for avoiding duplicate push
bool SAMsend = false;
bool SAM_NAC = false;
bool lastSAM_NAC = false;

//Speed display
int progspeed = 0xff;
int Lastprogspeed = 0xff;
int NAVspeed = 0xff;
int CVMspeed = 0xff;
bool CVMreliabity = false;
bool LastCVMreliabity = false;
bool CustomCVMreliabity = false;
int RefNAVspeed = 0xff;
bool CVMendspeed = false;
bool boolendspeed = false;
unsigned long lastendspeed;  //timer for stop display of speed limit
int EndspeedDelay = 5000;

// AFIL
////////////////////////
// left line
bool fix_left_line = false;
int type_left_line = 0;
int distance_to_left_line = 0;
// right line
bool fix_right_line = false;
int type_right_line = 0;
int distance_to_right_line = 0;
int AFIL_distance_threshold;  //AFIL threshold (include car half width  (1300mm = 300m form line if car width  is 2m)

bool left = false;
bool right = false;
int vehicle_speed = 0;    // current vehicle speed
int speedthreshold = 53;  //thresold to activate afil light (include 3km/h correction)

bool Animation_done = false;  //set to true to not do animation
bool DriverDoor = false;


//Stop Start deletion
int rpm = 0;
bool EngineBeenStarted = false;
unsigned long EngineBeenStartedTimer = 0;
bool SSstatus = false;
bool SSdesactivationDone = false;
bool SSrequest = false;
unsigned long SSrequestTimer = 0;
bool ignition = false;

//MEM fix
bool MEMfixDoneLimiter = false;
bool MEMfixDoneCruise = false;
bool Lastingnition = false;
unsigned long ignitiontimer = 0;
struct can_frame LimitMsg;
struct can_frame CruiseMsg;

// Button for arduino
const int SAMBUTTON_PIN = A0;  // Arduino pin connected to button's pin
const int SAMLED_PIN = A3;     // Arduino pin connected to LED's pin
const int ECOBUTTON_PIN = A1;  // Arduino pin connected to button's pin
const int ECOLED_PIN = A4;     // Arduino pin connected to LED's pin
const int AASBUTTON_PIN = A2;  // Arduino pin connected to button's pin
const int AASLED_PIN = A5;     // Arduino pin connected to LED's pin

int ECOledState = LOW;      // the current state of LED
int ECOlastButtonState;     // the previous state of button
int ECOcurrentButtonState;  // the current state of button
int SAMledState = LOW;      // the current state of LED
int SAMlastButtonState;     // the previous state of button
int SAMcurrentButtonState;  // the current state of button
int AASledState = LOW;      // the current state of LED
int AASlastButtonState;     // the previous state of button
int AAScurrentButtonState;  // the current state of button
bool SAMstatus = false;
bool AASstatus = false;
bool AFILstatus = false;
unsigned long ButtonTimer = 0;
bool ECOsend = false;
bool AASsend = false;

//setting cruise/limit control
bool MemState = false;
bool LastMemState = false;
bool SetLimiter = false;
int Limiterspeed = 0xff;
bool CheckLimiter = false;
unsigned long CheckLimiterTimer = 0;
bool LimiterCkecked = false;
unsigned long MemStateTimer = 0;
bool MemSend = false;
unsigned long MemSendTimer = 0;      //timer between virtual mem press send and 1A9 sending
unsigned long MemSendWaitTimer = 0;  //timer between sending 19b/1db and sending mem press

bool FixMemLimit = false;
unsigned long FixMemLimitTimmer = 0;

bool SetCruise = false;
int Cruisespeed = 0xff;
bool CheckCruise = false;
unsigned long CheckCruiseTimer = 0;
bool CruiseCkecked = false;
bool FixMemCruise = false;
unsigned long FixMemCruiseTimmer = 0;
int RXXpaused = false;

bool LimitMode = false;
bool CruiseMode = false;
bool PauseSend = false;
//unsigned long PauseSendWaitTimer = 0; //timer between sending 1a9 (setting rxx value) and sending pause press
int DisplayMemLogo = 0;
bool Speedvalid = false;
bool PlusSend = false;
//unsigned long PlusSendWaitTimer = 0; //timer between double MEM press and sending + press to set any speed in cruise mode
bool PauseSendDisable = false;
bool WiperActive = false;

//ambiance
bool EscState = false;
bool LastEscState = false;
int ambiance = 0x0E;  //default value at startup, 0E= off, 8E=relax ambiance,  4E= boost ambiance
int theme = 0x01;     //default value at startup, 01= blue, 02=bronze
int Theme1A9Send = 0;
unsigned long ESCtimer = 0;

unsigned long aastimer = 0;
unsigned long ecotimer = 0;
bool vp2videotoogle = false;
bool vp2forcerear = false;
bool vp2forcefront = false;
bool VP2videoshowing = false;
bool ObstacleDetected = false;
bool LastObstacleDetected = false;
bool reargear = false;
bool lastreargear = false;



void setup() {
  pinMode(ECOBUTTON_PIN, INPUT_PULLUP);  // set arduino pin to input pull-up mode
  pinMode(ECOLED_PIN, OUTPUT);           // set arduino pin to output mode
  pinMode(SAMBUTTON_PIN, INPUT_PULLUP);  // set arduino pin to input pull-up mode
  pinMode(SAMLED_PIN, OUTPUT);           // set arduino pin to output mode
  pinMode(AASBUTTON_PIN, INPUT_PULLUP);  // set arduino pin to input pull-up mode
  pinMode(AASLED_PIN, OUTPUT);           // set arduino pin to output mode

  digitalWrite(ECOLED_PIN, LOW);
  digitalWrite(SAMLED_PIN, LOW);
  digitalWrite(AASLED_PIN, LOW);

  ECOcurrentButtonState = digitalRead(ECOBUTTON_PIN);
  SAMcurrentButtonState = digitalRead(SAMBUTTON_PIN);
  AAScurrentButtonState = digitalRead(AASBUTTON_PIN);



  if (SerialEnabled) {
    // Initalize Serial for debug
    Serial.begin(SERIAL_SPEED);
    // CAN-BUS to CAN2010 device(s)
    Serial.println("Initialization CAN");
  }

  CAN0.reset();
  CAN0.setBitrate(CAN_SPEED, CAN_FREQ);
  CAN0.setNormalMode();
  while (CAN0.setNormalMode() != MCP2515::ERROR_OK) {
    delay(100);
    Serial.println("can0 ok");
  }


  CAN1.reset();
  CAN1.setBitrate(CAN_SPEED, CAN_FREQ);
  CAN1.setNormalMode();
  //CAN1.setListenOnlyMode();
  while (CAN1.setNormalMode() != MCP2515::ERROR_OK) {
    delay(100);
    Serial.println("can1 ok");
  }

  //Mem default Value


  LimitMsg.can_id = 0x19B;
  LimitMsg.can_dlc = 7;
  LimitMsg.data[0] = 0x32;
  LimitMsg.data[1] = 0x48;
  LimitMsg.data[2] = 0x53;
  LimitMsg.data[3] = 0x5D;
  LimitMsg.data[4] = 0x71;
  LimitMsg.data[5] = 0x85;
  LimitMsg.data[6] = 0x80;  //limiter value sorted ascending (but optional)


  CruiseMsg.can_id = 0x1DB;
  CruiseMsg.can_dlc = 7;
  CruiseMsg.data[0] = 0x32;
  CruiseMsg.data[1] = 0x48;
  CruiseMsg.data[2] = 0x53;
  CruiseMsg.data[3] = 0x5D;
  CruiseMsg.data[4] = 0x71;
  CruiseMsg.data[5] = 0x85;
  CruiseMsg.data[6] = 0x80;  //Cruise value sorted ascending (but optional)
}

void loop() {
  if (((millis() - ButtonTimer) > 100) && ignition) {  //check button state every 100ms only if ignition is on
    ButtonTimer = millis();
    ECOlastButtonState = ECOcurrentButtonState;          // save the last state
    ECOcurrentButtonState = digitalRead(ECOBUTTON_PIN);  // read new state
    if (!ECOcurrentButtonState && ECOlastButtonState) {  //is pushed and wasnot pushed before
      //Serial.println("eco pressed");
      ecotimer = millis();
    }
    if (ECOcurrentButtonState && !ECOlastButtonState) {
      //Serial.print("eco released   ");
      if ((millis() - ecotimer) >= 800) {
        ECOsend = true;  //ask sending
        //Serial.println("  eco long press");
      } else {
        //Serial.println("eco short press");
        if (!vp2forcerear && !reargear) {
          vp2forcerear = true;
          //Serial.println(" force rear start");
        } 
        else {
          vp2forcerear = false;
          if (!reargear) { vp2videotoogle = true; }
          //Serial.println(" force rear stop");
        }
      }
    }



    SAMlastButtonState = SAMcurrentButtonState;          // save the last state
    SAMcurrentButtonState = digitalRead(SAMBUTTON_PIN);  // read new state
    if (SAMlastButtonState == HIGH && SAMcurrentButtonState == LOW) {
      SAMsend = true;  //ask sending
    }

    AASlastButtonState = AAScurrentButtonState;          // save the last state
    AAScurrentButtonState = digitalRead(AASBUTTON_PIN);  // read new state
    if (!AAScurrentButtonState && AASlastButtonState) {  //is pushed and wasnot pushed before
      //Serial.println("aas pressed");
      aastimer = millis();
    }
    if (AAScurrentButtonState && !AASlastButtonState) {
      //Serial.print("aas released   ");
      if ((millis() - aastimer) >= 800) {
        AASsend = true;  //ask sending
        //Serial.println("  aas long press");
      } else {
        if (reargear) {vp2forcefront=!vp2forcefront;}

        else if (vp2forcerear) {
          vp2forcerear = false;
          //vp2videotoogle = true;  //uncoment to close windows
          //Serial.println(" aas force rear stop");
        } else {
          vp2videotoogle = true;
          //Serial.println(" aasforce auto");
        }
      }
    }
  }

  // Receive CAN messages from the car
  if (CAN0.readMessage(&canMsgRcv) == MCP2515::ERROR_OK) {
    int id = canMsgRcv.can_id;
    int len = canMsgRcv.can_dlc;

    if (debugCAN0) {
      Serial.print("FRAME:ID=0x");
      Serial.print(id, HEX);
      Serial.print(":LEN=");
      Serial.print(len);

      char tmp[3];
      for (int i = 0; i < len; i++) {
        Serial.print(":");

        snprintf(tmp, 3, "%02X", canMsgRcv.data[i]);

        Serial.print(tmp);
      }

      Serial.println();
    }

    if (id == 0x168) {  //wiper state
      WiperActive = bitRead(canMsgRcv.data[1], 3);
      //Serial.print("WiperActive is  :  ");Serial.println(WiperActive);
    }

    if (id == 0x1A9) {                          //NAC message
      lastSAM_NAC = SAM_NAC;                    //store previous state
      SAM_NAC = bitRead(canMsgRcv.data[3], 5);  //SAM toogle read
      if (SAM_NAC && !lastSAM_NAC) {            //is pushed and wasnot pushed before
        SAMsend = true;
        //  Serial.println("SAMsend asked ");
      }

      if (!SSdesactivationDone) {                                                                 //do stuff to  deactivate SS at right time and check it worked
        if (EngineBeenStarted && !SSrequest && (millis() - EngineBeenStartedTimer > 5 * 1000)) {  //ask deactivation if conditions are meet and note time
          canMsgSnd = canMsgRcv;                                                                  //copy frame
          canMsgSnd.data[6] = bitWrite(canMsgSnd.data[6], 7, 1);                                  //D0(push)  50 not pushed  NAC toogle key for stop & start
          CAN0.sendMessage(&canMsgSnd);
          SSrequest = true;           //we note we asked
          SSrequestTimer = millis();  //time we asked
        }

        if (SSrequest && (millis() - SSrequestTimer > 1 * 1000)) {  //we check if it worked 1s after request
          SSrequest = false;                                        //cancel request because we are checking it (so if it did not worked we will ask again
          if (SSstatus) {                                           //if it worked
            SSdesactivationDone = true;                             //descativation of full loop
          }
        }
      }

      if (ECOsend || AASsend || vp2videotoogle) {  //something need to be send
        canMsgSnd = canMsgRcv;                     //copy frame
        canMsgSnd.data[6] = bitWrite(canMsgSnd.data[6], 7, ECOsend);
        ECOsend = false;  //write ECOsend value to frame
        canMsgSnd.data[3] = bitWrite(canMsgSnd.data[3], 2, AASsend);
        AASsend = false;  //write AASend value to frame
        canMsgSnd.data[7] = bitWrite(canMsgSnd.data[7], 7, vp2videotoogle);
        vp2videotoogle = false;  //write vp2videotoogle value to frame
        CAN0.sendMessage(&canMsgSnd);
      }

      if (SetLimiter && LimiterCkecked && !MemSend && (millis() - MemSendTimer > 100)) {  //we need to set cruise/limit: setxx asked+check confirmed  and MemSend is done (3rd press
        canMsgSnd = canMsgRcv;                                                            //copy frame
        canMsgSnd.data[4] = Limiterspeed;
        canMsgSnd.data[5] = 0x10;  //position 1
        canMsgSnd.data[7] = 0x08;
        CAN0.sendMessage(&canMsgSnd);

        //Serial.println("Variable reset, Limiter 1A9 sent");
        LimiterCkecked = false;
        CheckLimiter = false;
        //SetLimiter = false;
        FixMemLimit = true;
        FixMemLimitTimmer = millis();

        if (RXXpaused == 0x80) {
          PauseSend = true;  //PauseSendWaitTimer = millis();
          //Serial.println("limit PauseSend requested ");
        }
      }

      if (SetCruise && CruiseCkecked && !MemSend && (millis() - MemSendTimer > 100)) {  //we need to request cruise/limit sending (setxx asked and check confirmed)
        canMsgSnd = canMsgRcv;                                                          //copy frame
        canMsgSnd.data[4] = Cruisespeed;
        canMsgSnd.data[5] = 0x10;
        canMsgSnd.data[7] = 0x08;
        CAN0.sendMessage(&canMsgSnd);
        //Serial.println("Variable reset, Cruise 1A9 sent");
        CruiseCkecked = false;
        CheckCruise = false;
        //SetCruise = false;
        FixMemCruise = true;
        FixMemCruiseTimmer = millis();

        //if (!((RXXpaused==0x48)||(RXXpaused==0x50)||(RXXpaused==0x60))){
        if ((RXXpaused == 0x40) && !PauseSendDisable) {  //Cruise is paused (0x40) and PauseSend is allowed
          PauseSend = true;                              //PauseSendWaitTimer = millis();
          // Serial.println("cruise PauseSend requested ");
        }
        //Serial.print("PauseSendDisable :  ");Serial.println(PauseSendDisable);
        //Serial.print("PauseSend after:  ");Serial.println(PauseSend);
        PauseSendDisable = false;  //reset variable to original state
        //Serial.print("PauseSendDisable :  ");Serial.println=Theme1A9Send-1(PauseSendDisable);
      }

      if (Theme1A9Send >= 1) {
        Theme1A9Send = Theme1A9Send - 1;
        canMsgSnd = canMsgRcv;  //copy frame
        canMsgSnd.data[6] = bitWrite(canMsgSnd.data[6], 5, 1);
        CAN0.sendMessage(&canMsgSnd);
        // Serial.print("Theme1A9sent (70), new number is:  "); Serial.println(Theme1A9Send, DEC);
      }
    }

    if ((id == 0x19B) || (id == 0x267)) {  // we need to check and we received limiter ID
      if (CheckLimiter) {
        if ((canMsgRcv.data[0] == Limiterspeed)) {  //correct speed value in position 1 confirm sucess and cancel check
          LimiterCkecked = true;
          CheckLimiter = false;
          SetLimiter = true;
          //Serial.println("  LimiterCkecked confirmed ");
          //        Serial.print("  CheckLimiter  ");Serial.print(CheckLimiter); Serial.print("  LimiterCkecked  ");Serial.print(LimiterCkecked);
          //        Serial.print("  SetLimiter  ");Serial.println(SetLimiter);
        }
        if ((millis() - CheckLimiterTimer) > 2000) {  //timeout of 1s (should be enought to receive one confirmation frame) --> cancel check
          LimiterCkecked = false;
          CheckLimiter = false;
          SetLimiter = false;
          //        Serial.println("Variable reset, LimiterCkecked failure ");
          //        Serial.print("  CheckLimiter  ");Serial.print(CheckLimiter); Serial.print("  LimiterCkecked  ");Serial.print(LimiterCkecked);
          //        Serial.print("  SetLimiter  ");Serial.println(SetLimiter);
        }
      } else if (FixMemLimit && (millis() - FixMemLimitTimmer > 1 * 1000)) {  //We need to fix Mem after waiting 3s  (CheckLimiter is false)
        //canMsgSnd.can_id = 0x19B; canMsgSnd.can_dlc = 7; canMsgSnd.data[0] = 0x32; canMsgSnd.data[1] = 0x48; canMsgSnd.data[2] = 0x53; canMsgSnd.data[3] = 0x5D; canMsgSnd.data[4] = 0x71; canMsgSnd.data[5] = 0x85; canMsgSnd.data[6] = 0x80; //limiter value sorted ascending (but optional)
        CAN0.sendMessage(&LimitMsg);
        FixMemLimit = false;
        SetLimiter = false;
        //Serial.println("FixMemLimit done ");
      }

      if (!MEMfixDoneLimiter && ignition && (millis() - ignitiontimer > 5 * 1000)) {  //Right time to fix MEM: we receive MEM ID and at least 5s after start
        CAN0.sendMessage(&LimitMsg);
        MEMfixDoneLimiter = true;  //we send it only once
      }
    }

    if ((id == 0x1DB) || (id == 0x2A7)) {  // we need to check and we received Cruise ID
      if (CheckCruise) {
        if ((canMsgRcv.data[0] == Cruisespeed)) {  //correct speed value in position 1 confirm sucess and cancel check
          CruiseCkecked = true;
          CheckCruise = false;
          SetCruise = true;
          //Serial.println("  CruiseCkecked confirmed ");
          //        Serial.print("  CheckCruise  ");Serial.print(CheckCruise); Serial.print("  CruiseCkecked  ");Serial.print(CruiseCkecked);
          //        Serial.print("  SetCruise  ");Serial.println(SetCruise);
        }
        if ((millis() - CheckCruiseTimer) > 2000) {  //timeout of 1s (should be enought to receive one confirmation frame) --> cancel check
          CruiseCkecked = false;
          CheckCruise = false;
          SetCruise = false;
          //Serial.println("Variable reset, CruiseCkecked failure ");
          //        Serial.print("  CheckCruise  ");Serial.print(CheckCruise); Serial.print("  CruiseCkecked  ");Serial.print(CruiseCkecked);
          //        Serial.print("  SetCruise  ");Serial.println(SetCruise);
        }
      }

      else if (FixMemCruise && (millis() - FixMemCruiseTimmer > 1 * 1000)) {  //We need to fix Mem after waiting   (CheckCruise is false)
        //canMsgSnd.can_id = 0x1DB; canMsgSnd.can_dlc = 7; canMsgSnd.data[0] = 0x32; canMsgSnd.data[1] = 0x48; canMsgSnd.data[2] = 0x53; canMsgSnd.data[3] = 0x5D; canMsgSnd.data[4] = 0x71; canMsgSnd.data[5] = 0x85; canMsgSnd.data[6] = 0x80; //limiter value sorted ascending (but optional)
        CAN0.sendMessage(&CruiseMsg);
        FixMemCruise = false;
        SetCruise = false;
        //Serial.println("FixMemCruise done ");
      }

      if (!MEMfixDoneCruise && ignition && (millis() - ignitiontimer > 5 * 1000)) {  //Right time to fix MEM: we receive MEM ID and at least 5s after start
        CAN0.sendMessage(&CruiseMsg);
        MEMfixDoneCruise = true;  //we send it only once
      }
    }

    if (id == 0x217 && SAMsend) {  //217 received and something need to be send
      //Serial.println("217 received");
      canMsgSnd = canMsgRcv;  //copy frame
      canMsgSnd.data[3] = bitWrite(canMsgSnd.data[3], 3, SAMsend);
      SAMsend = false;
      // Serial.println("SAM struture written and SAMsend reset");
      CAN0.sendMessage(&canMsgSnd);
      //Serial.println("217 sent");
    }

    if (id == 0x2D1) {  //frame for SAM state (turn on cirocco line)
      SAMstatus = bitRead(canMsgRcv.data[0], 2);
      if (!SAMstatus && ignition) {
        digitalWrite(SAMLED_PIN, HIGH);  //turn on led
      } else {
        digitalWrite(SAMLED_PIN, LOW);  //turn off led
      }

      if (SAMstatus == 0) {  //SAM active
        canMsgSnd.can_id = 0x321;
        canMsgSnd.can_dlc = 5;
        canMsgSnd.data[0] = 0x0;
        canMsgSnd.data[1] = 0x0;
        canMsgSnd.data[2] = 0x0;
        canMsgSnd.data[3] = 0x0;
        canMsgSnd.data[4] = 0x0;
        CAN0.sendMessage(&canMsgSnd);  //send 0x321 frame to turn on indicator
      }
    }

    if (id == 0x227) {  // ID for BSI status (AAS, ESP,AFIL, StopStart)
      SSstatus = bitRead(canMsgRcv.data[3], 2);
      if (SSstatus && ignition) {
        digitalWrite(ECOLED_PIN, HIGH);  //turn on led
      } else {
        digitalWrite(ECOLED_PIN, LOW);  //turn off
      }

      AASstatus = bitRead(canMsgRcv.data[0], 6);
      if (AASstatus && ignition) {
        digitalWrite(AASLED_PIN, HIGH);  //turn on led
      } else {
        digitalWrite(AASLED_PIN, LOW);  //turn off
      }

      AFILstatus = bitRead(canMsgRcv.data[1], 4);

      //Serial.print("canMsgRcv.data[3] ");Serial.print(canMsgRcv.data[3]);Serial.print(" SSstatus is "); Serial.println(SSstatus);
    }

    if (id == 0x228) {  //RXX frame and also trigger frame for sending afil state

      canMsgSnd = canMsgRcv;  //copy frame
      IDChanged = false;
      if (DisplayMemLogo >= 1) {  //We need to display at least 1 frame (~1s)

        canMsgSnd.data[7] = bitWrite(canMsgRcv.data[7], 0, 1);  //Mem logo

        if ((LimitMode && (Limiterspeed >= 30)) || (CruiseMode && (Cruisespeed >= 40) && (vehicle_speed >= 40))) {  //Display MEM logo only if in valid range limiter speed >30 or cruise speed >40 and vehicule speed enought
          IDChanged = true;
        }
        DisplayMemLogo = DisplayMemLogo - 1;
        //Serial.println("MemLogo done and cancelled ");
      }


      if (SetLimiter || SetCruise) {                            //Display MEM logo when setting speed
        canMsgSnd.data[7] = bitWrite(canMsgRcv.data[7], 0, 1);  //Mem logo
        IDChanged = true;
      }

      if (IDChanged) {  //Send message only if we changed something
        CAN0.sendMessage(&canMsgSnd);
        IDChanged = false;
      }

      if (canMsgRcv.data[0] == 0xFF && canMsgRcv.data[1] == 0xFF) {  //when not set data0/1 are 0xFF  --> Not valid
        Speedvalid = false;
        //Serial.println("Speed is unvalid ");
      } else {
        Speedvalid = true;
        //Serial.println("Speed is valid ");
      }


      RXXpaused = canMsgRcv.data[2];
      LimitMode = bitRead(canMsgRcv.data[2], 7);
      CruiseMode = bitRead(canMsgRcv.data[2], 6);
      //     Serial.print("RXXpaused ");Serial.println(RXXpaused,HEX);
      //      Serial.print(" LimitMode ");Serial.print(LimitMode);
      //      Serial.print(" CruiseMode ");Serial.println(CruiseMode);

      canMsgSnd.can_id = 0x1E7;
      canMsgSnd.can_dlc = 8;
      canMsgSnd.data[0] = 0x00;
      canMsgSnd.data[1] = 0x19;
      canMsgSnd.data[2] = 0x65;
      canMsgSnd.data[3] = 0x00;
      canMsgSnd.data[4] = 0x00;
      canMsgSnd.data[5] = 0x00;  //binary: left green=xx10 left orange=xx11 right green=10xx right orange=11xx   0000= all grey
      canMsgSnd.data[6] = 0x00;  //00=off 40=on  80=flashing afil light
      canMsgSnd.data[7] = 0x00;

      if (fix_left_line && fix_right_line) {  //both line are detected
        AFIL_distance_threshold = 0.15 * (distance_to_left_line + distance_to_right_line - 2052) + 1026;
      }  //20% of margin (2052/1026= car width/half width) --> large road= big margin small road= small margin (20% for 3.5m road will give ~300mm)
      else {
        AFIL_distance_threshold = 1300;  //default value
      }


      if (fix_left_line == true) {                  //line detected
        canMsgSnd.data[5] = canMsgSnd.data[5] ^ 2;  //--> green xx1x
        if (distance_to_left_line < AFIL_distance_threshold) {
          canMsgSnd.data[5] = canMsgSnd.data[5] ^ 1;  //change green to orange --> xx11
          if (!(left ^ right) && (vehicle_speed >= speedthreshold)) {
            canMsgSnd.data[6] = 0x80;  //flashing afil light
          }
        }
      }

      if (fix_right_line == true) {                 //line detected
        canMsgSnd.data[5] = canMsgSnd.data[5] ^ 8;  //--> green 1xxx
        if (distance_to_right_line < AFIL_distance_threshold) {
          canMsgSnd.data[5] = canMsgSnd.data[5] ^ 4;  //change green to orange --> 11xx
          if (!(left ^ right) && (vehicle_speed >= speedthreshold)) {
            canMsgSnd.data[6] = 0x80;  //flashing afil light
          }
        }
      }

      CAN0.sendMessage(&canMsgSnd);  //send afil frame ( ID 1E7)
    }

    if (id == 0x236) {                      //ANIMATION
      if (!Animation_done && DriverDoor) {  //5s timeout
        canMsgSnd = canMsgRcv;              //copy frame
        canMsgSnd.data[5] = bitWrite(canMsgSnd.data[5], 6, 1);
        CAN0.sendMessage(&canMsgSnd);
        Animation_done = true;
      }
    }

    if (id == 0x1E9 && len == 6) {  //NAV speed
      NAVspeed = canMsgRcv.data[1];
      // NAVspeed = 55;
    }

    if (id == 0x268) {  //Id for sending speed limit

      Lastprogspeed = progspeed;

      if (CVMendspeed && (millis() - lastendspeed) < EndspeedDelay) {  //display end of speed limit
        canMsgSnd.data[0] = 0x30;                                      //any value
        //progspeed=CVMspeed;   // left commented to not change speed
        canMsgSnd.data[1] = 0x48;                    // end of speed sign
      } else if (CVMreliabity && CVMspeed < 0xFC) {  //display CVM speed in red
        canMsgSnd.data[0] = CVMspeed;
        progspeed = CVMspeed;
        canMsgSnd.data[1] = 0x10;
      } else if (CustomCVMreliabity && (NAVspeed == CVMspeed) && (NAVspeed < 0xFC)) {  //display CVM speed in red
        canMsgSnd.data[0] = NAVspeed;
        progspeed = NAVspeed;
        canMsgSnd.data[1] = 0x10;
      } else if (NAVspeed < 0xFD) {  //display NAV speed in grey  NAV speed valid
        canMsgSnd.data[0] = NAVspeed;
        progspeed = NAVspeed;
        canMsgSnd.data[1] = 0x00;
      } else if (CVMspeed > 0x05 && CVMspeed < 0xFD) {  //display modified CVM speed in grey)
        canMsgSnd.data[0] = CVMspeed - 1;
        progspeed = CVMspeed;
        canMsgSnd.data[1] = 0x00;
      } else {  //display nothing
        canMsgSnd.data[0] = 0xFF;
        progspeed = 0xFF;
        canMsgSnd.data[1] = 0x10;
      }

      if (Lastprogspeed != progspeed && (LimitMode || CruiseMode)) {  //Speed changed and limiter/cruise is active
        DisplayMemLogo = 3;                                           //number of 0x228 ID frame to sent MEM  (1 frame =~1s)
        //Serial.println("MemLogo asked ");
      }


      //      canMsgSnd.data[0] = CVMspeed;
      //      if (CVMreliabity){canMsgSnd.data[1] = 0x10;}
      //      else{canMsgSnd.data[1] = 0x00;}
      //canMsgSnd.data[1] = 0x10;   //speed type 0x00=grey sign, 0x10=red sign  0x20=grey blinking red 0x30= blinking red   0x40/0x48= end of limit grey sign
      canMsgSnd.data[2] = 0x00;
      canMsgSnd.data[3] = 0x00;
      canMsgSnd.data[4] = 0x7C;
      canMsgSnd.data[5] = 0xF8;
      canMsgSnd.data[6] = 0x00;
      canMsgSnd.data[7] = 0x00;
      canMsgSnd.can_id = 0x268;
      canMsgSnd.can_dlc = 8;
      CAN0.sendMessage(&canMsgSnd);
      //Serial.println(NAVspeed);
    }

    if (id == 0xF6) {                            // If for turning indicator ignition
      left=bitRead(canMsgRcv.data[7], 1);
      //Serial.print(" left: "); Serial.println(left);

      right=bitRead(canMsgRcv.data[7], 0);
      // Serial.print(" right: "); Serial.println(right);

      Lastingnition = ignition;
      ignition=bitRead(canMsgRcv.data[0], 3);
      if (ignition && !Lastingnition) {ignitiontimer = millis();}  //ignition switched to ON
      if (!ignition){
        SSdesactivationDone = false;  //request a new SS desactivation  if ignition is off
        EngineBeenStarted = false;    // reset EngineBeenStarted (if ignition off engine can't be running) If not reseted, on "warm" start (arduino not powered off between 2 engine start) SS will deactivate when as soon as ignition is on
      }

      //Serial.print(" left: "); Serial.print(left); Serial.print(" right: "); Serial.print(right); Serial.print(" ignition: "); Serial.println(ignition);

      lastreargear = reargear;
      reargear = bitRead(canMsgRcv.data[7], 7);
      //Serial.print("reargear : ");Serial.println(reargear);
      
      if (lastreargear != reargear) { //gear changed reset
        vp2forcefront = false;
        vp2forcerear = false;
        //Serial.println(" gear changed reset vp2forcexxx");
      }

      if (vehicle_speed>=25) { //speed over 25, nac automaticly close windows so we need to stop sending fake gear (otherwise video will reopen when going under 25)
        vp2forcefront = false;
        vp2forcerear = false;
        //Serial.println(" speed above 25 reset vp2forcexxx");
      }

      canMsgSnd = canMsgRcv;  //copy frame
      if (vp2forcerear) {canMsgSnd.data[7] = bitWrite(canMsgSnd.data[7], 7, 1);Serial.println ("vp2 rear forced writed");} //we need to change reverse gear value
      if (vp2forcefront) {canMsgSnd.data[7] = bitWrite(canMsgSnd.data[7], 7, 0);}  //we need to change front gear value
      
      //Serial.print(" 0F6 mod  was "); Serial.print(canMsgRcv.data[7], HEX); Serial.print(" new value is : ");Serial.println(canMsgSnd.data[7], HEX);

      if (canMsgSnd.data[7] != canMsgRcv.data[7]) { //send only if modified
        CAN0.sendMessage(&canMsgSnd);
        //Serial.print(" 0F6 sent,  was "); Serial.print(canMsgRcv.data[7], HEX); Serial.print(" new value is : ");Serial.println(canMsgSnd.data[7], HEX);
      }
    }

    if (id == 0xb6) {                                                      //Id for speed and rpm
      vehicle_speed = (canMsgRcv.data[2] << 8 | canMsgRcv.data[3]) / 100;  //no correction included (3km/h)
      //Serial.print("CAN_1 Vehicle speed = ");
      //Serial.println(vehicle_speed);

      rpm = (canMsgRcv.data[0] << 5 | (unsigned int)canMsgRcv.data[1] >> 3);
      if (!EngineBeenStarted && (rpm > 500)) {
        EngineBeenStarted = true;
        EngineBeenStartedTimer = millis();
      }

      //Serial.print("data0="); Serial.print(canMsgRcv.data[1]);Serial.print(" data1="); Serial.print(canMsgRcv.data[1]); Serial.print(" rpm="); Serial.println(rpm);
      //Serial.print("EngineBeenStarted= "); Serial.print(EngineBeenStarted); Serial.print(" EngineBeenStartedTimer ");Serial.println(EngineBeenStartedTimer);
    }

    if (id == 0x2E9) {  //Requested ambiance change
      //Serial.print("RCVdata1 is ");Serial.println(canMsgRcv.data[1],HEX);
      //Serial.print("ambiance is  :  "); Serial.println(ambiance, HEX);
      //Serial.print("RCVdata0 is ");Serial.println(canMsgRcv.data[0],HEX);
      //Serial.print("theme is  :  "); Serial.println(theme, HEX);

      canMsgSnd = canMsgRcv;  //copy frame
      canMsgSnd.data[0] = ((canMsgSnd.data[0] & 0xFC) | (theme & 0x03));  //theme value is only in bit0&1 =0x03 mask  0xFC is reseting SndData -->  DDDDDD00 | 000000TT   D=original data, T=theme
      canMsgSnd.data[1] = ((canMsgSnd.data[1] & 0x3F) | (ambiance & 0xC0));  //ambiance value is only in bit 6&7=0xC0 mask
      if ((canMsgRcv.data[0] != canMsgSnd.data[0]) || (canMsgRcv.data[1] != canMsgSnd.data[1])) {  //diffrent value received from NAC, we need to request the new value
        CAN0.sendMessage(&canMsgSnd);
        //Serial.print("2E9 sent with theme:  "); Serial.println(theme, HEX);
        //Serial.print("2E9 sent with ambiance:  "); Serial.println(canMsgSnd.data[1], HEX);
      }
    }

    if (id == 0x166) {  //VP2 status from ecu
      //if (canMsgRcv.data[0]==0x01 ||canMsgRcv.data[0]==0x0F){VP2videoshowing=false;}
      if (canMsgRcv.data[0] == 0x01) {
        VP2videoshowing = false;
      } else {
        VP2videoshowing = true;
      }
      //Serial.print("Active windows is: "); Serial.print (canMsgRcv.data[0], HEX); Serial.print("    VP2videoshowing is: "); Serial.println(VP2videoshowing);
    }

    if (id == 0x0E1) {  //AAS state
      LastObstacleDetected = ObstacleDetected;
      if (canMsgRcv.data[2] == 0x3F) {
        ObstacleDetected = false;
      } else {
        ObstacleDetected = true;
      }
      //Serial.print("    ObstacleDetected is: "); Serial.print(ObstacleDetected);Serial.println(LastObstacleDetected);
      if (!LastObstacleDetected && ObstacleDetected && !VP2videoshowing) {
        vp2videotoogle = true;
        //Serial.println("new ObstacleDetected request videotoogle");
      }
    }
  }


  // Listen on CVM CAN BUS
  if (CAN1.readMessage(&canMsgRcv) == MCP2515::ERROR_OK) {
    int id = canMsgRcv.can_id;
    int len = canMsgRcv.can_dlc;

    if (debugCAN1) {
      Serial.print("FRAME:ID=0x");
      Serial.print(id, HEX);
      Serial.print(":LEN=");
      Serial.print(len);

      char tmp[3];
      for (int i = 0; i < len; i++) {
        Serial.print(":");

        snprintf(tmp, 3, "%02X", canMsgRcv.data[i]);

        Serial.print(tmp);
      }
      Serial.println();
    }

    if (id == 0x11C) {  //CVM info ID 0x11C (HEX) or 284 (DEC)

      LastCVMreliabity = CVMreliabity;
      CVMspeed = canMsgRcv.data[2];
      CVMreliabity = bitRead(canMsgRcv.data[3], 5);
      CVMendspeed = bitRead(canMsgRcv.data[3], 6);

      if (CVMendspeed && !boolendspeed) {  //first read of end of speed sending
        boolendspeed = true;
        lastendspeed = millis();
        // Serial.println("millis reset");
      }

      if (!CVMendspeed && (millis() - lastendspeed) > EndspeedDelay) {  //3s wait for
        boolendspeed = false;
      }

      if (LastCVMreliabity && !CVMreliabity) {  //CVMreliabity became false & NAV=CVM speed --> store nav speed and enable cutom relibility
        RefNAVspeed = NAVspeed;
        CustomCVMreliabity = true;
      }

      if (CustomCVMreliabity) {  //CVMreliabity became true or RefNAVspeed is diffrent than NAV speed -->reset refnav speedif
        if (RefNAVspeed != NAVspeed) {
          CustomCVMreliabity = false;
        }
      }
    }

    if (id == 0x5c) {  //CVM left line data
      if ((canMsgRcv.data[5] & 80 == 80) or (canMsgRcv.data[5] & 1 == 1)) {
        // fix left line
        type_left_line = canMsgRcv.data[5] & 0xE;
        distance_to_left_line = ((canMsgRcv.data[6] << 8) + canMsgRcv.data[7]) / 4;  // in mm
        fix_left_line = true;
      } else {
        fix_left_line = false;
      }
    }

    if (id == 0x9c) {  //CVM right line data
      if ((canMsgRcv.data[5] & 80 == 80) or (canMsgRcv.data[5] & 1 == 1)) {
        // fix right line
        type_right_line = canMsgRcv.data[5] & 0xE;
        distance_to_right_line = (0xFFFF - (canMsgRcv.data[6] << 8) + canMsgRcv.data[7]) / 4;  // in mm
        fix_right_line = true;
      } else {
        fix_right_line = false;
      }
    }

    if (id == 0x94) {  //VCI state for cruise/limier
      LastMemState = MemState;
      MemState = (bitRead(canMsgRcv.data[7], 7) == 1);  //change to blackpanel for testing prupose
      //Serial.println(MemState);

      if (MemState && !LastMemState) {  //is pushed and wasnot pushed before
        //Serial.println("Mempressed");

        if (MemState && (millis() - MemStateTimer) < 1 * 1000) {  //1s delay for mem double press detection
          //Serial.println(" Mem double press detected");
          //Serial.print(" can data3: "); Serial.println(canMsgRcv.data[3],HEX);

          if ((canMsgRcv.data[3] == 0x80) && !SetLimiter) {  //ask set limiter and send mem value + request additional Mem Press   only if not already done    need check of valid speed and 1a9 not working when changing speed
            //progspeed=134;

            //insert transformation if needed +3km/h   + check of value (40km/h min for cruiseSetLimiter=true;
            if (WiperActive) {  //remove 20 for 130 and 10 for 90/110
              if (progspeed >= 129) {
                Limiterspeed = (progspeed - 20);
              } else if (progspeed >= 89) {
                Limiterspeed = progspeed - 10;
              } else {
                Limiterspeed = progspeed;
              }
            } else {
              Limiterspeed = progspeed;
            }
            //Serial.print("limiterspeedrain: "); Serial.println(Limiterspeed);

            if (Limiterspeed >= 80) {
              Limiterspeed = Limiterspeed + 3;
            } else if (Limiterspeed >= 70) {
              Limiterspeed = Limiterspeed + 2;
            }
            //else {Limiterspeed = progspeed;}
            //Serial.print("limiterspeedrain+offset: "); Serial.println(Limiterspeed);


            //Serial.println(Limiterspeed);
            if ((Limiterspeed >= 30) && (Limiterspeed <= 180)) {  //speed in range allowed by NAC zone 211b
              LimiterCkecked = false;
              CheckLimiter = false;
              //Serial.print("SetLimiter asked");
              //Limiterspeed=42;
              canMsgSnd.can_id = 0x19b;
              canMsgSnd.can_dlc = 7;
              canMsgSnd.data[0] = Limiterspeed;
              canMsgSnd.data[1] = Limiterspeed;
              canMsgSnd.data[2] = Limiterspeed;
              canMsgSnd.data[3] = Limiterspeed;
              canMsgSnd.data[4] = Limiterspeed;
              canMsgSnd.data[5] = Limiterspeed;
              canMsgSnd.data[6] = 0x80;
              CAN0.sendMessage(&canMsgSnd);
              //Serial.print("Mem value request sent with "); Serial.println(Limiterspeed);
              CheckLimiter = true;
              CheckLimiterTimer = millis();  //ask cheking and note time
              MemSend = true;
              MemSendWaitTimer = millis();
              //Serial.println("MemSend requested ");
            }
          }

          if ((canMsgRcv.data[3] == 0x40) && !SetCruise) {  //ask set Cruise and send mem value   only if not already done    need check of valid speed and 1a9 not working when changing speed
            //progspeed=29;

            //insert transformation if needed +3km/h   + check of value (40km/h min for cruiseSetLimiter=true;
            if (WiperActive) {  //remove 20 for 130 and 10 for 90/110
              if (progspeed >= 129) {
                Cruisespeed = progspeed - 20;
              } else if (progspeed >= 89) {
                Cruisespeed = progspeed - 10;
              } else {
                Cruisespeed = progspeed;
              }
            } else {
              Cruisespeed = progspeed;
            }

            if (Cruisespeed >= 80) {
              Cruisespeed = Cruisespeed + 3;
            } else if (Cruisespeed >= 70) {
              Cruisespeed = Cruisespeed + 2;
            }
            //else {Cruisespeed = progspeed;}


            //Serial.println(Cruisespeed);
            if ((Cruisespeed >= 40) && (Cruisespeed <= 180)) {  //speed in range allowed by NAC zone 211b
              CruiseCkecked = false;
              CheckCruise = false;
              //Serial.print("SetCruise asked");
              //Cruisespeed=42;
              canMsgSnd.can_id = 0x1db;
              canMsgSnd.can_dlc = 7;
              canMsgSnd.data[0] = Cruisespeed;
              canMsgSnd.data[1] = Cruisespeed;
              canMsgSnd.data[2] = Cruisespeed;
              canMsgSnd.data[3] = Cruisespeed;
              canMsgSnd.data[4] = Cruisespeed;
              canMsgSnd.data[5] = Cruisespeed;
              canMsgSnd.data[6] = 0x80;
              CAN0.sendMessage(&canMsgSnd);
              //Serial.print("  Mem value request sent with "); Serial.println(Cruisespeed);
              CheckCruise = true;
              CheckCruiseTimer = millis();  //ask cheking and note time

              if (!Speedvalid) {  //speed is not set
                PlusSend = true;  //PlusSendWaitTimer = millis();
                MemSend = true;
                MemSendWaitTimer = millis();  //To put inside Plus send???
                PauseSendDisable = true;
                //Serial.print("PlusSend requested    "); Serial.println("MemSend requested ");
              } else {
                MemSend = true;
                MemSendWaitTimer = millis();
                //Serial.println("MemSend requested    without pausesend disable");
              }
            }
          }
        }

        MemStateTimer = millis();
      }

      if (MemSend && ((millis() - MemSendWaitTimer) > 200)) {   //send MemSend
        canMsgSnd = canMsgRcv;                                  //copy frame
        canMsgSnd.data[7] = bitWrite(canMsgRcv.data[7], 7, 1);  //Mem Key ID
        CAN1.sendMessage(&canMsgSnd);
        MemSend = false;
        MemSendTimer = millis();
        //Serial.println("MemSend done and cancelled ");
      }

      //if (PauseSend && ((millis() - PauseSendWaitTimer) >0) ) { //Pause MemSend
      if (PauseSend) {                                          //Pause MemSend
        canMsgSnd = canMsgRcv;                                  //copy frame
        canMsgSnd.data[3] = bitWrite(canMsgRcv.data[3], 3, 1);  //Pause Key ID
        CAN1.sendMessage(&canMsgSnd);
        PauseSend = false;
        //Serial.println("Pause Send done and cancelled ");
      }

      //if (PlusSend && ((millis() - PlusSendWaitTimer) >0)) { //send MemSend
      if (PlusSend) {                                           //send MemSend
        canMsgSnd = canMsgRcv;                                  //copy frame
        canMsgSnd.data[3] = bitWrite(canMsgRcv.data[3], 5, 1);  //5=+ Key ID   4=-Key ID
        CAN1.sendMessage(&canMsgSnd);
        PlusSend = false;
        //Serial.println("Plus Send done and cancelled ");
      }
    }

    if (id == 0x00E) {  //door state
      DriverDoor = bitRead(canMsgRcv.data[1], 6);
      //Serial.print("DriverDoor is  :  ");Serial.println(DriverDoor);
    }

    if (id == 0xA2) {  //VCI state lower right (ESC)
      LastEscState = EscState;
      EscState = bitRead(canMsgRcv.data[1], 4);  //ESC key
      //Serial.println(EscState);
      if (EscState && !LastEscState) {  //is pushed and wasnot pushed before
        //Serial.println("ESC pressed");
        ESCtimer = millis();
      }
      if (!EscState && LastEscState) {
        //Serial.println("ESC released");
        if ((millis() - ESCtimer) >= 1000) {
          // Serial.println("ESC long press");
          Theme1A9Send = 2;  //number of time to send 70 value (2time on nac)
          switch (theme) {
            case 0x01: theme = 0x02; break;   //Switch blue to bzonze
            case 0x02: theme = 0x01; break;   //Switch bzonze to blue
            default: ambiance = 0x01; break;  //return to off for any other value
          }
          //Serial.print("Theme is  :  "); Serial.println(theme, HEX);

        } else {
          //Serial.println("ESC short press");
          switch (ambiance) {
            case 0x0E: ambiance = 0x8E; break;  //Switch off to relax
            case 0x8E: ambiance = 0x4E; break;  //Switch relax to boost
            case 0x4E: ambiance = 0x0E; break;  //Switch boost to off
            default: ambiance = 0x0E; break;    //return to off for any other value
          }
          //Serial.print("ambiance is  :  "); Serial.println(ambiance, HEX);
        }
      }
    }
  }
}
