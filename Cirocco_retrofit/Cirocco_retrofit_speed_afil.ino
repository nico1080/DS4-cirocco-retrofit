//Sketch by Nico1080
//This skecth was tested on a DS4, and should also work on any C4 gen 2 (hatcback, sedan, etc)
//Many thank to:
//Keryan (https://www.forum-peugeot.com/Forum/members/keryan.126947/) for code insipration 
//Cesenate (https://www.drive2.ru/users/cesenate) for the HBA cluster CAN ID from DS5  (ID 0x217 Byte 3, value 0x80)
//Infizer for many items  (https://github.com/infizer91/can_extender  , https://www.drive2.ru/users/infizer/ , https://telecoder.ru/index.php )
//And all I have forgotten 
//
//Feel free to use this code and improve it. In return I only ask that you share your improvement (and obviously that you don't sell it)
//
//The function of this sktech:
//Make AAS (Aide Au Stationement, parking sensor) and SAM (Surveillance Angles Mort, blind spots monitor) NAC toogle switch work
//Speed limit display from CVM and NAV data (see comment)
//Fix SAM light on cirocco (frame 0x321)  (Thanks to Infizer)
//Fix animation on cirocco (frame 0x236)  (Thanks to Infizer)
//Display green/orange line on cirrocco (AFIL and LKA need to be activated in cirocco) (Thanks to Infizer)
//Make AFIL indicator flash when lines are being crossed without turn indicator (see comment)(Thanks to Infizer)
//
//
//For speed limit display it follow this logic:
//-show end of speedlimit sign for 3s when it is read by CVM
//-show CVM speed when it is reliable (red sign)
//-keep showing CVM speed in red sign after CVM loose reliability. Only if NAV speed have the same value and has not changed since it lost CVM reliability (CVM reliability is easily lost so I "extend" it)
//-show NAV speed (grey sign)
//-if no nav speed is avilable it display last read sign (grey sign and remove 1km/h. Example last sign=70km/h display 69 in grey sign)
//-If no previous sign (and no nav speed) it display nothing

//For AFIL  (Alerte Franchissement Involontaire de Ligne, lign dectection):
//display green orange line as soon as they are detected at any time (require LKA activated in cirocco)
//Flash AFIL light when one line become orange and if one(and not 2) turning indicator is ON  and when speed is over 50km/h (require AFIL activated in cirocco)
//When left/right lines are both detected: margin calculation for AFIL threeshold (large road= big margin small road= small margin) default value is 1300.

/////////////////////
//    Libraries    //
/////////////////////

#include <SPI.h>
#include <mcp2515.h> // https://github.com/autowp/arduino-mcp2515 + https://github.com/watterott/Arduino-Libs/tree/master/digitalWriteFast

/////////////////////
//  Configuration  //
/////////////////////

#define CS_PIN_CAN0 10   //pin connected to NAC/cirocco CAN bus module
#define CS_PIN_CAN1 9    //pin connected to CVM CAN bus module
#define SERIAL_SPEED 115200
#define CAN_SPEED CAN_125KBPS // Diagnostic CAN bus - High Speed
#define CAN_FREQ MCP_8MHZ // Switch to 16MHZ if you have a 16Mhz module


MCP2515 CAN0(CS_PIN_CAN0); // NAC/cirocco CAN bus
MCP2515 CAN1(CS_PIN_CAN1); // CVM CAN bus


////////////////////
//   Variables    //
////////////////////

// My variables
bool debugCAN0 = false;   //Debug for NAC/cirocco CAN bus
bool debugCAN1 = false;   //Debug CVM CAN bus
bool SerialEnabled = true;

//Bool for avoiding duplicate push
bool SAMsend = false;
long lastSAM_NAC;


int NAVspeed = 0xff;
int CVMspeed = 0xff;
bool CVMreliabity = false;
bool LastCVMreliabity = false;
bool CustomCVMreliabity = false;
int RefNAVspeed = 0xff;
bool CVMendspeed = false;
bool boolendspeed = false;
long lastendspeed; //timer for stop display of speed limit
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
int AFIL_distance_threshold; //AFIL threshold (include car half width  (1300mm = 300m form line if car width  is 2m)

bool left = false;
bool right = false;
int vehicle_speed = 0;  // current vehicle speed
int speedthreshold = 53; //thresold to activate afil light (include 3km/h correction)

bool Animation_done = false;  //set to true to not do animation


// CAN-BUS Messages
struct can_frame canMsgSnd;
struct can_frame canMsgRcv;

void setup() {
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


}

void loop() {
  // Receive CAN messages from the car
  if (CAN0.readMessage( & canMsgRcv) == MCP2515::ERROR_OK) {
    int id = canMsgRcv.can_id;
    int len = canMsgRcv.can_dlc;

    if (debugCAN0 ) {
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
    if (id == 0x1A9) {
      if (bitRead(canMsgRcv.data[3] , 5 ) == 1 ) {
        //   Serial.println("SAM is pushed on NAC (1A9)");
        if (millis() - lastSAM_NAC > 300) {  //300ms wait for button
          SAMsend = true;
          lastSAM_NAC = millis();
          //  Serial.println("SAMsend asked and wait xxms");
        }
      }

    }
    if (id == 0x217 && SAMsend) {  //217 received and something need to be send
      //Serial.println("217 received");
      canMsgSnd = canMsgRcv;   //copy frame
      if (SAMsend) {
        bitWrite(canMsgSnd.data[3] , 3, 1 );
        SAMsend = false;
        // Serial.println("SAM struture written and SAMsend reset");
      }

      CAN0.sendMessage( & canMsgSnd);
      //Serial.println("217 sent");
    }
    if (id == 0x2D1) {     //frame for SAM state
      if (bitRead(canMsgRcv.data[0] , 2 ) == 0 ) { //SAM active
        canMsgSnd.can_id = 0x321;
        canMsgSnd.can_dlc = 5;
        canMsgSnd.data[0] = 0x0;
        canMsgSnd.data[1] = 0x0;
        canMsgSnd.data[2] = 0x0;
        canMsgSnd.data[3] = 0x0;
        canMsgSnd.data[4] = 0x0;
        CAN0.sendMessage( & canMsgSnd);  //send 0x321 frame to turn on indicator
      }
    }

    if (id == 0x228) {     //trigger frame for sending afil state
      canMsgSnd.can_id = 0x1E7;
      canMsgSnd.can_dlc = 8;
      canMsgSnd.data[0] = 0x00;
      canMsgSnd.data[1] = 0x19;
      canMsgSnd.data[2] = 0x65;
      canMsgSnd.data[3] = 0x00;
      canMsgSnd.data[4] = 0x00;
      canMsgSnd.data[5] = 0x00; //binary: left green=xx10 left orange=xx11 right green=10xx right orange=11xx   0000= all grey
      canMsgSnd.data[6] = 0x00;  //00=off 40=on  80=flashing afil light
      canMsgSnd.data[7] = 0x00;

      if (fix_left_line && fix_right_line){   //both line are detected
        AFIL_distance_threshold=0.2*(distance_to_left_line + distance_to_right_line-2052)+1026;  //20% of margin (2052/1026= car width/half width) --> large road= big margin small road= small margin (20% for 3.5m road will give ~300mm)
      }
      else{
        AFIL_distance_threshold=1300; //default value
      }

      if (fix_left_line == true)
      {
        canMsgSnd.data[5] = canMsgSnd.data[5] ^ 2;   //--> green xx1x

        if (distance_to_left_line < AFIL_distance_threshold)
        {
          canMsgSnd.data[5] = canMsgSnd.data[5] ^ 1;  //change green to orange --> xx11

          if (!(left ^ right)&&(vehicle_speed >= speedthreshold))
          {
            canMsgSnd.data[6] = 0x80; //flashing afil light
          }
//          else
//          {
//            canMsgSnd.data[6] = 0;
//          }
        }
      }

      if (fix_right_line == true)
      {
        canMsgSnd.data[5] = canMsgSnd.data[5] ^ 8;   //--> green 1xxx

        if (distance_to_right_line < AFIL_distance_threshold)
        {
          canMsgSnd.data[5] = canMsgSnd.data[5] ^ 4;  //change green to orange --> 11xx

          if (!(left ^ right)&&(vehicle_speed >= speedthreshold))
          {
            canMsgSnd.data[6] = 0x80; //flashing afil light
          }
//          else
//          {
//            canMsgSnd.data[6] = 0;
//          }
        }
      }




      CAN0.sendMessage( & canMsgSnd);  //send afil frame (1E7)

    }



    if (id == 0x236)    //ANIMATION
    { if (!Animation_done && millis() > 5000)    //5s timeout
      {
        canMsgSnd = canMsgRcv;   //copy frame
        canMsgSnd.data[5] = bitWrite(canMsgSnd.data[5], 6, 1);
        CAN0.sendMessage( & canMsgSnd);
        Animation_done = true;
      }
    }


    if (id == 0x1E9 && len == 6) {
      NAVspeed = canMsgRcv.data[1];
      // NAVspeed = 55;
    }

    if (id == 0x268) {  //Id for speed limit

      if (CVMendspeed && (millis() - lastendspeed) < EndspeedDelay) {  //display end of speed limit
        canMsgSnd.data[0] = 0x30; //any value
        canMsgSnd.data[1] = 0x48; // end of speed sign
      }
      else if (CVMreliabity && CVMspeed < 0xFC) { //display CVM speed in red
        canMsgSnd.data[0] = CVMspeed;
        canMsgSnd.data[1] = 0x10;
      }
      else if (CustomCVMreliabity && (NAVspeed == CVMspeed) && (NAVspeed < 0xFC)) { //display CVM speed in red
        canMsgSnd.data[0] = NAVspeed;
        canMsgSnd.data[1] = 0x10;
      }
      else if (NAVspeed < 0xFD) { //display NAV speed in grey  NAV speed valid
        canMsgSnd.data[0] = NAVspeed;
        canMsgSnd.data[1] = 0x00;
      }
      else if (CVMspeed > 0x05 && CVMspeed < 0xFD) { //display modified CVM speed in grey)
        canMsgSnd.data[0] = CVMspeed - 1;
        canMsgSnd.data[1] = 0x00;
      }
      else  {  //display nothing
        canMsgSnd.data[0] = 0xFF;
        canMsgSnd.data[1] = 0x10;
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
      CAN0.sendMessage( & canMsgSnd);
      //Serial.println(NAVspeed);
    }

    if (id == 0xF6)  // Turning indicator
    {
      if ((canMsgRcv.data[7] & 2) == 2) {
        left = true;
        //Serial.println("left on");
      }
      else
      {
        left = false;
      }

      if ((canMsgRcv.data[7] & 1) == 1) {
        right = true;
        //Serial.println("right on");
      }
      else
      {
        right = false;
      }
//      if (right && left){  // if both are true= warning
//        left = false;
//        right = false;
//      }
    }

if (id == 0xb6)
  {
    vehicle_speed = (canMsgRcv.data[2] << 8 | canMsgRcv.data[3]) / 100;  //no correction included (3km/h)
    //Serial.print("CAN_1 Vehicle speed = ");
    //Serial.println(vehicle_speed);
  }
  }

  // Listen on CVM CAN BUS
  if (CAN1.readMessage( & canMsgRcv) == MCP2515::ERROR_OK) {
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

    if (id == 0x11C) {    //CVM info ID 0x11C (HEX) or 284 (DEC)

      LastCVMreliabity = CVMreliabity;
      CVMspeed = canMsgRcv.data[2];
      CVMreliabity = bitRead(canMsgRcv.data[3] , 5 );
      CVMendspeed = bitRead(canMsgRcv.data[3] , 6 );

      if (CVMendspeed && !boolendspeed) { //first read of end of speed sending
        boolendspeed = true;
        lastendspeed = millis();
        // Serial.println("millis reset");
      }

      if (!CVMendspeed && (millis() - lastendspeed) > EndspeedDelay) { //3s wait for
        boolendspeed = false;
      }

      //if (LastCVMreliabity &&!CVMreliabity){ //CVMreliabity became false  store nav speed
      //  RefNAVspeed=NAVspeed;
      //  if(RefNAVspeed=CVMspeed){ CustomCVMreliabity= true;}
      //}
      //if ((!LastCVMreliabity &&CVMreliabity)||!(RefNAVspeed=NAVspeed)){ //VMreliabity became true or NAV speed changed  reset refnav speed
      //  //RefNAVspeed=0x100;
      //  CustomCVMreliabity=false;
      //}

      if (LastCVMreliabity && !CVMreliabity) { //CVMreliabity became false & NAV=CVM speed --> store nav speed and enable cutom relibility
        RefNAVspeed = NAVspeed;
        CustomCVMreliabity = true;
      }
      //if (!LastCVMreliabity &&CVMreliabity){ //CVMreliabity became true or RefNAVspeed is diffrent than NAV speed -->reset refnav speedif
      //    CustomCVMreliabity=false;
      //}
      if (CustomCVMreliabity) { //CVMreliabity became true or RefNAVspeed is diffrent than NAV speed -->reset refnav speedif
        if (RefNAVspeed != NAVspeed ) {
          CustomCVMreliabity = false;
        }
      }

    }

    if (id == 0x5c)   //CVM left line data
    {

      if ((canMsgRcv.data[5] & 80 == 80) or (canMsgRcv.data[5] & 1 == 1))
      {
        // fix left line
        type_left_line = canMsgRcv.data[5] & 0xE;
        distance_to_left_line = ((canMsgRcv.data[6] << 8) + canMsgRcv.data[7]) / 4; // in mm



        fix_left_line = true;
      }
      else
      {
        fix_left_line = false;
      }
    }

    if (id == 0x9c)   //CVM right line data
    {

      if ((canMsgRcv.data[5] & 80 == 80) or (canMsgRcv.data[5] & 1 == 1))
      {
        // fix right line
        type_right_line = canMsgRcv.data[5] & 0xE;
        distance_to_right_line = (0xFFFF-(canMsgRcv.data[6] << 8) + canMsgRcv.data[7]) / 4; // in mm



        fix_right_line = true;
      }
      else
      {
        fix_right_line = false;
      }
    }

  }

}
