
//Sketch by Nico1080
//This sketch will make the AAS (parking sensor) and SAM (blind spot monitoring) toogle key on NAC work (BSI telecoding for AAS is optional)
//It also allow physical button and LED to work again. Check the connection yourself. Beware that LED are in +12v logic
//This skecth was tested on a DS4, and should also work on any C4 gen 2 (hatcback, sedan, etc)
//HBA (High Beam Assist) do not work on DS4/C4 (car ignore CAN message from cluster), but it should be possible to make it work on DS5
//Many thank to Keryan (https://www.forum-peugeot.com/Forum/members/keryan.126947/) for code insipration and to Cesenate (https://www.drive2.ru/users/cesenate) for the HBA cluster CAN ID from DS5  (ID 0x217 Byte 3, value 0x80)
//Feel free to use this code and improve it. In return I only ask that you share your improvement (and obviously that you don't sell it)


/////////////////////
//    Libraries    //
/////////////////////

#include <EEPROM.h>
#include <SPI.h>
#include <mcp2515.h> // https://github.com/autowp/arduino-mcp2515 + https://github.com/watterott/Arduino-Libs/tree/master/digitalWriteFast

/////////////////////
//  Configuration  //
/////////////////////

#define CS_PIN_CAN0 10
#define SERIAL_SPEED 115200
#define CAN_SPEED CAN_125KBPS // Diagnostic CAN bus - High Speed
#define CAN_FREQ MCP_8MHZ // Switch to 16MHZ if you have a 16Mhz module


// LED and button input
//const int HBABUTTON_PIN = A3; // Arduino pin connected to HBA button's pin
//const int HBALED_PIN    = 6; // Arduino pin connected to HBA LED's pin
const int SAMBUTTON_PIN = A4; // Arduino pin connected to SAM button's pin
const int SAMLED_PIN    = 5; // Arduino pin connected to SAM LED's pin
const int AASBUTTON_PIN = A5; // Arduino pin connected to AAS button's pin
const int AASLED_PIN    = 3; // Arduino pin connected to AAS LED's pin


////////////////////
// Initialization //
////////////////////

MCP2515 CAN0(CS_PIN_CAN0); // CAN-BUS Shield N°2

////////////////////
//   Variables    //
////////////////////

// My variables
bool debugCAN0 = false;
bool SerialEnabled = true;

// timer for physical switch
long lastSAM;
long lastSAM_NAC;
long lastAAS;

//Bool for avoiding duplicate push
bool SAMsend = false;
bool SAMpushrelease = true;

bool AASsendNAC = false;
bool AASsendCLUSTER = false;
bool AASpushrelease = true;

// CAN-BUS Messages
struct can_frame canMsgSnd;
struct can_frame canMsgRcv;

void setup() {
 // pinMode(HBALED_PIN, OUTPUT);          // set arduino pin to output mode
  pinMode(SAMLED_PIN, OUTPUT);          // set arduino pin to output mode
  pinMode(AASLED_PIN, OUTPUT);          // set arduino pin to output mode
 // digitalWrite(HBALED_PIN, LOW); // Initial state of HBA LED
  digitalWrite(SAMLED_PIN, LOW); // Initial state of SAM LED
  digitalWrite(AASLED_PIN, LOW); // Initial state of AAS LED



  if (SerialEnabled) {
    // Initalize Serial for debug
    Serial.begin(SERIAL_SPEED);
    // CAN-BUS to CAN2010 device(s)
    Serial.println("Initialization CAN0");
  }

  CAN0.reset();
  CAN0.setBitrate(CAN_SPEED, CAN_FREQ);
  while (CAN0.setNormalMode() != MCP2515::ERROR_OK) {
    delay(100);
    Serial.println("can ok");
  }
}

void loop() {
  // Receive CAN messages from the car
  if (CAN0.readMessage( & canMsgRcv) == MCP2515::ERROR_OK) {
    int id = canMsgRcv.can_id;
    int len = canMsgRcv.can_dlc;

    if (debugCAN0 ) {
      Serial.print("FRAME:ID=");
      Serial.print(id);
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
        Serial.println("SAM is pushed on NAC (1A9)");
        if (millis() - lastSAM_NAC > 300) {  //300ms wait for button
          SAMsend = true;
          lastSAM_NAC = millis();
          Serial.println("SAMsend asked and wait xxms");
        }
      }
      if (AASsendNAC) { //AAS need to activated with NAC frame
        canMsgSnd = canMsgRcv;   //copy frame
        bitWrite(canMsgSnd.data[3] , 2, 1 );
        AASsendNAC = false;
        CAN0.sendMessage( & canMsgSnd);
        Serial.println("1A9 sent for AAS");
      }
    }
    if (id == 0x2D1) {     //frame for SAM state
      if (bitRead(canMsgRcv.data[0] , 2 ) == 0 ) { //SAM active
        digitalWrite(SAMLED_PIN, HIGH); // Turn ON SAM LED
      }
      else {
        digitalWrite(SAMLED_PIN, LOW);  // Turn OFF SAM LED
      }

    }

    if (id == 0x227) {     //frame for AAS state
      if (bitRead(canMsgRcv.data[0] , 6 ) == 1 ) { //AAS desactivated
        digitalWrite(AASLED_PIN, HIGH); // Turn ON AAS LED
      }
      else  {
        digitalWrite(AASLED_PIN, LOW); // Turn OFF AAS LED
      }
    }

    if (millis() - lastSAM > 500) {  //500ms wait for button
      if (digitalRead(SAMBUTTON_PIN) == LOW && SAMpushrelease) {
        SAMsend = true;
        Serial.println("SAMsend from switch");
        lastSAM = millis();
        SAMpushrelease = false;
      }
      else if (digitalRead(SAMBUTTON_PIN) == HIGH) {
        SAMpushrelease = true;
        //Serial.println("SAM switch released");
      }
    }
    if (millis() - lastAAS > 500) {  //500ms wait for button
      if (digitalRead(AASBUTTON_PIN) == LOW && AASpushrelease) {
        AASsendNAC = true;
        AASsendCLUSTER = true;
        Serial.println("AASsend from switch");
        lastAAS = millis();
        AASpushrelease = false;
      }
      else if (digitalRead(AASBUTTON_PIN) == HIGH) {
        AASpushrelease = true;
      }
    }


    if (id == 0x217 && (SAMsend || AASsendCLUSTER )) {  //217 received and something need to be send
      //Serial.println("217 received");
      canMsgSnd = canMsgRcv;   //copy frame
      if (SAMsend) {
        bitWrite(canMsgSnd.data[3] , 3, 1 );
        SAMsend = false;
        // Serial.println("SAM struture written and SAMsend reset");
      }
      if (AASsendCLUSTER) {
        bitWrite(canMsgSnd.data[2] , 7, 1 );
        AASsendCLUSTER = false;
        //
      }
      CAN0.sendMessage( & canMsgSnd);
      //Serial.println("217 sent");
    }

  }
}
