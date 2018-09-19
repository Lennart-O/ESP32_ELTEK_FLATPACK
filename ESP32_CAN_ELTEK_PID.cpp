//Code by Lennart O.

//include libraries

#include <BlynkSimpleEsp32_BLE.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <ESP32CAN.h>
#include <CAN_config.h>
#include <Arduino.h>
#include <PID_v1.h>

//define constants and variables

word outputvoltage = 5760; //variable for charger voltage setpoint, set startup voltage to 57,6V (offset = 0,01)
word outputcurrent = 175; //variable for charger current setpoint, set startup current to 17,5A (offset = 0,1)
word overvoltage = 5850;

double PIDSetpoint1, PIDInput1, PIDOutput1; // parameters for the PID controller
unsigned char serialnr[8] = {0x12,0x35,0x71,0x10,0x16,0x79,0x00,0x00}; //Flatpack Seriennummer
unsigned long int login_ID = 0x05004804;
unsigned long int send_ID = 0x05FF4004;

int pv_tempin; //Temperature in from flatpack
int pv_tempout; //Temperature out from flatpack
int pv_inputvoltage; //input voltage from flatpack
float pv_voltage; //output voltage from flatpack
float pv_current; //output current from flatpack

int charger_enable = HIGH;
String charger_status = "Laden ein";

CAN_frame_t rx_frame;
CAN_frame_t tx_frame;

#define BLYNK_PRINT Serial
#define BLYNK_USE_DIRECT_CONNECT

#define SWITCH  1  // V1, Charger on/off switch
#define V_SLIDER  2 // V2, voltage setting slider
#define C_SLIDER  3 // V3, current setting slider
#define V_DISPLAY 4 // V4, present charge voltage display
#define C_DISPLAY 5 // V5, present charge current display
#define VI_DISPLAY 6 // V6, present charger input voltage display
#define TI_DISPLAY 7 // V7, present charge temp in display
#define TO_DISPLAY 8 // V8, present charger temp out display
#define S_DISPLAY 9 // V9, present charger status display

char auth[] = ""; //put your Blynk auth key in here

//define objects

CAN_device_t CAN_cfg; //can bus object
BlynkTimer timer; //timer object
PID myPID1(&PIDInput1, &PIDOutput1, &PIDSetpoint1, 0.8,0.8,0,DIRECT);   // PID controller settings

BLYNK_CONNECTED() {

Blynk.syncAll(); // Request Blynk server to re-send latest values for all pins

}

/************************************************
** Function name:           setVoltage
** Descriptions:            set target voltage
*************************************************/

void setVoltage(int t_voltage) { //can be used to set desired voltage to i.e. 80% SOC

  if(t_voltage >= 4750.0 && t_voltage <= 5760.0){ //makes sure voltage is in range of Emco scooter

    outputvoltage = t_voltage;

  }

 }

/************************************************
** Function name:           setCurrent
** Descriptions:            set target current
*************************************************/

void setCurrent(int t_current) { //can be used to reduce or adjust charging speed

  if(t_current >= 0.0 && t_current <= 420.0){ //makes sure current is in range of Charger

    outputcurrent = t_current;

  }

}

/************************************************
** Function name:           BLYNK_READ
** Descriptions:            send present charge voltage to app
*************************************************/

BLYNK_READ(V_DISPLAY){ //Blynk app has something on V4

  Blynk.virtualWrite(V_DISPLAY, pv_voltage); //sending to Blynk

}

/************************************************
** Function name:           BLYNK_READ
** Descriptions:            send present charge current to app
*************************************************/

BLYNK_READ(C_DISPLAY){ //Blynk app has something on V5

  Blynk.virtualWrite(C_DISPLAY, pv_current); //sending to Blynk

}

/************************************************
** Function name:           BLYNK_READ
** Descriptions:            send present inputvoltage to app
*************************************************/

BLYNK_READ(VI_DISPLAY){ //Blynk app has something on V6

  Blynk.virtualWrite(VI_DISPLAY, pv_inputvoltage); //sending to Blynk

}

/************************************************
** Function name:           BLYNK_READ
** Descriptions:            send present temp_in to app
*************************************************/

BLYNK_READ(TI_DISPLAY){ //Blynk app has something on V7

  Blynk.virtualWrite(TI_DISPLAY, pv_tempin); //sending to Blynk

}

/************************************************
** Function name:           BLYNK_READ
** Descriptions:            send present temp_out to app
*************************************************/

BLYNK_READ(TO_DISPLAY){ //Blynk app has something on V8

  Blynk.virtualWrite(TO_DISPLAY, pv_tempout); //sending to Blynk

}

/************************************************
** Function name:           BLYNK_READ
** Descriptions:            send present charger status to app
*************************************************/

BLYNK_READ(S_DISPLAY){ //Blynk app has something on V8

  Blynk.virtualWrite(S_DISPLAY, charger_status); //sending to Blynk

}

/************************************************
** Function name:           BLYNK_WRITE
** Descriptions:            receive switch signal
*************************************************/

BLYNK_WRITE(SWITCH){ //Switch is writing to V1

  int on_off = param.asInt(); // assigning incoming value from pin V1 to a variable

  if(on_off == HIGH){

    charger_enable = HIGH;
    charger_status = "Laden ein";
    Blynk.virtualWrite(S_DISPLAY, charger_status);

  }else if(on_off == LOW){

    charger_enable = LOW;
    charger_status = "Laden aus";
    Blynk.virtualWrite(S_DISPLAY, charger_status);

  }

}

/************************************************
** Function name:           BLYNK_WRITE
** Descriptions:            receive voltage slider signal
*************************************************/

BLYNK_WRITE(V_SLIDER){ //Voltage slider is writing to V2

  float slider_voltage = param.asFloat(); // assigning incoming value from pin V1 to a variable

  slider_voltage = (int) (slider_voltage*100); //charger protocol offset

  if(slider_voltage > 5760){

    setVoltage(5760);
      //Serial.println(5760);

  }else{

    setVoltage(slider_voltage); //set desired voltage
    //Serial.println(slider_voltage);

  }

}

/************************************************
** Function name:           BLYNK_WRITE
** Descriptions:            receive current slider signal
*************************************************/

BLYNK_WRITE(C_SLIDER){ //Current slider is writing to V3

  float slider_current = param.asFloat(); // assigning incoming value from pin V1 to a variable

  slider_current = (int) (slider_current*10); //charger protocol offset
  setCurrent(slider_current); //set desired current

  //Serial.println(slider_current);

}

/************************************************
** Function name:           canWrite
** Descriptions:            write CAN message
*************************************************/

void canWrite(unsigned char message_data[8], long int ID){

  tx_frame.FIR.B.FF = CAN_frame_ext;
  tx_frame.MsgID = ID;
  tx_frame.FIR.B.DLC = 8;

  for(int i = 0; i < 8; i++){

    tx_frame.data.u8[i] = message_data[i];

  }

  ESP32Can.CANWriteFrame(&tx_frame);

  //Serial.println("Nachricht gesendet");

}

/************************************************
** Function name:           canRead
** Descriptions:            read CAN message
*************************************************/

void canRead(){

  if(xQueueReceive(CAN_cfg.rx_queue,&rx_frame, 3*portTICK_PERIOD_MS)==pdTRUE){

      //Serial.println("Nachricht empfangen");

      switch (rx_frame.MsgID) {

        case 0x05009888: //if CANID = 0500xxyy where xxyy the last 2 digits of the serial nr
        case 0x05014400:{ //this is the request from the Flatpack rectifier during walk-in (start-up) or normal operation when no log-in response has been received for a while (ID1 or ID2)

          canWrite(serialnr, login_ID);
          //Serial.println("Eingeloggt");

        }break;

        case 0x05014004:                          //01 ist hier die zugewiesene ID des Flatpacks im Header
        case 0x05014008:
        case 0x05014010:
        case 0x0501400C:{

          //Serial.println("Daten gelesen");
          pv_tempin = rx_frame.data.u8[0]; //Inlet temperature is byte 0 in the status message
          pv_tempout = rx_frame.data.u8[7]; //Outlet temperature is byte 7 in the status message
          pv_inputvoltage = 256*rx_frame.data.u8[6]+rx_frame.data.u8[5]; //Input voltage is byte 6 (highbyte) and byte 5 (lowbyte) in the status message
          pv_voltage = (0.01*(256*rx_frame.data.u8[4]+rx_frame.data.u8[3])); //Output voltage is byte 4 (highbyte) and 3 (lowbyte) in the status message
          pv_current = (0.1*(256*rx_frame.data.u8[2]+rx_frame.data.u8[1])); //Output current is byte 2 (highbyte) and 1 (lowbyte) in the status message

          /*Serial.println(pv_tempin);
          Serial.println(pv_tempout);
          Serial.println(pv_inputvoltage);
          Serial.println(pv_voltage);
          Serial.println(pv_current);*/

        }

      }

  }

}

/************************************************
** Function name:           writeTimer
** Descriptions:            Function of sendTimer
*************************************************/

void writeTimer() { //zyklisch vom Timer aufgerufene Funktion

 if(charger_enable == HIGH){

   PIDSetpoint1 = outputcurrent*0.1;
   PIDInput1 = pv_current;
   myPID1.Compute();
   word v_out = 4650 + PIDOutput1*(outputvoltage-4650)/255;

   unsigned char voltamp1[8] = {lowByte(outputcurrent), highByte(outputcurrent), lowByte(v_out), highByte(v_out), lowByte(v_out), highByte(v_out),lowByte(overvoltage), highByte(overvoltage)};

   canWrite(voltamp1, send_ID); //Schreibfunktion aufrufen

 }else if(charger_enable == LOW){

   unsigned char voltamp2[8] = {lowByte(0), highByte(0), lowByte(4400), highByte(4400), lowByte(4400), highByte(4400),lowByte(overvoltage), highByte(overvoltage)};

   canWrite(voltamp2, send_ID); //Schreibfunktion aufrufen

 }



}

/************************************************
** Function name:           readTimer
** Descriptions:            Function of receiveTimer
*************************************************/

void readTimer() { //zyklisch vom Timer aufgerufene Funktion

  canRead(); //Lesefunktion aufrufen

}

/************************************************
** Function name:           loginTimer
** Descriptions:            Function of receiveTimer
*************************************************/

void loginTimer() { //zyklisch vom Timer aufgerufene Funktion

  canWrite(serialnr, login_ID); //Login every 5 seconds

}

/************************************************
** Function name:           setup
** Descriptions:            Arduino setup
*************************************************/

void setup(){

  CAN_cfg.speed=CAN_SPEED_125KBPS; //125kbit ist der richtige Wert!
  CAN_cfg.tx_pin_id = GPIO_NUM_5;
  CAN_cfg.rx_pin_id = GPIO_NUM_4;
  CAN_cfg.rx_queue = xQueueCreate(1,sizeof(CAN_frame_t));

  ESP32Can.CANInit(); //start CAN Module

  delay(50);

  //Serial.begin(115200);

  timer.setInterval(951, writeTimer); //send every 951 ms
  timer.setInterval(500, readTimer); //read every 500 ms
  timer.setInterval(4750, loginTimer); //login every 4750 ms

  delay(50);

  PIDInput1 = 0; // Initial input of the first PID controller
  PIDSetpoint1 = outputcurrent*0.1; // Initial Setpoint of the first PID controller
  myPID1.SetMode(AUTOMATIC);

  charger_enable = HIGH;
  charger_status = "Laden ein";

  delay(50);

  canWrite(serialnr, login_ID); // send message to log in to the first flatpack and assign ID1 (DO NOT use an ID in the header, use 00, the third last number of the CANID determines ID of the flatpack (4=0, 8=1 C=2 etc) so 0x5004404 is ID0, 0x5004804 is ID1, 0x5004C04 is ID3 etc

  delay(50);

  Blynk.setDeviceName("ESP32 Bluetooth Test");

  Blynk.begin(auth);

}

/************************************************
** Function name:           loop
** Descriptions:            Arduino loop
*************************************************/

void loop(){

  Blynk.run();
  timer.run();

}
