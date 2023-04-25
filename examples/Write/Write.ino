/*
This is simple example to send random data to CAN bus in 20Hz rate, using delay (not recommended in real implementations).
*/

#include "STM32_CAN.h"
STM32_CAN Can( CAN1, DEF );  //Use PA11/12 pins for CAN1.
//STM32_CAN Can( CAN1, ALT );  //Use PB8/9 pins for CAN1.
//STM32_CAN Can( CAN1, ALT_2 );  //Use PD0/1 pins for CAN1.
//STM32_CAN Can( CAN2, DEF );  //Use PB12/13 pins for CAN2.
//STM32_CAN Can( CAN2, ALT );  //Use PB5/6 pins for CAN2
//STM32_CAN Can( CAN3, DEF );  //Use PA8/15 pins for CAN3.
//STM32_CAN Can( CAN3, ALT );  //Use PB3/4 pins for CAN3

static CAN_message_t CAN_TX_msg;

void setup() {
  Serial.begin(115200);
  Can.begin();
  //Can.setBaudRate(250000);  //250KBPS
  Can.setBaudRate(500000);  //500KBPS
  //Can.setBaudRate(1000000);  //1000KBPS
}

void loop() {
  uint8_t Counter = 0;
  while(1){
    if (Counter > 255){ Counter = 0;}
    delay(50);
    CAN_TX_msg.id = (0x1A5);
    CAN_TX_msg.len = 8;
    CAN_TX_msg.buf[0] =  0x03;
    CAN_TX_msg.buf[1] =  0x41;
    CAN_TX_msg.buf[2] =  0x11;
    CAN_TX_msg.buf[3] =  Counter;
    CAN_TX_msg.buf[4] =  0x00;
    CAN_TX_msg.buf[5] =  0x00;
    CAN_TX_msg.buf[6] =  0x00;
    CAN_TX_msg.buf[7] =  0x00;
  
    Can.write(CAN_TX_msg);

    CAN_TX_msg.id = (0x1AC32CF5);
    CAN_TX_msg.flags.extended = 1;  // To enable extended ID.
    CAN_TX_msg.len = 8;
    CAN_TX_msg.buf[0] =  0x03;
    CAN_TX_msg.buf[1] =  0x41;
    CAN_TX_msg.buf[3] =  0x21;
    CAN_TX_msg.buf[4] =  0x00;
    CAN_TX_msg.buf[5] =  Counter;
    CAN_TX_msg.buf[6] =  0x00;
    CAN_TX_msg.buf[7] =  0xFF;

    Can.write(CAN_TX_msg);

    CAN_TX_msg.id = (0xA63);
    CAN_TX_msg.len = 8;
    CAN_TX_msg.buf[0] =  0x63;
    CAN_TX_msg.buf[1] =  0x49;
    CAN_TX_msg.buf[2] =  0x11;
    CAN_TX_msg.buf[3] =  0x22;
    CAN_TX_msg.buf[4] =  0x00;
    CAN_TX_msg.buf[5] =  0x00;
    CAN_TX_msg.buf[6] =  Counter;
    CAN_TX_msg.buf[7] =  0x00;

    Can.write(CAN_TX_msg);

    CAN_TX_msg.id = (0x23);
    CAN_TX_msg.flags.extended = 0;  // Back to standard ID.
    CAN_TX_msg.len = 8;
    CAN_TX_msg.buf[0] =  0x03;
    CAN_TX_msg.buf[1] =  0x41;
    CAN_TX_msg.buf[2] =  0x11;
    CAN_TX_msg.buf[3] =  0x33;
    CAN_TX_msg.buf[4] =  0x00;
    CAN_TX_msg.buf[5] =  0x00;
    CAN_TX_msg.buf[6] =  0x00;
    CAN_TX_msg.buf[7] =  Counter;

    Can.write(CAN_TX_msg);

    CAN_TX_msg.id = (0x55);
    CAN_TX_msg.len = 8;
    CAN_TX_msg.buf[0] =  0x03;
    CAN_TX_msg.buf[1] =  0x44;
    CAN_TX_msg.buf[2] =  0x31;
    CAN_TX_msg.buf[3] =  0x53;
    CAN_TX_msg.buf[4] =  0x00;
    CAN_TX_msg.buf[5] =  Counter;
    CAN_TX_msg.buf[6] =  0x00;
    CAN_TX_msg.buf[7] =  0x00;

    Can.write(CAN_TX_msg);
    Serial.print("Sent: ");
    Serial.println(Counter, HEX);
    Counter++;
  }
}
