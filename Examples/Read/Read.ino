#include "STM32_CAN.h"
//STM32_CAN Can1( CAN1, DEF );
//STM32_CAN Can1( CAN1, ALT );
STM32_CAN Can1( CAN1, ALT_2 );

static CAN_message_t CAN_RX_msg;

void setup() {
  Serial.begin(115200);
  Can1.begin();
  Can1.setBaudRate(500000);
}

void loop() {
  if (Can1.read(CAN_RX_msg) ) {
    Serial.print("Channel:");
    Serial.print(CAN_RX_msg.bus);
    Serial.print(" Standard ID:");
    Serial.print(CAN_RX_msg.id, HEX);

    Serial.print(" DLC: ");
    Serial.print(CAN_RX_msg.len);
    if (CAN_RX_msg.flags.remote == false) {
       Serial.print(" buf: ");
      for(int i=0; i<CAN_RX_msg.len; i++) {
        Serial.print("0x"); 
        Serial.print(CAN_RX_msg.buf[i], HEX); 
        if (i != (CAN_RX_msg.len-1))  Serial.print(" ");
      }
      Serial.println();
    } else {
       Serial.println(" Data: REMOTE REQUEST FRAME");
    }
  }
}