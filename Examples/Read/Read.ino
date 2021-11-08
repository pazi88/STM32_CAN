#include "STM32_CAN.h"
//STM32_CAN Can1( CAN1, DEF );
//STM32_CAN Can1( CAN1, ALT );
STM32_CAN Can0( CAN1, ALT_2 );

void setup() {
  Serial.begin(115200);
  delay(3000);
  

}

void loop() {
  uint8_t recv_ch = 1;
  static CAN_message_t CAN_RX_msg;
  static CAN_message_t CAN_TX_msg;

    CAN_TX_msg.id = (0x153);
    CAN_TX_msg.len = 8;
    CAN_TX_msg.buf[0] =  0x03;                    // sending 3 bytes
    CAN_TX_msg.buf[1] =  0x00;                    // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
    CAN_TX_msg.buf[2] =  0xB0;                    // pid code
    CAN_TX_msg.buf[3] =  0x00;                    // A
    CAN_TX_msg.buf[4] =  0x00;                    // B
    CAN_TX_msg.buf[5] =  0x00; 
    CAN_TX_msg.buf[6] =  0x00; 
    CAN_TX_msg.buf[7] =  0x00;

  Can0.begin();
  Can0.setBaudRate(500000);
  Can0.enableFIFO();
  uint8_t SecL = 0;
  while(1){
    if (Can0.read(CAN_RX_msg) ) {
      Serial.print("Channel:");
      Serial.print(recv_ch);
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
      if (CAN_RX_msg.id == 0x316){
      Can0.write(CAN_TX_msg);
      }
    }
  }
}
