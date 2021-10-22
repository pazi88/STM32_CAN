#include <src/STM32_CAN/STM32_CAN.h>
void setup() {
  Serial.begin(115200);
  delay(2000);

  

}

void loop() {
  //STM32_CAN Can0 (_CAN1,DEF);
  uint8_t recv_ch = 1;
  static CAN_message_t CAN_RX_msg;

  Can1.begin(true);
  Can1.setBaudRate(500000);
  Can1.enableFIFO();
  uint8_t SecL = 0;
  while(1){
    delay(1000);
    while (Can1.read(CAN_RX_msg) ) {
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
    }
  }
}
