#include <src/STM32_CAN/STM32_CAN.h>

void setup() {
  Serial.begin(115200);
  delay(1000);

}

void loop() {
  //STM32_CAN Can0 (_CAN1,DEF);
  uint8_t recv_ch = 1;
  static CAN_message_t CAN_TX_msg;

  Can1.begin( false);
  Can1.setBaudRate(500000);
  Can1.enableFIFO();
  uint8_t SecL = 0;
  while(1){
    if (SecL > 255){ SecL = 0;}
    delay(50);
    CAN_TX_msg.id = (0x1A5);
    CAN_TX_msg.len = 8;
    CAN_TX_msg.buf[0] =  0x03;                    // sending 3 bytes
    CAN_TX_msg.buf[1] =  0x41;                    // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
    CAN_TX_msg.buf[2] =  0x11;                    // pid code
    CAN_TX_msg.buf[3] =  SecL;                    // A
    CAN_TX_msg.buf[4] =  0x00;                    // B
    CAN_TX_msg.buf[5] =  0x00; 
    CAN_TX_msg.buf[6] =  0x00; 
    CAN_TX_msg.buf[7] =  0x00;
  
    Can1.write(CAN_TX_msg);
    Can1.write(CAN_TX_msg);
    Can1.write(CAN_TX_msg);

    CAN_TX_msg.id = (0x7E8);
    CAN_TX_msg.len = 8;
    CAN_TX_msg.buf[0] =  0x03;                    // sending 3 bytes
    CAN_TX_msg.buf[1] =  0x41;                    // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
    CAN_TX_msg.buf[2] =  0x11;                    // pid code
    CAN_TX_msg.buf[3] =  0x21;                    // A
    CAN_TX_msg.buf[4] =  0x00;                    // B
    CAN_TX_msg.buf[5] =  SecL; 
    CAN_TX_msg.buf[6] =  0x00; 
    CAN_TX_msg.buf[7] =  0x00;
  
    Can1.write(CAN_TX_msg);
    Can1.write(CAN_TX_msg);
    Can1.write(CAN_TX_msg);

    CAN_TX_msg.id = (0xA63);
    CAN_TX_msg.len = 8;
    CAN_TX_msg.buf[0] =  0x03;                    // sending 3 bytes
    CAN_TX_msg.buf[1] =  0x41;                    // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
    CAN_TX_msg.buf[2] =  0x11;                    // pid code
    CAN_TX_msg.buf[3] =  0x33;                    // A
    CAN_TX_msg.buf[4] =  0x00;                    // B
    CAN_TX_msg.buf[5] =  0x00; 
    CAN_TX_msg.buf[6] =  0x00; 
    CAN_TX_msg.buf[7] =  SecL;

  
    Can1.write(CAN_TX_msg);
    Can1.write(CAN_TX_msg);
    Can1.write(CAN_TX_msg);

    CAN_TX_msg.id = (0x23);
    CAN_TX_msg.len = 8;
    CAN_TX_msg.buf[0] =  0x03;                    // sending 3 bytes
    CAN_TX_msg.buf[1] =  0x41;                    // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
    CAN_TX_msg.buf[2] =  0x11;                    // pid code
    CAN_TX_msg.buf[3] =  0x33;                    // A
    CAN_TX_msg.buf[4] =  0x00;                    // B
    CAN_TX_msg.buf[5] =  0x00; 
    CAN_TX_msg.buf[6] =  0x00; 
    CAN_TX_msg.buf[7] =  SecL;

  
    Can1.write(CAN_TX_msg);
    Can1.write(CAN_TX_msg);
    Can1.write(CAN_TX_msg);

    CAN_TX_msg.id = (0x55);
    CAN_TX_msg.len = 8;
    CAN_TX_msg.buf[0] =  0x03;                    // sending 3 bytes
    CAN_TX_msg.buf[1] =  0x41;                    // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
    CAN_TX_msg.buf[2] =  0x11;                    // pid code
    CAN_TX_msg.buf[3] =  0x33;                    // A
    CAN_TX_msg.buf[4] =  0x00;                    // B
    CAN_TX_msg.buf[5] =  0x00; 
    CAN_TX_msg.buf[6] =  0x00; 
    CAN_TX_msg.buf[7] =  SecL;

  
    Can1.write(CAN_TX_msg);
    Can1.write(CAN_TX_msg);
    Can1.write(CAN_TX_msg);
    Serial.print("Sent: ");
    Serial.println(SecL, HEX);
    SecL++;
  }
}
