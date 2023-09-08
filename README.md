# CAN bus Library for Arduino STM32

This is universal CAN library for STM32 Arduino use. Originally this was created to be used with Speeduino EFI and other
CAN bus projects used in car environment. But has since grown into universal CAN bus library for Arduino STM32.
This library should support all STM32 MCUs that are also supported in stm32duino Arduino_Core_STM32 and supports 
up to 3x CAN buses. This library is based on several STM32 CAN example libraries linked below and it has been 
combined with few things from Teensy FlexCAN library to make it compatible with CAN coding projects made for Teensy.

Note! This will currently only work with CAN interface. Not with CANFD.


Links to repositories that have helped with this:

https://github.com/nopnop2002/Arduino-STM32-CAN <br>
https://github.com/J-f-Jensen/libraries/tree/master/STM32_CAN <br>
https://github.com/jiauka/STM32F1_CAN

STM32 core: https://github.com/stm32duino/Arduino_Core_STM32

## How to use.
To use this library, CAN module needs to be enabled in HAL drivers. If PIO is used, it's enough
to add -DHAL_CAN_MODULE_ENABLED as build flag. With Arduino IDE it's easiest to create hal_conf_extra.h -file
to same folder with sketch and haven #define HAL_CAN_MODULE_ENABLED there. See examples for this.

### Setup
The sketch needs include in the header to use the library.
```
#include "STM32_CAN.h"
```
Use constructor to set which CAN interface and pins to use.
```
STM32_CAN Can1( CAN1, DEF );
```
Depending on the STM32 model, there is CAN1, CAN2 and CAN3 available.
The pins are:
 - CAN1
   - DEF: PA11/12
   - ALT: PB8/9
   - ALT_2: PD0/1
 - CAN2
   - DEF: PB12/13
   - ALT: PB5/6
 - CAN3
   - DEF: PA8/15
   - ALT: PB3/4

In constructor you can optionally set TX and RX buffer sizes. Default is 16 messages.
```
STM32_CAN Can1 (CAN1, ALT_2, RX_SIZE_256, TX_SIZE_256);
```
There is structure to contain CAN packets and info about those.
```
typedef struct CAN_message_t {
  uint32_t id = 0;         // can identifier
  uint16_t timestamp = 0;  // time when message arrived
  uint8_t idhit = 0;       // filter that id came from
  struct {
    bool extended = 0;     // identifier is extended (29-bit)
    bool remote = 0;       // remote transmission request packet type
    bool overrun = 0;      // message overrun
    bool reserved = 0;
  } flags;
  uint8_t len = 8;         // length of data
  uint8_t buf[8] = { 0 };  // data
  int8_t mb = 0;           // used to identify mailbox reception
  uint8_t bus = 1;         // used to identify where the message came (CAN1, CAN2 or CAN3)
  bool seq = 0;            // sequential frames
} CAN_message_t;
```
In sketch.
```
static CAN_message_t CAN_msg;
```
Before using library commands, begin must be called.
```
Can1.begin();
```
Optionally automatic retransmission can be enabled with begin.
```
Can1.begin(true);
```
Set baud rate by using.
```
Can1.setBaudRate(500000);
```
500 kbit/s in this case.

By default bank 0 will receive CAN messages with all IDs. But using filters, we can set the CAN to receive
only specific message IDs. Ie. if we want to receive only message IDs 0x153 and 0x613, so we use bank 0 and 1.
```
Can1.setFilter( 0, 0x153, 0x1FFFFFFF );
Can1.setFilter( 1, 0x613, 0x1FFFFFFF );
```
First number is the bank number. Second is message ID. And last one is mask.

### Main loop
To read CAN messages from buffer:
```
Can1.read(CAN_msg)
```
This will return true if there is messages available on receive buffer.

To write messages to send queue.
```
Can1.write(CAN_msg);
```
This will return true if there room in the send queue.
