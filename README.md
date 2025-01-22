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
to add `-DHAL_CAN_MODULE_ENABLED` as build flag. With Arduino IDE it's easiest to create `hal_conf_extra.h` file
to same folder with sketch and haven `#define HAL_CAN_MODULE_ENABLED` there. See examples for this.

### Setup
The sketch needs include in the header to use the library.
```Cpp
#include "STM32_CAN.h"
```
Use constructor to set which CAN interface and pins to use.
```Cpp
STM32_CAN Can1( CAN1 ); //by peripheral. Uses first pin defined in arduino PeripheralPins maps
STM32_CAN Can1( PA11 /* rx pin */, PA12 /* tx pin */); //by arduino digital pin number. Finds matching peripheral automatically
STM32_CAN Can1( PA_11, PA_12 ); //by PinName. Finds matching peripheral automatically
```
**Note**: Digital pin and PinName may not be mixed or one will be implicitly converted and possibly causing unintended behavior. PinNames are defined with an underscore between port and pin. Digital pin numbers are aliased with port and pinnumber without an underscore.
Can convert between the two with `digitalPinToPinName()` and `pinNametoDigitalPin()`.

Tx pin may be defined as `PNUM_NOT_DEFINED` (digital pin number) or `NC` (PinName). Then only Rx is setup, allowing a listen only mode.

The original method by choosing from 3 sets of fixed pin combinations can still be used as well for compatibility.
```Cpp
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
```Cpp
STM32_CAN Can1 (PA_11, PA_12, RX_SIZE_256, TX_SIZE_256);
```

#### Additional settings
Following settings may be changed before starting the driver with `begin()`.
```Cpp
setIRQPriority(uint32_t preemptPriority, uint32_t subPriority); // default: lowest prio, 0
setAutoRetransmission(bool enabled);  //default: true
setRxFIFOLock(bool fifo0locked);      //default: false
setTxBufferMode(TX_BUFFER_MODE mode); //default: FIFO
setTimestampCounter(bool enabled);    //default: false
setMode(MODE mode);                   //default: NORMAL
setAutoBusOffRecovery(bool enabled);  //default: false
```
**Note**: `setTimestampCounter()` should always be set `false`. Feature is marked as non functioning in Erratas.

**Note**: begin() will overwrite `setAutoRetransmission()` setting (`false` by default).

#### Start / Stop bus

Before using library commands, begin must be called.
```Cpp
Can1.begin();
```
Optionally automatic retransmission can be enabled with begin.
```Cpp
Can1.begin(true);
```
Set baud rate by using.
```Cpp
Can1.setBaudRate(500000);
```
500 kbit/s in this case.

`setBaudRate` may be called before or after begin. Bus start once both are called.

It may also be called while running to change baudrate.

Call
```Cpp
Can1.end();
```
to stop the bus and release all resources. Will be implicitly called by destructor when freeing object.
Only one STM32_CAN instance can control a CAN Peripheral at one time. It reserves the peripheral from begin() to end().

#### Filters
Filters may only be set after bus is started (`begin()` & `setBaudRate()` was called).

By default bank 0 will receive CAN messages with all IDs. But using filters, we can set the CAN to receive
only specific message IDs. The CAN peripheral has many different methods of defining filters for messages.
These functions help setting up filters:
```Cpp
bool setFilterSingleMask(uint8_t bank_num, uint32_t id,  uint32_t mask,  IDE std_ext);
bool setFilterDualID    (uint8_t bank_num, uint32_t id1, uint32_t id2,   IDE std_ext1, IDE std_ext2);
bool setFilterDualMask  (uint8_t bank_num, uint32_t id1, uint32_t mask1, IDE std_ext1, uint32_t id2, uint32_t mask2, IDE std_ext2);
bool setFilterQuadID    (uint8_t bank_num, uint32_t id1, IDE std_ext1, uint32_t id2, IDE std_ext2, uint32_t id3, IDE std_ext3, uint32_t id4, IDE std_ext4);
```
The simplest one is `setFilterSingleMask`. The others allow setting multiple filters into a single filter bank. When using the functions `setFilterDualMask` and `setFilterQuadID` for extended ids, the 15 LSBs of the ID are always masked out.

Example: we want to receive only messages with standard ID 0x153 and extended ID 0x613, so we use bank 0 and 1.
```Cpp
Can1.setFilterSingleMask( 0, 0x153, 0x7FF, STD);
Can1.setFilterSingleMask( 1, 0x613, 0x1FFFFFFF, EXT);
```
First number is the bank number. Second is message ID. 3rd ID mask. And last one defines ID type: `AUTO`, `STD` or `EXT`.


**Note**: optional args of filter settings allow storing messages into the non-default RxFIFO. `read()` currently only reads from the default FIFO, so messages will not be accessible if sent to other FIFO.

**Note**: the following function allowed setting filters in the past. This function was bugged and did only work as long as optional args filter_mode and filter_scale where not changed.
Its behavior was kept in the broken state for compatibility. It has a 4th arg for ID type, defaults to `AUTO`.
```Cpp
bool setFilter(uint8_t bank_num, uint32_t filter_id, uint32_t mask)
```

To disable a filter bank call (`true` to re-enable it)
```Cpp
Can1.setFilter(bank_num, false);
```


The amount of available filter banks can be queried with:
```Cpp
Can1.getFilterBankCount()
```
**Note**: Single CAN devices have 14, dual CAN devices 28. Driver does split the 28 into 14 for each CAN instance.


### Main loop
The library defines this structure to describe a CAN message.
```Cpp
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

Define variable to store a message
```Cpp
CAN_message_t CAN_msg;
```

To read CAN messages from buffer:
```Cpp
Can1.read(CAN_msg);
```
This will return true if there is messages available on receive buffer.

To write messages to send queue.
```Cpp
Can1.write(CAN_msg);
```
This will return true if there room in the send queue.
