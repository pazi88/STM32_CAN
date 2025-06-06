/*
This is universal CAN library for STM32 that was made to be used with Speeduino EFI and in my other projects.
It should support all STM32 MCUs that are also supported in stm32duino Arduino_Core_STM32 and supports up to 3x CAN buses.
The library is created because at least currently (year 2021) there is no official CAN library in the STM32 core.
This library is based on several STM32 CAN example libraries linked below and it has been combined with few
things from Teensy FlexCAN library to make it compatible with CAN coding projects made for Teensy.
Links to repositories that have helped with this:
https://github.com/nopnop2002/Arduino-STM32-CAN
https://github.com/J-f-Jensen/libraries/tree/master/STM32_CAN
https://github.com/jiauka/STM32F1_CAN

STM32 core: https://github.com/stm32duino/Arduino_Core_STM32

IMPORTANT NOTE! To use this library, CAN module needs to be enabled in HAL drivers. If PIO is used, it's enough
to add -DHAL_CAN_MODULE_ENABLED as build flag. With Arduino IDE it's easiest to create hal_conf_extra.h -file
to same folder with sketch and haven #define HAL_CAN_MODULE_ENABLED there. See examples for this.
*/

#ifndef STM32_CAN_H
#define STM32_CAN_H

#include <Arduino.h>

/** Handling special cases for IRQ Handlers */
#if defined(STM32F0xx)
#if defined(STM32F042x6) || defined(STM32F072xB) || defined(STM32F091xC) || defined(STM32F098xx)

  /**
   * NOTE: STM32F0 share IRQ Handler with HDMI CEC
   * and there is only a single IRQ Handler not 4
   * CEC_CAN_IRQn | CEC_CAN_IRQHandler
   * 
   * define all in one alias for IRQn and Handler
   */
  #define CAN1_IRQn_AIO       CEC_CAN_IRQn
  #define CAN1_IRQHandler_AIO CEC_CAN_IRQHandler
  /**
   * NOTE: CAN IRQ is shared with CEC
   * To use CEC with CAN declare:
   * CEC_HandleTypeDef * phcec;
   * and point to your CEC handle.
   * Internal IRQ Handler will call CEC Handler as well.
   */
  #define STM32_CAN1_SHARED_WITH_CEC

#endif
#endif

#if defined(STM32F1xx)
#if defined(STM32F103x6) || defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG)
/**
 * NOTE: STM32F103xx uses shared IRQ Handler with USB
 * USB_HP_CAN1_TX_IRQn  | USB_HP_CAN1_TX_IRQHandler
 * USB_LP_CAN1_RX0_IRQn | USB_LP_CAN1_RX0_IRQHandler
 * 
 * the CMSIS files define already aliases for the CAN *_IRQHandler and *_IRQn
 * conforming with standard naming convention:
 * CAN1_TX_IRQn  | CAN1_TX_IRQHandler
 * CAN1_RX0_IRQn | CAN1_RX0_IRQHandler
 * 
 * when USB is enabled the USBDevice driver also implements these making a concurrent use impossible.
 * 
 * Following are unaffected:
 * CAN1_RX1_IRQn | CAN1_RX1_IRQHandler
 * CAN1_SCE_IRQn | CAN1_SCE_IRQHandler
 */

#ifdef USBCON
#define STM32_CAN1_TX_RX0_BLOCKED_BY_USB
#endif

#endif
#endif

#if defined(STM32F3xx)
#if defined(STM32F302x8) || defined(STM32F302xC) || defined(STM32F302xE)\
 || defined(STM32F303xC) || defined(STM32F303xE)

  /**
   * NOTE: STM32F3 with USB share IRQ Handler with it
   * USB_HP_CAN_TX_IRQn  | USB_HP_CAN_TX_IRQHandler
   * USB_LP_CAN_RX0_IRQn | USB_LP_CAN_RX0_IRQHandler
   * 
   * the CMSIS files define already aliases for the CAN *_IRQHandler and *_IRQn
   * missing peripheral index:
   * CAN_TX_IRQn  | CAN_TX_IRQHandler
   * CAN_RX0_IRQn | CAN_RX0_IRQHandler
   * 
   * when USB is enabled the USBDevice driver also implements these making a concurrent use impossible.
   * 
   * Following are unaffected:
   * CAN_RX1_IRQn | CAN_RX1_IRQHandler
   * CAN_SCE_IRQn | CAN_SCE_IRQHandler
   * 
   * define more aliases with peripheral index
   */

  #define CAN1_TX_IRQn      USB_HP_CAN_TX_IRQn
  #define CAN1_RX0_IRQn     USB_LP_CAN_RX0_IRQn
  #define CAN1_RX1_IRQn     CAN_RX1_IRQn
  #define CAN1_SCE_IRQn     CAN_SCE_IRQn

  #define CAN1_TX_IRQHandler  USB_HP_CAN_TX_IRQHandler
  #define CAN1_RX0_IRQHandler USB_LP_CAN_RX0_IRQHandler
  #define CAN1_RX1_IRQHandler CAN_RX1_IRQHandler
  #define CAN1_SCE_IRQHandler CAN_SCE_IRQHandler

  /** NOTE: USE_USB_INTERRUPT_REMAPPED may be used to use 
   * different USB IRQs and not block the CAN IRQ handlers */
  #if defined(USBCON) && !defined(USE_USB_INTERRUPT_REMAPPED)
  #define STM32_CAN1_TX_RX0_BLOCKED_BY_USB
  #endif

#elif defined(STM32F303x8) || defined(STM32F328xx) || defined(STM32F334x8)\
 || defined(STM32F358xx) || defined(STM32F373xC)\
 || defined(STM32F378xx) || defined(STM32F398xx)

  /**
   * NOTE: STM32F3 without USB define symbols without peripheral index
   * define more aliases with peripheral index
   */

  #define CAN1_TX_IRQn      CAN_TX_IRQn
  #define CAN1_RX0_IRQn     CAN_RX0_IRQn
  #define CAN1_RX1_IRQn     CAN_RX1_IRQn
  #define CAN1_SCE_IRQn     CAN_SCE_IRQn

  #define CAN1_TX_IRQHandler  CAN_TX_IRQHandler
  #define CAN1_RX0_IRQHandler CAN_RX0_IRQHandler
  #define CAN1_RX1_IRQHandler CAN_RX1_IRQHandler
  #define CAN1_SCE_IRQHandler CAN_SCE_IRQHandler
#endif
#endif

#if defined(STM32_CAN1_TX_RX0_BLOCKED_BY_USB) && !defined(STM32_CAN_USB_WORKAROUND_POLLING)
#error "USB and CAN interrupts are shared on the F1/F3 platform, driver is not compatible with USBDevice of Arduino core. Can define STM32_CAN_USB_WORKAROUND_POLLING to disable error msg and call STM32_CAN_Poll_IRQ_Handler to poll for Tx IRQ events. Only use FIFO 1."
#elif defined(USBCON) && defined(STM32_CAN_USB_WORKAROUND_POLLING)
#warning "CAN IRQ Handler is used by USBDevice driver, call STM32_CAN_Poll_IRQ_Handler() frequently to handle CAN events."
extern "C" void STM32_CAN_Poll_IRQ_Handler(void);
#define CAN_FILTER_DEFAULT_FIFO   CAN_FILTER_FIFO1
#define CAN_FILTER_DEFAULT_ACTION STORE_FIFO1
#else
#define CAN_FILTER_DEFAULT_FIFO   CAN_FILTER_FIFO0
#define CAN_FILTER_DEFAULT_ACTION STORE_FIFO0
#endif


// This struct is directly copied from Teensy FlexCAN library to retain compatibility with it. Not all are in use with STM32.
// Source: https://github.com/tonton81/FlexCAN_T4/

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

typedef struct {
  uint32_t baudrate;
  uint16_t prescaler;
  uint8_t time_quanta;
  uint8_t timeseg1;
  uint8_t timeseg2;
} Baudrate_entry_t;

typedef enum CAN_PINS {DEF, ALT, ALT_2,} CAN_PINS;

typedef enum RXQUEUE_TABLE {
  RX_SIZE_2 = (uint16_t)2,
  RX_SIZE_4 = (uint16_t)4,
  RX_SIZE_8 = (uint16_t)8,
  RX_SIZE_16 = (uint16_t)16,
  RX_SIZE_32 = (uint16_t)32,
  RX_SIZE_64 = (uint16_t)64,
  RX_SIZE_128 = (uint16_t)128,
  RX_SIZE_256 = (uint16_t)256,
  RX_SIZE_512 = (uint16_t)512,
  RX_SIZE_1024 = (uint16_t)1024
} RXQUEUE_TABLE;

typedef enum TXQUEUE_TABLE {
  TX_SIZE_2 = (uint16_t)2,
  TX_SIZE_4 = (uint16_t)4,
  TX_SIZE_8 = (uint16_t)8,
  TX_SIZE_16 = (uint16_t)16,
  TX_SIZE_32 = (uint16_t)32,
  TX_SIZE_64 = (uint16_t)64,
  TX_SIZE_128 = (uint16_t)128,
  TX_SIZE_256 = (uint16_t)256,
  TX_SIZE_512 = (uint16_t)512,
  TX_SIZE_1024 = (uint16_t)1024
} TXQUEUE_TABLE;

/* Teensy FlexCAN uses Mailboxes for different RX filters, but in STM32 there is Filter Banks. These work practically same way,
so the Filter Banks are named as mailboxes in "setMBFilter" -functions, to retain compatibility with Teensy FlexCAN library.
*/
typedef enum CAN_BANK {
  MB0 = 0,
  MB1 = 1,
  MB2 = 2,
  MB3 = 3,
  MB4 = 4,
  MB5 = 5,
  MB6 = 6,
  MB7 = 7,
  MB8 = 8,
  MB9 = 9,
  MB10 = 10,
  MB11 = 11,
  MB12 = 12,
  MB13 = 13,
  MB14 = 14,
  MB15 = 15,
  MB16 = 16,
  MB17 = 17,
  MB18 = 18,
  MB19 = 19,
  MB20 = 20,
  MB21 = 21,
  MB22 = 22,
  MB23 = 23,
  MB24 = 24,
  MB25 = 25,
  MB26 = 26,
  MB27 = 27
} CAN_BANK;

typedef enum CAN_FLTEN {
  ACCEPT_ALL = 0,
  REJECT_ALL = 1
} CAN_FLTEN;

typedef enum IDE {
  STD = 0,
  EXT = 1,
  AUTO = 2
} IDE;

typedef struct {
  void * __this;
  CAN_HandleTypeDef handle;
  uint32_t bus;
} stm32_can_t;

class STM32_CAN {

  public:
    enum MODE {
      NORMAL               = CAN_MODE_NORMAL,
      SILENT               = CAN_MODE_SILENT,
      SILENT_LOOPBACK      = CAN_MODE_SILENT_LOOPBACK,
      LOOPBACK             = CAN_MODE_LOOPBACK
    };

    enum FILTER_ACTION {
      STORE_FIFO0,
      STORE_FIFO1,
    };

    enum TX_BUFFER_MODE {
      FIFO  = ENABLE, /** Sequential transfers order */
      QUEUE = DISABLE /** Sequence based on msg ID priorities. Only effects hardware queue. */
    };


    // Default buffer sizes are set to 16. But this can be changed by using constructor in main code.
    STM32_CAN(uint32_t rx, uint32_t tx = PNUM_NOT_DEFINED, RXQUEUE_TABLE rxSize = RX_SIZE_16, TXQUEUE_TABLE txSize = TX_SIZE_16);
    STM32_CAN(PinName rx, PinName tx = NC, RXQUEUE_TABLE rxSize = RX_SIZE_16, TXQUEUE_TABLE txSize = TX_SIZE_16);
    STM32_CAN(CAN_TypeDef* canPort, RXQUEUE_TABLE rxSize = RX_SIZE_16, TXQUEUE_TABLE txSize = TX_SIZE_16);
    //legacy for compatibility
    STM32_CAN(CAN_TypeDef* canPort, CAN_PINS pins, RXQUEUE_TABLE rxSize = RX_SIZE_16, TXQUEUE_TABLE txSize = TX_SIZE_16);
    ~STM32_CAN();
/**-------------------------------------------------------------
 *     setup functions
 *     no effect after begin()
 * -------------------------------------------------------------
 */
    void setIRQPriority(uint32_t preemptPriority, uint32_t subPriority);

    /** send message again on arbitration failure */
    void setAutoRetransmission(bool enabled);

    /** If locked incoming msg is dropped when fifo is full,
     *  when unlocked last msg in fifo is overwritten
     *  2nd arg has no effect, setting effects both fifos */
    void setRxFIFOLock(bool fifo0locked, bool fifo1locked = true);
    void setTxBufferMode(TX_BUFFER_MODE mode);
    void setTimestampCounter(bool enabled);

    void setMode(MODE mode);
    void enableLoopBack(bool yes = 1);
    void enableSilentMode(bool yes = 1);
    void enableSilentLoopBack(bool yes = 1);

    void setAutoBusOffRecovery(bool enabled);

/**-------------------------------------------------------------
 *     lifecycle functions
 *     setBaudRate may be called before or after begin
 * -------------------------------------------------------------
 */
    // Begin. By default the automatic retransmission is enabled. If it causes problems, use begin(false) to disable it.
    void begin(bool retransmission = false);
    void end(void);

    void setBaudRate(uint32_t baud);

/**-------------------------------------------------------------
 *     post begin(), setup filters, data transfer
 * -------------------------------------------------------------
 */
    bool write(CAN_message_t &CAN_tx_msg, bool sendMB = false);
    bool read(CAN_message_t &CAN_rx_msg);

    /** returns number of available filter banks. If hasSharedFilterBanks() is false counts may differ by id type. */
    uint8_t getFilterBankCount(IDE std_ext = STD);
    /** returns if filter count and index are shared (true) or dedicated per id type (false) */
    bool hasSharedFilterBanks() {
      return true;
    }

    /** 
     * Manually set STM32 filter bank parameters
     * These return true on success
     */
    /** set filter state and action, keeps filter rules intact */
    bool setFilter(uint8_t bank_num, bool enabled, FILTER_ACTION action = CAN_FILTER_DEFAULT_ACTION);
    bool setFilterSingleMask(uint8_t bank_num, uint32_t id, uint32_t mask, IDE std_ext, FILTER_ACTION action = CAN_FILTER_DEFAULT_ACTION, bool enabled = true);
    bool setFilterDualID(uint8_t bank_num, uint32_t id1, uint32_t id2, IDE std_ext1, IDE std_ext2, FILTER_ACTION action = CAN_FILTER_DEFAULT_ACTION, bool enabled = true);
    bool setFilterDualMask(uint8_t bank_num, uint32_t id1, uint32_t mask1, IDE std_ext1, uint32_t id2, uint32_t mask2, IDE std_ext2, FILTER_ACTION action = CAN_FILTER_DEFAULT_ACTION, bool enabled = true);
    bool setFilterQuadID(uint8_t bank_num, uint32_t id1, IDE std_ext1, uint32_t id2, IDE std_ext2, uint32_t id3, IDE std_ext3, uint32_t id4, IDE std_ext4, FILTER_ACTION action = CAN_FILTER_DEFAULT_ACTION, bool enabled = true);
    bool setFilterRaw(uint8_t bank_num, uint32_t id, uint32_t mask, uint32_t filter_mode, uint32_t filter_scale, FILTER_ACTION action = CAN_FILTER_DEFAULT_ACTION, bool enabled = true);
    /** Legacy, broken! Only works correctly for 32 bit mask mode 
     * Returns true on Error, false on Success (like Teensy functions, opposite of STM32 function)
    */
    bool setFilter(uint8_t bank_num, uint32_t filter_id, uint32_t mask, IDE = AUTO, uint32_t filter_mode = CAN_FILTERMODE_IDMASK, uint32_t filter_scale = CAN_FILTERSCALE_32BIT, uint32_t fifo = CAN_FILTER_DEFAULT_FIFO);

/**-------------------------------------------------------------
 *     Teensy FlexCAN compatibility functions
 * -------------------------------------------------------------
 * These return false on success
 */
    bool setMBFilterProcessing(CAN_BANK bank_num, uint32_t filter_id, uint32_t mask, IDE = AUTO);
    void setMBFilter(CAN_FLTEN input); /* enable/disable traffic for all MBs (for individual masking) */
    void setMBFilter(CAN_BANK bank_num, CAN_FLTEN input); /* set specific MB to accept/deny traffic */
    bool setMBFilter(CAN_BANK bank_num, uint32_t id1, IDE = AUTO); /* input 1 ID to be filtered */
    bool setMBFilter(CAN_BANK bank_num, uint32_t id1, uint32_t id2, IDE = AUTO); /* input 2 ID's to be filtered */

    void enableFIFO(bool status = 1);
    void enableMBInterrupts();
    void disableMBInterrupts();

    // These are public because these are also used from interrupts.
    typedef struct RingbufferTypeDef {
      volatile uint16_t head;
      volatile uint16_t tail;
      uint16_t size;
      volatile CAN_message_t *buffer;
    } RingbufferTypeDef;

    RingbufferTypeDef rxRing;
    RingbufferTypeDef txRing;

    bool addToRingBuffer(RingbufferTypeDef &ring, const CAN_message_t &msg);
    bool removeFromRingBuffer(RingbufferTypeDef &ring, CAN_message_t &msg);

  protected:
    uint16_t sizeRxBuffer;
    uint16_t sizeTxBuffer;

  private:
    void      init(void);
    CAN_TypeDef * getPeripheral(void);
    bool      allocatePeripheral(CAN_TypeDef *instance);
    bool      freePeripheral(void);
    bool      hasPeripheral(void);
    void      start(void);
    void      stop(void);
    void      initializeFilters();
    bool      isInitialized() { return rx_buffer != 0; }
    void      initRingBuffer(RingbufferTypeDef &ring, volatile CAN_message_t *buffer, uint32_t size);
    void      initializeBuffers(void);
    void      freeBuffers(void);
    bool      isRingBufferEmpty(RingbufferTypeDef &ring);
    uint32_t  ringBufferCount(RingbufferTypeDef &ring);

    template <typename T, size_t N>
    bool      lookupBaudrate(int Baudrate, const T(&table)[N]);
    bool      calculateBaudrate(int Baudrate);
    void      setBaudRateValues(uint16_t prescaler, uint8_t timeseg1,
                                uint8_t timeseg2, uint8_t sjw);
    uint32_t  getCanPeripheralClock(void);
    uint32_t  fixPinFunction(uint32_t function);

    volatile CAN_message_t *rx_buffer = nullptr;
    volatile CAN_message_t *tx_buffer = nullptr;

    static constexpr Baudrate_entry_t BAUD_RATE_TABLE_48M[] {
      {
        1000000, 3, 16, 13, 2
      },
      {
        800000, 4, 15, 12, 2
      },
      {
        500000, 6, 16, 13, 2
      },
      {
        250000, 12, 16, 13, 2
      },
      {
        125000, 24, 16, 13, 2
      },
      {
        100000, 30, 16, 13, 2
      }
    };
	
    static constexpr Baudrate_entry_t BAUD_RATE_TABLE_45M[] {
      {
        1000000, 3, 15, 12, 2
      },
      {
        500000, 5, 18, 15, 2
      },
      {
        250000, 10, 18, 15, 2
      },
      {
        125000, 20, 18, 15, 2
      },
      {
        100000, 25, 18, 15, 2
      }
    };

    static constexpr Baudrate_entry_t BAUD_RATE_TABLE_42M[] {
      {
        1000000, 3, 14, 11, 2
      },
      {
        500000, 6, 14, 11, 2
      },
      {
        250000, 12, 14, 11, 2
      },
      {
        125000, 21, 16, 13, 2
      },
      {
        100000, 28, 15, 12, 2
      }
    };

    static constexpr Baudrate_entry_t BAUD_RATE_TABLE_36M[] {
      {
        1000000, 2, 18, 15, 2
      },
      {
        500000, 4, 18, 15, 2
      },
      {
        250000, 9, 16, 13, 2
      },
      {
        125000, 18, 16, 13, 2
      },
      {
        100000, 20, 18, 15, 2
      }
    };

    bool     _canIsActive = false;

    uint32_t baudrate;
    bool filtersInitialized;

    PinName rx;
    PinName tx;

    uint32_t preemptPriority;
    uint32_t subPriority;

    stm32_can_t _can;

};

#endif
