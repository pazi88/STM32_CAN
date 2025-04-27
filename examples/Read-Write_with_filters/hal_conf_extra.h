#if !defined(HAL_CAN_MODULE_ENABLED)
    #define HAL_CAN_MODULE_ENABLED

    #if defined(STM32F1xx)
        /** NOTE: Enable special alternate function remapping for F1 platform*/
        #define AFIO_MAPR_CAN_REMAP1
    #endif

    #if defined(USBCON) && defined(STM32F1xx)
        /** NOTE: On the F1 platform CAN and USB may not be used at the same time.
         * To still allow a program to be build that may use both
         * but not at the same time this workaround may be enabled.
         * Since USB driver is using the shared IRQ handlers, the CAN driver has no access to them.
         * To handle Tx events call
         * 
         * STM32_CAN_Poll_IRQ_Handler()
         * 
         * frequently
         */
        // #define STM32_CAN_USB_WORKAROUND_POLLING
    #endif

    #if defined(USBCON) && defined(STM32F3xx)
        /** NOTE: On F3 platform CAN and USB share IRQ by default.
         *  Since USB driver is using the shared IRQ handlers, the CAN driver has no access to them.
         * 
         *  Below define maps the USB IRQs to alternate IRQ vectors, 
         *  so USB and CAN IRQs are no longer shared
         */
        // #define USE_USB_INTERRUPT_REMAPPED
    #endif

#endif