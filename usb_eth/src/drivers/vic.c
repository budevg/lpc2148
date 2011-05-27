typedef void (*interrupt_handler_t)(void); 

#if    defined(LPC2119)
typedef enum {
  INT_CHANNEL_WDT    = 0,
  INT_CHANNEL_SWI    = 1,
  INT_CHANNEL_CORE1  = 2,
  INT_CHANNEL_CORE2  = 3,
  INT_CHANNEL_TIMER0 = 4,
  INT_CHANNEL_TIMER1 = 5,
  INT_CHANNEL_UART0  = 6,
  INT_CHANNEL_UART1  = 7,
  INT_CHANNEL_PWM0   = 8,
  INT_CHANNEL_I2C    = 9,
  INT_CHANNEL_SPI0   = 10,
  INT_CHANNEL_SPI1   = 11,
  INT_CHANNEL_PLL    = 12,
  INT_CHANNEL_RTC    = 13,
  INT_CHANNEL_EINT0  = 14,
  INT_CHANNEL_EINT1  = 15,
  INT_CHANNEL_EINT2  = 16,
  INT_CHANNEL_EINT3  = 17,
  INT_CHANNEL_ADC    = 18,
  INT_CHANNEL_CANACC = 19,
  INT_CHANNEL_CAN1TX = 20,
  INT_CHANNEL_CAN2TX = 21,
  INT_CHANNEL_CAN3TX = 22,
  INT_CHANNEL_CAN4TX = 23,
  INT_CHANNEL_CAN1RX = 26,
  INT_CHANNEL_CAN2RX = 27,
  INT_CHANNEL_CAN3RX = 28,
  INT_CHANNEL_CAN4RX = 29
} interrupt_channel_t;
#endif

#if    defined(LPC2148)
typedef enum {
  INT_CHANNEL_WDT    = 0,
  INT_CHANNEL_SWI    = 1,
  INT_CHANNEL_CORE1  = 2,
  INT_CHANNEL_CORE2  = 3,
  INT_CHANNEL_TIMER0 = 4,
  INT_CHANNEL_TIMER1 = 5,
  INT_CHANNEL_UART0  = 6,
  INT_CHANNEL_UART1  = 7,
  INT_CHANNEL_PWM0   = 8,
  INT_CHANNEL_I2C0   = 9,
  INT_CHANNEL_SPI0   = 10,
  INT_CHANNEL_SPI1   = 11,
  INT_CHANNEL_PLL    = 12,
  INT_CHANNEL_RTC    = 13,
  INT_CHANNEL_EINT0  = 14,
  INT_CHANNEL_EINT1  = 15,
  INT_CHANNEL_EINT2  = 16,
  INT_CHANNEL_EINT3  = 17,
  INT_CHANNEL_ADC    = 18,
  INT_CHANNEL_I2C1   = 19,
  INT_CHANNEL_BOD    = 20,
  INT_CHANNEL_ADC1   = 21,
  INT_CHANNEL_USB    = 22
} interrupt_channel_t;
#endif
                        
#define vicUpdatePriority() (VICVectAddr=0xff)

static void __attribute__ ((interrupt("IRQ")))
  _vicDefaultIsr(void) {
  vicUpdatePriority();
}

static void vicInit(void) {
  VICDefVectAddr = (uint32_t) _vicDefaultIsr;
}

static volatile uint32_t* address_register = &VICVectAddr0;
static volatile uint32_t* control_register = &VICVectCntl0;

static void vicEnableFIQ(interrupt_channel_t c) {
  VICIntSelect |= (1 << c); /* generate FIQ interrupt */
  VICIntEnable |= (1 << c);
}

static void vicEnableIRQ(interrupt_channel_t c, 
                  uint32_t            priority,
                  interrupt_handler_t h) {
  address_register[priority] = (uint32_t) h;
  control_register[priority] = c | (1<<5);
  VICIntSelect &= ~(1 << c); /* generate IRQ interrupt */
  VICIntEnable = (1 << c);
}

static void vicEnableDefaultIRQ(interrupt_handler_t h) {
  VICDefVectAddr = (uint32_t) h;
}

static inline void vicEnable(interrupt_channel_t c) {
  VICIntEnable = (1 << c);
}

static inline void vicDisable(interrupt_channel_t c) {
  VICIntEnClr = (1 << c);
}

static void ramvectorsInit(void) {
  uint32_t i;
  uint32_t* src = (uint32_t*) 0x40000200;
  uint32_t* dst = (uint32_t*) 0x40000000;
  
  for (i=0; i<16; i++) {
    *dst = *src;
    src  ++;
    dst++;
  }
  
  MEMMAP = BIT1;
}