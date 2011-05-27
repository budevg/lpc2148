typedef uint32_t interrupts_state_t;

#ifndef __thumb__

static inline interrupts_state_t interruptsSaveAndEnable(void) {
  uint32_t cpsr;
  asm volatile (                            
        "mrs %0,cpsr;"
        "mrs r4,cpsr;"
        "bic r4,r4,#0xC0;"
        "msr cpsr,r4"
        : "=r" (cpsr) 
        : /* no inputs */ 
        : "r4" );
  return cpsr;
}

static inline interrupts_state_t interruptsSaveAndDisable(void) {
  uint32_t cpsr;
  asm volatile (                            
        "mrs %0,cpsr;"
        "mrs r4,cpsr;"
        "orr r4,r4,#0xC0;"
        "msr cpsr,r4"
        : "=r" (cpsr) 
        : /* no inputs */ 
        : "r4" );
  return cpsr;
}

static inline void interruptsRestore(interrupts_state_t cpsr) {
  asm volatile (                              
      "mrs r3,cpsr;"                          
      "and r4,%0,#0xC0;"                      
      "bic r3,r3,#0xC0;"                      
      "orr r3,r3,r4;"                         
      "msr cpsr,r3"                           
      :                                       
      : "r" (cpsr)                            
      : "r3", "r4"                            
      );
}

static inline void interruptsEnable(void) {
    asm volatile (                              
        "mrs r3,cpsr;"                          
        "bic r3,r3,#0xC0;"                      
        "msr cpsr,r3"                           
        :                                       
        :                                       
        : "r3"                                  
        );
}
        
#else /* __thumb__ */
#error "interrupt state manipulation not supported in thumb mode"
#endif
