
#include <stdint.h>

/* lpc2148 board */
#include <lpc2000/io.h>
#include <lpc2000/interrupt.h>

/* interrupts driver */
#include "drivers/vic.c"


/* serial driver */
#define CLOCKS_PCLK 60000000
#define UART0_BAUD_RATE 19200
#include "drivers/uart0-polling.c"

#define putchar(c) uart0SendByte(c)
#include "printf.c"

#define DBG(fmt, args...) printf(fmt "%s", ##args, "\n")
#define TRACE_ENTER(func) printf("-> %s\n", func)
#define TRACE_RETURN(func) printf("<- %s\n", func)
#define ASSERT(x)

typedef int bool;

#include "drivers/usb.h"

#include "uip.h"
#include "uip_arp.h"

#include "usb-eth_dev.c"

#include "drivers/usb.c"

void uip_log(char *m)
{
  printf("uIP log message: %s\n", m);
}

#include "net_timers.c"

void telnetd_init(void);

int main(int argc, char *argv[])
{
  uip_ipaddr_t ipaddr;
  uint16_t eth_hdr_type;
  int i;

  VPBDIV = 0x00000001;  /* PCLK = CCLK */
  PLLCFG  = 0x00000024; /* Fosc=12MHz, CCLK=60MHz */
  PLLCON  = 0x00000001; /* enable the PLL  */
  PLLFEED = 0x000000AA; /* feed sequence   */
  PLLFEED = 0x00000055;
  while (!(PLLSTAT & 0x00000400));
  PLLCON = 3;     // enable and connect
  PLLFEED = 0xAA;
  PLLFEED = 0x55;

  vicInit();
  uart0Init();

  DBG("Initializing USB Stack");
  usbInit();
  usbEnableNAKInterrupts(INACK_II);

  network_device_init();
  
  uip_init();

  uip_ipaddr(ipaddr, 10, 3, 0, 1);
  uip_sethostaddr(ipaddr);

  telnetd_init();

  net_timers_init();
  

  DBG("Starting USB Stack");
  interruptsEnable();
  usbConnect();

  
  while(1) {
    uip_len = network_device_read();
    if(uip_len > 0) {
      eth_hdr_type = (uip_buf[0xc] << 8) | uip_buf[0xd];
      if(eth_hdr_type == UIP_ETHTYPE_IP) {
        uip_arp_ipin();
        uip_input();
        if(uip_len > 0) {
          uip_arp_out();
          network_device_send();
        }
      }
      else if(eth_hdr_type == UIP_ETHTYPE_ARP) {
        uip_arp_arpin();
        if(uip_len > 0) {
          network_device_send();
        }
      }

    }
    else if(net_timers_ip_expired()) {
      net_timers_ip_reset();
      for(i = 0; i < UIP_CONNS; i++) {
        uip_periodic(i);
        if(uip_len > 0) {
          uip_arp_out();
          network_device_send();
        }
      }

      if(net_timers_arp_expired()) {
        net_timers_arp_reset();
        uip_arp_timer();
      }
    }
  }
  return 0;
}
