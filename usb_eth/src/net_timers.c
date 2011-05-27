
#define ARP_TIMER_IN_MSEC (1000*10)
#define IP_TIMER_IN_MSEC (500)

static uint16_t arp_timer_counter = 0;
volatile static uint16_t arp_timer_expired = 0;

static uint16_t ip_timer_counter = 0;
volatile static uint16_t ip_timer_expired =0 ;

void  __attribute__ ((interrupt("IRQ"))) netTimersIsr(void) 
{
  T0IR = BIT0;
  arp_timer_counter++;
  ip_timer_counter++;

  if (arp_timer_counter >= ARP_TIMER_IN_MSEC) {
    arp_timer_counter = 0;
    arp_timer_expired = 1;
  }
  if (ip_timer_counter >= IP_TIMER_IN_MSEC) {
    ip_timer_counter = 0;
    ip_timer_expired = 1;
  }
  vicUpdatePriority();
}

void net_timers_init()
{
  /* setup timer for uip_periodic and uip_arp_timer calls */
  vicEnableIRQ(INT_CHANNEL_TIMER0, 1 /* priority */, netTimersIsr);
  T0PR = 60000; /* PCKLC = 60 MHz, timer resolution 1 msec */
  T0MR0 = 1;
  T0MCR = BIT0 | BIT1; /* interrutp and reset timer when count reached */
  T0TCR = BIT0; /* enable the counter */

}

int net_timers_ip_expired()
{
  return ip_timer_expired;
}

void net_timers_ip_reset()
{
  ip_timer_expired = 0;
}

int net_timers_arp_expired()
{
  return arp_timer_expired;
}

void net_timers_arp_reset()
{
  arp_timer_expired = 0;
}
