
#define INSTRUCTIONS_PER_SECOND (60 * 1000000)
#define INSTRUCTIONS_PER_MICRO_SECOND (INSTRUCTIONS_PER_SECOND/1000000)

#define BUS_INTERFACE_8_BIT (0)
#define BUS_INTERFACE_4_BIT (1)
#define DATA_TYPE (1)
#define CMD_TYPE (0)


/* busywait is the delay function
   since it is not calibrated in average it waits around milisecond
   even though it should wait about microsecond*/
void busywait1(uint32_t microseconds)
{
  volatile int i = microseconds * INSTRUCTIONS_PER_MICRO_SECOND;
  for (; i>0;i--);
}


void flush()
{
  lcd_en_set();
  busywait1(10);
  lcd_en_clear();
  busywait1(10);
}

/* send command or data based on bus interface (4/8 bit)
   c is the D0..D7 value */
void lcdSendCommand(int bus_interface, int is_data, char c)
{
  lcd_en_rs_out();
  lcd_rw_out();
  if (bus_interface == BUS_INTERFACE_8_BIT) {
    lcd_data8_out();
  }
  else {
    lcd_data4_out();
  }
  if (is_data) {
    lcd_rs_set();
  }
  else {
    lcd_rs_clear();
  }
  lcd_rw_clear();

  if (bus_interface == BUS_INTERFACE_8_BIT) {
    lcd_data8_set(c);
    flush();
  }
  else {
    lcd_data4_set((c >> 4) & 0xf);
    flush();
    lcd_data4_set((c & 0xf));
    flush();
  }
}

/* for debugging purposes */
void toggleLed()
{
  IODIR0 |= BIT12;
  if (IOPIN0 & BIT12)
    IOCLR0 = BIT12;
  else
    IOSET0 = BIT12;
}

void lcdInit(int bus_interface)
{
  int i;
  lcd_backlight_on();
  lcd_en_rs_out();
  lcd_rw_out();
  
  if (bus_interface == BUS_INTERFACE_8_BIT) {
    lcd_data8_out();
  }
  else {
    lcd_data4_out();
  }

  /* wait after power on */
  busywait1(500);

  for (i=0;i<3;i++) {
    if (bus_interface != BUS_INTERFACE_8_BIT) {
      lcd_rs_clear();
      lcd_rw_clear();
      lcd_data4_set(0x3); 
      flush();
      busywait1(500);
    }
    else {
      lcdSendCommand(BUS_INTERFACE_8_BIT, CMD_TYPE, 0x30); /* 1 line, 5x7 dots */
      busywait1(500);
    }
  }

  if (bus_interface != BUS_INTERFACE_8_BIT) {
    lcd_rs_clear();
    lcd_rw_clear();
    lcd_data4_set(0x2); 
    flush();
    busywait1(500);

    lcdSendCommand(BUS_INTERFACE_4_BIT, CMD_TYPE, 0x2c); /* 1 line, 5x7 dots */
  }
  else {
    lcdSendCommand(BUS_INTERFACE_8_BIT, CMD_TYPE, 0x3c); /* 1 line, 5x7 dots */
  }

  busywait1(500); /* wait after lcd line type configuration */
  lcdSendCommand(bus_interface, CMD_TYPE, 0xf); /* display on, cursor on,
                                                     blinking cursor */
  busywait1(500); /* wait after display configuration */
  
  lcdSendCommand(bus_interface, CMD_TYPE, 0x1); /* display clear */
  busywait1(500); /* wait after display clear */
  
  lcdSendCommand(bus_interface, CMD_TYPE, 0x6); /* set entry mode to increment
                                                     and shift  */
}

/* address represnts offset on display (DDRAM address) */
void lcdGoto(int bus_interface, char address)
{
  lcdSendCommand(bus_interface, CMD_TYPE, address | (1 << 7));
}


void lcdPrintChar(int bus_interface, char c)
{
  lcdSendCommand(bus_interface, DATA_TYPE, c);
}


void lcdSingleDisplayShiftLeft(int bus_interface)
{
  lcdSendCommand(bus_interface, CMD_TYPE, 0x18);
}

/* aditional delay after each character print */
void lcdPrintString(int bus_interface, char* s, int delay)
{
  int i=0;
  while (*s) {
    if (*s == '\n') {
      lcdGoto(bus_interface,0x40+3);
      i = 0;
      s++;
      continue;
    }
    if (i > 16) {
      /* shift for long strings */
      lcdSingleDisplayShiftLeft(bus_interface);
    }
    lcdPrintChar(bus_interface, *s);
    i++;
    s++;
    busywait1(delay);
  }
}

void lcdReturnHome(int bus_interface)
{
  lcdSendCommand(bus_interface, CMD_TYPE, 0x2);
}

void lcdClearScreen(int bus_interface)
{
  lcdSendCommand(bus_interface, CMD_TYPE, 0x1);
}


