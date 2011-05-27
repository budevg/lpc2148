
int lcd_initialized = 0;

void lcd(char *cmd, char *args)
{
  
  if (!lcd_initialized) {
    lcdInit(BUS_INTERFACE_8_BIT);
  }

  lcdClearScreen(BUS_INTERFACE_8_BIT);
  lcdReturnHome(BUS_INTERFACE_8_BIT);
  lcdPrintString(BUS_INTERFACE_8_BIT, args, 50);
}
