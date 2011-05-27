
char* color_names[] = {"off","red","blue","purple","green","yellow", "cyan", "white"};
void rgb(char *cmd, char *args)
{
  int i;
  PINSEL0 &= ~(BIT14|BIT15|BIT16|BIT17|BIT18|BIT19);
  IODIR0 |= BIT7|BIT8|BIT9;
  for (i=0; i<sizeof(color_names)/sizeof(color_names[0]); i++) {
    if (strncmp(args, color_names[i], strlen(color_names[i])) == 0) {
      IOCLR0 = BIT7|BIT8|BIT9;
      IOSET0 = (i & 0x7) << 7;
      return;
    }
  }
  shell_output("invalid color ", args);
  shell_output("use colors: [off,red,blue,purple,green,yellow,cyan,white]","");
}
