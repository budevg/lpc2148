
void motor(char *cmd, char *args)
{
  int i;
  volatile int j;
  int motor_counter = 0;

  IODIR0 |= (BIT21|BIT12);
  
  for (i=0; i<50; i++) {
    motor_counter = (motor_counter + 1) % 4;
    if (motor_counter  == 0) {
      IOSET0 = BIT12;
      IOSET0 =  BIT21;
    }
    else if (motor_counter == 1) {
      IOCLR0 = BIT12;
      IOSET0 =  BIT21;
    }
    else if (motor_counter == 2) {
      IOCLR0 = BIT12;
      IOCLR0 =  BIT21;
    }
    else if (motor_counter == 3) {
      IOSET0 = BIT12;
      IOCLR0 =  BIT21;
    }
    for (j=0;j<10000;j++);
  }
}
