
int is_i2c_initialized = 0;
void temperature(char *cmd, char *args)
{
  int rc;
  uint32_t temperature_2comp;
  int temperature;
  int negative = 0;

  char response[2];
  char str[64];
  
  if (!is_i2c_initialized) {
    i2cInit();
  }
  
  rc = i2cMasterTransact(0x90, 0, 0, (uint8_t*)response, 2);
    
  if (rc == 2) {
    /* first byte:   D15 D14 D13 D12 D11 D10 D9 D8
       seconds byte: D7  D6  D5  D4  D3  D2  D1 D0
    */
    temperature_2comp = (response[0] << 1) | (response[1] >> 7);
    if (temperature_2comp & (1 << 8)) {
      temperature = (((~(temperature_2comp)) + 1) & 0x7f);
      negative = 1;
    }
    else {
      temperature = temperature_2comp;
    }
    
    sprintf(str, "%s%d%s C",
            negative ? "-" : "",
            temperature/2,
            temperature % 2 ? ".5" : "");
    shell_output(str, "");
  }
  else {
    sprintf(str, "failed to read temperature, rc=%d", rc);
    shell_output(str, "");
  }
}
