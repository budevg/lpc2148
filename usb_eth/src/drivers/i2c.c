
#define I2C_CONSET_AA       (BIT2)
#define I2C_CONSET_SI       (BIT3)
#define I2C_CONSET_STO      (BIT4)
#define I2C_CONSET_STA      (BIT5)
#define I2C_CONSET_I2EN     (BIT6)
#define I2C_CONSET_MASK     (BIT2|BIT3|BIT4|BIT5|BIT6)

#define I2C_CONCLR_AAC      (BIT2)
#define I2C_CONCLR_SIC      (BIT3)
#define I2C_CONCLR_STAC     (BIT5)
#define I2C_CONCLR_I2ENC    (BIT6)
#define I2C_CONCLR_MASK     (BIT2|BIT3|BIT5|BIT6)

#define I2C_STAT_MASK       (0xf8)

typedef struct i2c_transaction_s {
  uint8_t address;
  uint8_t* command_data;
  int32_t command_data_len;
  uint8_t* response_data;
  int32_t response_data_len;
  int32_t counter;
} i2c_transaction_t;


volatile static i2c_transaction_t i2c_info;

#define TRUE  (1)
#define FALSE (0)
static volatile char i2c_bus_in_use;

#define RC_OK                            (0)
#define RC_NACK_AFTER_SLA                (-1)
#define RC_NACK_AFTER_DATA_TX            (-2)
#define RC_UNKNOWN_STATUS                (-3)
#define RC_TIMEOUT                       (-4)
static int32_t i2c_error_code;

void __attribute__ ((interrupt("IRQ"))) i2cIsr ()
{
  switch (I20STAT & I2C_STAT_MASK) {
  case 0x08:  /* start */
  case 0x10: /* repeated start */
    I20CONCLR = I2C_CONCLR_STAC;
    I20DAT = i2c_info.address;
    break;
    
  case 0x18: /* ack after SLA+W */
  case 0x28: /* ack after data byte transmit */
    if (i2c_info.command_data_len > 0 &&
        i2c_info.counter < i2c_info.command_data_len) {
      I20DAT = i2c_info.command_data[i2c_info.counter];
      I20CONCLR = I2C_CONCLR_STAC;
      i2c_info.counter++;
    }
    else {
      if (i2c_info.response_data_len > 0) {
        i2c_info.address |= 0x1; /* move to receiver mode */
        i2c_info.counter = 0;
        I20CONSET = I2C_CONSET_STA;
      }
      else {
        /* no need to read data ... finish transaction */
        I20CONCLR = I2C_CONCLR_STAC;
        I20CONSET = I2C_CONSET_STO;
        i2c_error_code = RC_OK;
        i2c_bus_in_use = FALSE;
      }
    }
    break;
  case 0x20: /* nack after SLA+W */
  case 0x48: /* nack after SLA+R */
    I20CONCLR = I2C_CONCLR_STAC;
    I20CONSET = I2C_CONSET_STO;
    i2c_error_code = RC_NACK_AFTER_SLA;
    i2c_bus_in_use = FALSE;
    break;
  case 0x30: /* nack after data byte transmit */
    I20CONCLR = I2C_CONCLR_STAC;
    I20CONSET = I2C_CONSET_STO;
    i2c_error_code = RC_NACK_AFTER_DATA_TX;
    i2c_bus_in_use = FALSE;
    break;
  case 0x38: /* arbitration lost */
    I20CONSET = I2C_CONSET_STA;
    break;
  case 0x58: /* data byte received nack was sent */
    /* last byte received */
    i2c_info.response_data[i2c_info.counter] = I20DAT;
    i2c_info.counter++;
    I20CONCLR = I2C_CONCLR_STAC;
    I20CONSET = I2C_CONSET_STO;
    i2c_error_code = RC_OK;
    i2c_bus_in_use = FALSE;
    break;

  case 0x50: /* data byte received ack was sent */
    i2c_info.response_data[i2c_info.counter] = I20DAT;
    i2c_info.counter++;
    /* fall through  */
  case 0x40: /* ack after SLA+R */
    if (i2c_info.counter < i2c_info.response_data_len-1) {
      I20CONCLR = I2C_CONCLR_STAC;
      I20CONSET = I2C_CONSET_AA;
    }
    else {
      I20CONCLR = I2C_CONCLR_STAC | I2C_CONCLR_AAC;
    }
    break;
  default:
    I20CONCLR = I2C_CONCLR_I2ENC;
    i2c_error_code = RC_UNKNOWN_STATUS;
    i2c_bus_in_use = FALSE;
  }

  I20CONCLR = 0x08; /*I2C_CONCLR_SIC;*/
  vicUpdatePriority();
    
}

void i2cInit()
{
  vicEnableIRQ(INT_CHANNEL_I2C0, 3, i2cIsr);
                 
  PINSEL0 =  (PINSEL0 & ~(BIT4|BIT5|BIT6|BIT7)) | (BIT4 | BIT6);      /* switch gpio to i2c pins */

  /* set bit rate */
  I20SCLH = 80;//CLOCKS_PCLK / (2*I2C_BIT_RATE) + (CLOCKS_PCLK % (2*I2C_BIT_RATE) ? 1 : 0);
  I20SCLL = 80;//CLOCKS_PCLK / (2*I2C_BIT_RATE) + (CLOCKS_PCLK % (2*I2C_BIT_RATE) ? 1 : 0);

  /* clear control bits and enable i2c interface */
  I20CONCLR = I2C_CONCLR_MASK;
  I20CONSET = I2C_CONSET_I2EN;

    
}


int32_t i2c_wait_completion()
{
  int max_iteration = 200000;
  while (max_iteration > 0) {
    if (i2c_bus_in_use == FALSE) {
      return RC_OK;
    }
    max_iteration--;
  }
  return RC_TIMEOUT;
  
}
int32_t i2cMasterTransact(uint8_t slave_address,
                          uint8_t* command,
                          int32_t command_len,
                          uint8_t* response,
                          int32_t response_len)
{
  int32_t rc;
  if (command_len > 0) {
    slave_address &= ~0x1;
  }
  else if (response_len > 0) {
    slave_address |= 0x1;
  }
  else {
    /* nothing to do */
    return 0;
  }


  if (i2c_bus_in_use == TRUE) {
    rc = i2c_wait_completion();
    if (rc != RC_OK) {
      return rc;
    }
  }

  i2c_bus_in_use = TRUE;
  i2c_info.address = slave_address;
  i2c_info.command_data = command;
  i2c_info.command_data_len = command_len;
  i2c_info.response_data = response;
  i2c_info.response_data_len = response_len;
  i2c_info.counter = 0;

  I20CONCLR = I2C_CONCLR_MASK;
  I20CONSET = I2C_CONSET_I2EN;
  I20CONSET = I2C_CONSET_STA;

  rc = i2c_wait_completion();
  if (rc != RC_OK) {
    return rc;
  }
  

  if (i2c_error_code != RC_OK) {
    return i2c_error_code;
  }
  
  if (response_len > 0) {
    return i2c_info.counter;
  }
  else {
    return 0;
  }
}
