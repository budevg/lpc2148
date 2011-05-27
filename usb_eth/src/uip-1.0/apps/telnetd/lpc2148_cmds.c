#include <stdint.h>
#include <lpc2000/io.h>
#include <lpc2000/interrupt.h>
#include "drivers/vic.c"

/* forward declarations */
int printf(const char *format, ...);
int sprintf(char *out, const char *format, ...);

#define DATA_8_MASK  (BIT16|BIT17|BIT18|BIT19|BIT20|BIT21|BIT22|BIT23)
#define DATA_4_MASK  (BIT20|BIT21|BIT22|BIT23)
#define DATA_8_OFFSET (16)
#define DATA_4_OFFSET (20)

#define lcd_backlight_on()                      \
do {                                            \
  IODIR0 |= BIT30;                              \
  IOSET0 = BIT30;                               \
}while (0);
#define lcd_backlight_off()                     \
do {                                            \
  IODIR0 |= BIT30;                              \
  IOCLR0 = BIT30;                               \
}while (0);
#define lcd_en_rs_out() (IODIR1 |= (BIT24 | BIT25))
#define lcd_rw_out() (IODIR0 |= BIT22)
#define lcd_data8_out() (IODIR1 |= DATA_8_MASK)
#define lcd_data8_in() (IODIR1 &= ~DATA_8_MASK)
#define lcd_data4_out() (IODIR1 |= DATA_4_MASK)
#define lcd_data4_in() (IODIR1 &= ~DATA_4_MASK)
#define lcd_data8_set(c)                                                \
do {                                                                    \
  IOCLR1 = DATA_8_MASK; /* clear D0..D7 */                              \
  IOSET1 = ((c) & 0xff) << DATA_8_OFFSET; /* set D0..D7 to the c value */ \
}while (0)
#define lcd_data4_set(c)                                                \
do {                                                                    \
  IOCLR1 = DATA_4_MASK; /* clear D4..D7 */                              \
  IOSET1 = ((c) & 0xf) << DATA_4_OFFSET; /* set D4..D7 to the c value */ \
} while (0)
#define lcd_rs_set() (IOSET1 = BIT24)
#define lcd_rs_clear() (IOCLR1 = BIT24)
#define lcd_en_set() (IOSET1 = BIT25)
#define lcd_en_clear() (IOCLR1 = BIT25)
#define lcd_rw_set() (IOSET0 = BIT22)
#define lcd_rw_clear() (IOCLR0 = BIT22)

#include "drivers/char-lcd.c"

#include "lcd_cmd.c"

#include "matrix_cmd.c"

#include "rgb_cmd.c"

#include "drivers/i2c.c"

#include "temperature_cmd.c"

#include "motor_cmd.c"
