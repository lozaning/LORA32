/* Stage-0 modem build reserves three 64 KiB RF banks. The original full
 * 25 KiB framebuffer is deliberately excluded from this diagnostic image. */
#include "display.h"
#include "driver/gpio.h"
bool display_init(void) {
    gpio_set_direction(38,GPIO_MODE_OUTPUT);gpio_set_level(38,1);return false;
}
void display_status(const char *a,const char *b){(void)a;(void)b;}
void display_waveform(const int8_t *iq,unsigned n,const char *label,bool tx){(void)iq;(void)n;(void)label;(void)tx;}
