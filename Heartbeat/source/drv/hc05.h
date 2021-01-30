#include <stdint.h>
#include <stdbool.h>
#include "drivers/fsl_gpio.h"

typedef enum{
	HC05_SUCCESS = true,
	HC05_FAILURE = false
}hc05_state_t;



hc05_state_t hc05_init();
hc05_state_t hc05_send_data(uint8_t * data, unsigned int len);

