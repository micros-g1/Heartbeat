#include "Spo2Sensor.h"
#include "drv/max30102/max30102.h"
#include "drv/max30102/algorithm_by_RF.h"
#include "pin_mux.h"
#include "fsl_gpio.h"
#include "fsl_debug_console.h"
#include "semphr.h"

static __volatile__ Sensor sensor;
static TaskHandle_t xTaskSpo2 = NULL;
static TaskHandle_t xTaskHrSpo2 = NULL;

static SemaphoreHandle_t xBinarySemaphore = NULL;
static SemaphoreHandle_t xBinarySemaphore_calculus = NULL;


/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define MAX_SAMP_CHARS 9
#define RF_SAMPLES 100
#define RF_SAMPLES_MARGIN 20

#define SPO2_SENSOR_N_BUFFERS 2

#define UINT18_MAX	((0x00000001<<18) - 1)

/*******************************************************************************
 * Variables
 ******************************************************************************/
static max30102_interrupt_status_t max30102_interrupts;

char hr[5];
char spo2[5];

static uint32_t curr_buffer_n_samples = 0;

static uint32_t ir_led_samples[RF_SAMPLES + RF_SAMPLES_MARGIN];
static uint32_t red_led_samples[RF_SAMPLES + RF_SAMPLES_MARGIN];
static uint32_t ir_led_samples1[RF_SAMPLES + RF_SAMPLES_MARGIN];
static uint32_t red_led_samples1[RF_SAMPLES + RF_SAMPLES_MARGIN];

static uint32_t * ir_buffers[SPO2_SENSOR_N_BUFFERS] = {ir_led_samples, ir_led_samples1};
static uint32_t * red_buffers[SPO2_SENSOR_N_BUFFERS] = {red_led_samples, red_led_samples1};

static int curr_buffer = 0;	//buffer that is being filled.

static int32_t curr_heart_rate = 0;
static int8_t curr_hr_valid = 0;
static float curr_spo2 = 0;
static int8_t curr_spo2_valid = 0;
static float curr_ratio = 0;
static float curr_correl = 0;

void hr_spo2_task(void *pvParameters);
void spo2_task(void *pvParameters);

void spo2_init(uint32_t task_priority);
void spo2_start_sampling(void);
void spo2_stop_sampling(void);


__volatile__ Sensor * new_spo2_sensor(void)
{
	sensor.type = SENSOR_SPO2;
	sensor.init = spo2_init;
	sensor.start_sampling = spo2_start_sampling;
	sensor.stop_sampling = spo2_stop_sampling;
	sensor.status = false;

	return &sensor;
}

void spo2_init(uint32_t task_priority)
{
	sensor.status = max30102_init(MAX30102_SPO2_MODE) == MAX30102_SUCCESS;
	if(!sensor.status) {
		PRINTF("MAX30102 INITIALIZATION ERROR\n");
		return;
	}

	sensor.status = max30102_set_led_current(0x24,  MAX30102_LED_PULSE_AMPLITUDE_ADDR_1);
	if(!sensor.status) {
		PRINTF("MAX30102 LED1 ERROR\n");
		return;
	}

	sensor.status = max30102_set_led_current(0x24,  MAX30102_LED_PULSE_AMPLITUDE_ADDR_2);
	if(!sensor.status){
		PRINTF("MAX30102 LED2 ERROR\n");
		return;
	}

	max30102_fifo_configuration_t fifo_conf;
	fifo_conf.val = 0;
	fifo_conf.fifo_a_full = 0xF;
	fifo_conf.fifo_roll_over_en = true;
	fifo_conf.smp_ave = MAX30102_SMP_AVE_4;
	sensor.status = max30102_set_fifo_config(&fifo_conf);
	if(!sensor.status) {
		PRINTF("MAX30102 FIFO CONFIG ERROR\n");
		return;
	}
	max30102_spo2_configuration_t spo2_conf;
	spo2_conf.val = 0;
	spo2_conf.led_pw = MAX30102_LED_PW_411US_ADC_18_BITS;
	spo2_conf.spo2_sr = MAX30102_SPO2_SAMPLE_RATE_100HZ;
	spo2_conf.spo2_adc_rge = MAX30102_SPO2_ADC_RESOLUTION_4096NA;
	sensor.status = max30102_set_spo2_config(&spo2_conf);
	if(!sensor.status) {
		PRINTF("MAX30102 SPO2 CONFIG ERROR\n");
		return;
	}
	memset(&max30102_interrupts, 0, sizeof(max30102_interrupts));

	sensor.status = xTaskCreate(
			spo2_task, "spo2 task",
			configMINIMAL_STACK_SIZE + 166, NULL,
			task_priority, &xTaskSpo2) == pdTRUE;

	sensor.status = xTaskCreate(
			hr_spo2_task, "hr spo2 task",
			configMINIMAL_STACK_SIZE + 500, NULL,
			task_priority-1, &xTaskHrSpo2) == pdTRUE;

	if (!sensor.status) {
		PRINTF("Spo2 task creation failed!.\r\n");
		return;
	}

	sensor.status = (xBinarySemaphore = xSemaphoreCreateBinary()) != NULL;
	if(sensor.status)
		sensor.status = (xBinarySemaphore_calculus = xSemaphoreCreateBinary()) != NULL;

	if (!sensor.status) {
		PRINTF("Spo2 semaphore creation failed!.\r\n");
		return;
	}

	for(int i=0; i < RF_SAMPLES + RF_SAMPLES_MARGIN;i++){
		ir_led_samples[i] = 0;
		ir_led_samples1[i] = 0;
		red_led_samples[i] = 0;
		red_led_samples1[i] = 0;
	}

	set_limits(EVENT_SPO2_LED, 0.0, 1.0);
}


void spo2_start_sampling(void)
{
	sensor.status = max30102_trigger_spo2_reads() == MAX30102_SUCCESS;
	if (!sensor.status) {
		PRINTF("Could not start Spo2 task!\n");
		return;
	}
	vTaskResume(xTaskSpo2);
}


void spo2_stop_sampling(void)
{
	vTaskSuspend(xTaskSpo2);
}

void hr_spo2_task(void *pvParameters){
	uint8_t curr_debuffer = 0;
	while(true){
		xSemaphoreTake(xBinarySemaphore_calculus, portMAX_DELAY);
		curr_debuffer = ((curr_buffer-1) == -1) ? (SPO2_SENSOR_N_BUFFERS-1): curr_buffer-1;
		rf_heart_rate_and_oxygen_saturation(red_buffers[curr_debuffer], RF_SAMPLES, ir_buffers[curr_debuffer],
			&curr_spo2, &curr_spo2_valid, &curr_heart_rate, &curr_hr_valid, &curr_ratio, &curr_correl);

		if(curr_hr_valid && curr_spo2_valid){
			write_sample((float)curr_heart_rate, EVENT_SPO2_BPM, NULL);
			write_sample((float)curr_spo2, EVENT_SPO2_SPO2, NULL);
		}
		else{
			write_sample((float)curr_heart_rate, EVENT_SPO2_BPM_NOT_VALID, NULL);
			write_sample((float)curr_spo2, EVENT_SPO2_SPO2_NOT_VALID, NULL);
		}
	}
}

void spo2_task(void *pvParameters)
{
	while(true) {
		while (!GPIO_PinRead(GPIOB, BOARD_MAX30102_INT_PIN_PIN)) {
			max30102_get_interrupt_status(&max30102_interrupts);

			if(max30102_interrupts.pwr_rdy){
				max30102_interrupts.pwr_rdy = false;
				// no action necessary as the pwr_rdy flag is cleared when read.
				// "The interrupts are cleared whenever the interrupt status register is read,
				// or when the register that triggered the interrupt is read".
			}

			if(max30102_interrupts.ppg_rdy){
				max30102_interrupts.ppg_rdy = false;

				uint32_t aux_sample_red = 0;
				uint32_t aux_sample_ir = 0;

				max30102_read_sample(&aux_sample_ir, &aux_sample_red);
				ir_buffers[curr_buffer][curr_buffer_n_samples] = aux_sample_ir;
				red_buffers[curr_buffer][curr_buffer_n_samples] = aux_sample_red;

				write_sample(((float)aux_sample_ir) / ((float)UINT18_MAX), EVENT_SPO2_LED, NULL);

				curr_buffer_n_samples++;

				//is the buffer full ? (should the accumulated samples be processed?)
				if(curr_buffer_n_samples >= RF_SAMPLES){
					curr_buffer = curr_buffer == (SPO2_SENSOR_N_BUFFERS-1) ? 0 : curr_buffer + 1;
					curr_buffer_n_samples = 0;
					//the red led is switched with the ir led on the board.
					xSemaphoreGive(xBinarySemaphore_calculus);
				}
			}
			if(max30102_interrupts.alc_ovf){
				max30102_interrupts.alc_ovf = false;

			}
			if(max30102_interrupts.die_temp_rdy){
				max30102_interrupts.die_temp_rdy = false;

			}

			if(max30102_interrupts.a_full){
				max30102_interrupts.a_full = false;
			}

		}
		xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
	}
}


/* PORTB_IRQn interrupt handler */
void GPIOB_IRQHANDLER(void) {
  /* Get pin flags */
	if (xTaskSpo2 == NULL || xBinarySemaphore == NULL)
		return;

	BaseType_t xHigherPriorityTaskWoken;
	uint32_t pin_flags = GPIO_PortGetInterruptFlags(GPIOB);

	for(int i = 0; i < sizeof(pin_flags)*8; i++) {
		if(pin_flags & (1 << i)){
			if(i == BOARD_MAX30102_INT_PIN_PIN) {
				xSemaphoreGiveFromISR(xBinarySemaphore, &xHigherPriorityTaskWoken);
			}
		}
	}

  /* Clear pin flags */
  GPIO_PortClearInterruptFlags(GPIOB, pin_flags);

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
