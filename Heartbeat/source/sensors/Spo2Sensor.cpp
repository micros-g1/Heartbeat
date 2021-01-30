#include "Spo2Sensor.h"
#include "drv/max30102/max30102.h"
#include "drv/max30102/algorithm_by_RF.h"
#include "pin_mux.h"
#include "fsl_gpio.h"
#include "semphr.h"

static Spo2Sensor * sensor = nullptr;
static TaskHandle_t xTaskSpo2 = nullptr;
static SemaphoreHandle_t xBinarySemaphore = nullptr;


/*******************************************************************************
 * Variables
 ******************************************************************************/
static max30102_interrupt_status_t max30102_interrupts;

char hr[5];
char spo2[5];

static uint32_t curr_buffer_n_samples = 0;

uint32_t ir_led_samples[RF_SAMPLES];
uint32_t red_led_samples[RF_SAMPLES];
unsigned char samp1[MAX_SAMP_CHARS];
unsigned char samp2[MAX_SAMP_CHARS];

static int32_t curr_heart_rate = 0;
static int8_t curr_hr_valid = 0;
static float curr_spo2 = 0;
static int8_t curr_spo2_valid = 0;
static float curr_ratio = 0;
static float curr_correl = 0;


/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define I2C_A_IRQn I2C0_IRQn

/* Priorities at which the tasks are created.  */
#define M2T(X) ((unsigned int)((X)*(configTICK_RATE_HZ/1000.0)))
#define MAX_SAMP_CHARS 9
#define RF_SAMPLES 100
#define RF_SAMPLES_MARGIN 20



Spo2Sensor::Spo2Sensor(uint32_t task_priority) : Sensor(SENSOR_SPO2, EVENT_SPO2_EKG)
{
	if (sensor != nullptr)
		return;
	sensor = this;


	NVIC_EnableIRQ(PORTB_IRQn);

	status = max30102_init(MAX30102_SPO2_MODE) == MAX30102_SUCCESS;
	if(!status) {
		PRINTF("MAX30102 INITIALIZATION ERROR\n");
		return;
	}

	status = max30102_set_led_current(0x24,  MAX30102_LED_PULSE_AMPLITUDE_ADDR_1);
	if(!status) {
		PRINTF("MAX30102 LED1 ERROR\n");
		return;
	}

	status = max30102_set_led_current(0x24,  MAX30102_LED_PULSE_AMPLITUDE_ADDR_2);
	if(!status){
		PRINTF("MAX30102 LED2 ERROR\n");
		return;
	}

	max30102_fifo_configuration_t fifo_conf;
	fifo_conf.val = 0;
	fifo_conf.fifo_a_full = 0xF;
	fifo_conf.fifo_roll_over_en = true;
	fifo_conf.smp_ave = MAX30102_SMP_AVE_4;
	status = max30102_set_fifo_config(&fifo_conf);
	if(!status) {
		PRINTF("MAX30102 FIFO CONFIG ERROR\n");
		return;
	}
	max30102_spo2_configuration_t spo2_conf;
	spo2_conf.val = 0;
	spo2_conf.led_pw = MAX30102_LED_PW_411US_ADC_18_BITS;
	spo2_conf.spo2_sr = MAX30102_SPO2_SAMPLE_RATE_100HZ;
	spo2_conf.spo2_adc_rge = MAX30102_SPO2_ADC_RESOLUTION_4096NA;
	status = max30102_set_spo2_config(&spo2_conf);
	if(!status) {
		PRINTF("MAX30102 SPO2 CONFIG ERROR\n");
		return;
	}
	memset(&max30102_interrupts, 0, sizeof(max30102_interrupts));

	status = xTaskCreate(
			spo2_task, "spo2 task",
			configMINIMAL_STACK_SIZE + 166, nullptr,
			task_priority, &xTaskSpo2) == pdTRUE;
	if (!status) {
		PRINTF("Spo2 task creation failed!.\r\n");
		return;
	}

	status = (xBinarySemaphore = xSemaphoreCreateBinary()) == nullptr;
	if (!status) {
		PRINTF("Spo2 semaphore creation failed!.\r\n");
		return;
	}


	for(int i=0; i < MAX_SAMP_CHARS; i++){
		samp1[i] = 0;
		samp2[i] = 0;
	}

	for(int i=0; i < RF_SAMPLES + RF_SAMPLES_MARGIN;i++){
		ir_led_samples[i] = 0;
		red_led_samples[i] = 0;
	}

	Sensor::set_limits(EVENT_SPO2_EKG, 0.0, 1.0);
}


Spo2Sensor::~Spo2Sensor()
{
	stop_sampling();

	vSemaphoreDelete(xBinarySemaphore);
	xBinarySemaphore = nullptr;

	vTaskDelete(xTaskSpo2);
	xTaskSpo2 = nullptr;
}


void Spo2Sensor::start_sampling()
{
	max30102_trigger_spo2_reads();
	vTaskResume(xTaskSpo2);
}


void Spo2Sensor::stop_sampling()
{
	vTaskSuspend(xTaskSpo2);
}


void spo2_task(void *pvParameters)
{
	while(true){
		xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);

		if (!GPIO_PinRead(GPIOB, BOARD_MAX30102_INT_PIN_PIN)) {
			max30102_get_interrupt_status(&max30102_interrupts);
			interrupt_flag = false;

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
				ir_led_samples[curr_buffer_n_samples] = aux_sample_ir;
				red_led_samples[curr_buffer_n_samples] = aux_sample_red;

				sensor->write_sample(float(aux_sample_ir)/float(0x00000001<<18 - 1));

				curr_buffer_n_samples++;


				//is the buffer full ? (should the accumulated samples be processed?)
				if(curr_buffer_n_samples >= RF_SAMPLES){
					//the red led is switched with the ir led on the board.
					rf_heart_rate_and_oxygen_saturation(red_led_samples, RF_SAMPLES, ir_led_samples,
						&curr_spo2, &curr_spo2_valid, &curr_heart_rate, &curr_hr_valid, &curr_ratio, &curr_correl);

					if(curr_hr_valid && curr_spo2_valid){
						sensor->write_sample(float(curr_heart_rate), EVENT_SPO2_BPM);
						sensor->write_sample(float(curr_spo2)/float(100), EVENT_SPO2_SPO2);
					}
					curr_buffer_n_samples = 0;
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
	}
}


/* PORTB_IRQn interrupt handler */
void GPIOB_IRQHANDLER(void) {
  /* Get pin flags */
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
