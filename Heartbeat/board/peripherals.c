/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Peripherals v9.0
processor: MK64FN1M0xxx12
package_id: MK64FN1M0VLL12
mcu_data: ksdk2_0
processor_version: 9.0.0
board: FRDM-K64F
functionalGroups:
- name: BOARD_InitPeripherals
  UUID: 5e99aa89-113c-4ea7-8360-300e047a8bb3
  called_from_default_init: true
  selectedCore: core0
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
component:
- type: 'system'
- type_id: 'system'
- global_system_definitions:
  - user_definitions: ''
  - user_includes: ''
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

/***********************************************************************************************************************
 * Included files
 **********************************************************************************************************************/
#include "peripherals.h"

/***********************************************************************************************************************
 * BOARD_InitPeripherals functional group
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * GPIOB initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'GPIOB'
- type: 'gpio'
- mode: 'GPIO'
- custom_name_enabled: 'false'
- type_id: 'gpio_5920c5e026e8e974e6dc54fbd5e22ad7'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'GPIOB'
- config_sets:
  - fsl_gpio:
    - enable_irq: 'true'
    - port_interrupt:
      - IRQn: 'PORTB_IRQn'
      - enable_interrrupt: 'noInit'
      - enable_priority: 'false'
      - priority: '0'
      - enable_custom_name: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

static void GPIOB_init(void) {
  /* Make sure, the clock gate for port B is enabled (e. g. in pin_mux.c) */
  /* Interrupt PORTB_IRQn request in the NVIC is not initialized (disabled by default). */
  /* It can be enabled later by EnableIRQ(GPIOB_IRQN);  function call. */
}

/***********************************************************************************************************************
 * I2C0 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'I2C0'
- type: 'i2c'
- mode: 'freertos'
- custom_name_enabled: 'false'
- type_id: 'i2c_2566d7363e7e9aaedabb432110e372d7'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'I2C0'
- config_sets:
  - fsl_i2c:
    - i2c_mode: 'kI2C_Master'
    - clockSource: 'BusInterfaceClock'
    - clockSourceFreq: 'GetFreq'
    - rtos_handle:
      - enable_custom_name: 'true'
      - handle_custom_name: 'I2CA_rtosHandle'
    - i2c_master_config:
      - enableMaster: 'true'
      - enableStopHold: 'false'
      - baudRate_Bps: '100000'
      - glitchFilterWidth: '0'
    - interrupt_priority:
      - IRQn: 'I2C0_IRQn'
      - enable_priority: 'false'
      - priority: '0'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
i2c_rtos_handle_t I2CA_rtosHandle;
const i2c_master_config_t I2C0_config = {
  .enableMaster = true,
  .enableStopHold = false,
  .baudRate_Bps = 100000UL,
  .glitchFilterWidth = 0U
};

static void I2C0_init(void) {
  /* Initialization function */
  I2C_RTOS_Init(&I2CA_rtosHandle, I2C0_PERIPHERAL, &I2C0_config, I2C0_CLK_FREQ);
}

/***********************************************************************************************************************
 * I2S0 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'I2S0'
- type: 'sai'
- mode: 'transfer'
- custom_name_enabled: 'false'
- type_id: 'sai_37a0d4b4ecc2db8ea149dbe2026c6550'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'I2S0'
- config_sets:
  - fsl_sai:
    - mclk_config:
      - masterClockSource: 'kSAI_MclkSourceSysclk'
      - masterClockSourceFreq: 'BOARD_BootClockRUN'
      - masterClockFrequency: '6.144 MHz'
      - init_mclk_config: 'true'
    - sai_master_clock: []
    - usage: 'playback'
    - signal_config:
      - 0:
        - sourceTx: 'Tx'
      - 1:
        - sourceTx: 'Tx'
    - syncSwapI: []
    - bclkTxSetting: []
    - syncTxSetting: []
    - whole:
      - tx_group:
        - sai_transceiver:
          - bitClock:
            - modeM: 'master'
            - bitClockSource: 'kSAI_BclkSourceMclkDiv'
            - bclkPolarityM: 'kSAI_PolarityActiveLow'
            - bclkInputDelayM: 'false'
          - frameSync:
            - modeM: 'master'
            - frameSyncWidthM: '16'
            - frameSyncPolarityM: 'kSAI_PolarityActiveLow'
            - frameSyncEarlyM: 'true'
          - sampleRate_Hz: 'kSAI_SampleRate16KHz'
          - channelMask: 'kSAI_Channel0Mask'
          - serialData:
            - differentFirstWord: 'false'
            - sameDataWordLengthM: 'kSAI_WordWidth16bits'
            - dataOrder: 'kSAI_DataMSB'
            - dataFirstBitShiftedM: '16'
            - dataWordNumM: '2'
            - dataMasked_config:
              - dataMasked_L:
                - 0: 'false'
                - 1: 'false'
                - 2: 'false'
                - 3: 'false'
                - 4: 'false'
                - 5: 'false'
                - 6: 'false'
                - 7: 'false'
                - 8: 'false'
                - 9: 'false'
                - 10: 'false'
                - 11: 'false'
                - 12: 'false'
                - 13: 'false'
                - 14: 'false'
                - 15: 'false'
              - dataMasked_H:
                - 0: 'false'
                - 1: 'false'
                - 2: 'false'
                - 3: 'false'
                - 4: 'false'
                - 5: 'false'
                - 6: 'false'
                - 7: 'false'
                - 8: 'false'
                - 9: 'false'
                - 10: 'false'
                - 11: 'false'
                - 12: 'false'
                - 13: 'false'
                - 14: 'false'
                - 15: 'false'
          - fifo:
            - fifoWatermarkM: '7'
        - transfer_config_group:
          - init_transfer: 'false'
          - transfer:
            - dataSize: '44100'
          - init_callback: 'false'
          - callback_fcn: ''
          - user_data: ''
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
/* I2S0 Tx configuration */
sai_transceiver_t I2S0_Tx_config = {
  .masterSlave = kSAI_Master,
  .bitClock = {
    .bclkSrcSwap = false,
    .bclkSource = kSAI_BclkSourceMclkDiv,
    .bclkPolarity = kSAI_PolarityActiveLow,
    .bclkInputDelay = false
  },
  .frameSync = {
    .frameSyncWidth = 16U,
    .frameSyncPolarity = kSAI_PolarityActiveLow,
    .frameSyncEarly = true,
  },
  .syncMode = kSAI_ModeAsync,
  .channelMask = kSAI_Channel0Mask,
  .startChannel = 0U,
  .endChannel = 0U,
  .channelNums = 1U,
  .serialData = {
    .dataWord0Length = (uint8_t)kSAI_WordWidth16bits,
    .dataWordNLength = (uint8_t)kSAI_WordWidth16bits,
    .dataWordLength = (uint8_t)kSAI_WordWidth16bits,
    .dataOrder = kSAI_DataMSB,
    .dataFirstBitShifted = 16U,
    .dataWordNum = 2U,
    .dataMaskedWord = 0x0U
  },
  .fifo = {
    .fifoWatermark = 7U,
  }
};
sai_master_clock_t I2S0_MCLK_config = {
  .mclkOutputEnable = true,
  .mclkSource = kSAI_MclkSourceSysclk,
  .mclkSourceClkHz = I2S0_MCLK_SOURCE_CLOCK_HZ,
  .mclkHz = I2S0_USER_MCLK_HZ
};
sai_handle_t I2S0_Tx_handle;

static void I2S0_init(void) {
  /* Initialize SAI clock gate */
  SAI_Init(I2S0_PERIPHERAL);
  /* Create the SAI Tx transfer handle */
  SAI_TransferTxCreateHandle(I2S0_PERIPHERAL, &I2S0_Tx_handle, NULL, NULL);
  /* Configures SAI Tx sub-module functionality */
  SAI_TransferTxSetConfig(I2S0_PERIPHERAL, &I2S0_Tx_handle, &I2S0_Tx_config);
  /* Set up SAI Tx bitclock rate by calculation of divider. */
  SAI_TxSetBitClockRate(I2S0_PERIPHERAL, I2S0_TX_BCLK_SOURCE_CLOCK_HZ, I2S0_TX_SAMPLE_RATE, I2S0_TX_WORD_WIDTH, I2S0_TX_WORDS_PER_FRAME);
  /* Initialize SAI master clock */
  SAI_SetMasterClockConfig(I2S0_PERIPHERAL, &I2S0_MCLK_config);
}

/***********************************************************************************************************************
 * UART0 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'UART0'
- type: 'uart'
- mode: 'polling'
- custom_name_enabled: 'false'
- type_id: 'uart_88ab1eca0cddb7ee407685775de016d5'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'UART0'
- config_sets:
  - uartConfig_t:
    - uartConfig:
      - clockSource: 'BusInterfaceClock'
      - clockSourceFreq: 'GetFreq'
      - baudRate_Bps: '115200'
      - parityMode: 'kUART_ParityDisabled'
      - stopBitCount: 'kUART_OneStopBit'
      - txFifoWatermark: '0'
      - rxFifoWatermark: '1'
      - idleType: 'kUART_IdleTypeStartBit'
      - enableTx: 'true'
      - enableRx: 'true'
    - quick_selection: 'QuickSelection1'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const uart_config_t UART0_config = {
  .baudRate_Bps = 115200UL,
  .parityMode = kUART_ParityDisabled,
  .stopBitCount = kUART_OneStopBit,
  .txFifoWatermark = 0U,
  .rxFifoWatermark = 1U,
  .idleType = kUART_IdleTypeStartBit,
  .enableTx = true,
  .enableRx = true
};

static void UART0_init(void) {
  UART_Init(UART0_PERIPHERAL, &UART0_config, UART0_CLOCK_SOURCE);
}

/***********************************************************************************************************************
 * UART3 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'UART3'
- type: 'uart'
- mode: 'freertos'
- custom_name_enabled: 'false'
- type_id: 'uart_88ab1eca0cddb7ee407685775de016d5'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'UART3'
- config_sets:
  - fsl_uart_freertos:
    - uart_rtos_configuration:
      - clockSource: 'BusInterfaceClock'
      - clockSourceFreq: 'GetFreq'
      - baudrate: '115200'
      - parity: 'kUART_ParityDisabled'
      - stopbits: 'kUART_OneStopBit'
      - buffer_size: '2000'
    - interrupt_rx_tx:
      - IRQn: 'UART3_RX_TX_IRQn'
      - enable_priority: 'false'
      - priority: '0'
    - interrupt_err:
      - IRQn: 'UART3_ERR_IRQn'
      - enable_priority: 'false'
      - priority: '0'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
uart_rtos_handle_t UART3_rtos_handle;
uart_handle_t UART3_uart_handle;
uint8_t UART3_background_buffer[UART3_BACKGROUND_BUFFER_SIZE];
uart_rtos_config_t UART3_rtos_config = {
  .base = UART3_PERIPHERAL,
  .baudrate = 115200UL,
  .parity = kUART_ParityDisabled,
  .stopbits = kUART_OneStopBit,
  .buffer = UART3_background_buffer,
  .buffer_size = UART3_BACKGROUND_BUFFER_SIZE
};

static void UART3_init(void) {
  /* UART clock source initialization */
  UART3_rtos_config.srcclk = UART3_CLOCK_SOURCE;
  /* UART rtos initialization */
  UART_RTOS_Init(&UART3_rtos_handle, &UART3_uart_handle, &UART3_rtos_config);
}

/***********************************************************************************************************************
 * ADC0 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'ADC0'
- type: 'adc16'
- mode: 'ADC'
- custom_name_enabled: 'false'
- type_id: 'adc16_7a29cdeb84266e77f0c7ceac1b49fe2d'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'ADC0'
- config_sets:
  - fsl_adc16:
    - adc16_config:
      - referenceVoltageSource: 'kADC16_ReferenceVoltageSourceVref'
      - clockSource: 'kADC16_ClockSourceAsynchronousClock'
      - enableAsynchronousClock: 'true'
      - clockDivider: 'kADC16_ClockDivider8'
      - resolution: 'kADC16_ResolutionSE12Bit'
      - longSampleMode: 'kADC16_LongSampleDisabled'
      - enableHighSpeed: 'false'
      - enableLowPower: 'false'
      - enableContinuousConversion: 'false'
    - adc16_channel_mux_mode: 'kADC16_ChannelMuxA'
    - adc16_hardware_compare_config:
      - hardwareCompareModeEnable: 'false'
    - doAutoCalibration: 'false'
    - offset: '0'
    - trigger: 'true'
    - hardwareAverageConfiguration: 'kADC16_HardwareAverageDisabled'
    - enable_dma: 'false'
    - enable_irq: 'true'
    - adc_interrupt:
      - IRQn: 'ADC0_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'false'
      - priority: '0'
      - enable_custom_name: 'false'
    - adc16_channels_config:
      - 0:
        - enableDifferentialConversion: 'false'
        - channelNumber: 'SE.19'
        - enableInterruptOnConversionCompleted: 'true'
        - channelGroup: '0'
        - initializeChannel: 'true'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
adc16_channel_config_t ADC0_channelsConfig[1] = {
  {
    .channelNumber = 19U,
    .enableDifferentialConversion = false,
    .enableInterruptOnConversionCompleted = true,
  }
};
const adc16_config_t ADC0_config = {
  .referenceVoltageSource = kADC16_ReferenceVoltageSourceVref,
  .clockSource = kADC16_ClockSourceAsynchronousClock,
  .enableAsynchronousClock = true,
  .clockDivider = kADC16_ClockDivider8,
  .resolution = kADC16_ResolutionSE12Bit,
  .longSampleMode = kADC16_LongSampleDisabled,
  .enableHighSpeed = false,
  .enableLowPower = false,
  .enableContinuousConversion = false
};
const adc16_channel_mux_mode_t ADC0_muxMode = kADC16_ChannelMuxA;
const adc16_hardware_average_mode_t ADC0_hardwareAverageMode = kADC16_HardwareAverageDisabled;

static void ADC0_init(void) {
  /* Initialize ADC16 converter */
  ADC16_Init(ADC0_PERIPHERAL, &ADC0_config);
  /* Make sure, that hardware trigger is used */
  ADC16_EnableHardwareTrigger(ADC0_PERIPHERAL, true);
  /* Configure hardware average mode */
  ADC16_SetHardwareAverage(ADC0_PERIPHERAL, ADC0_hardwareAverageMode);
  /* Configure channel multiplexing mode */
  ADC16_SetChannelMuxMode(ADC0_PERIPHERAL, ADC0_muxMode);
  /* Initialize channel */
  ADC16_SetChannelConfig(ADC0_PERIPHERAL, ADC0_CH0_CONTROL_GROUP, &ADC0_channelsConfig[0]);
  /* Enable interrupt ADC0_IRQn request in the NVIC. */
  EnableIRQ(ADC0_IRQN);
}

/***********************************************************************************************************************
 * PIT initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'PIT'
- type: 'pit'
- mode: 'LPTMR_GENERAL'
- custom_name_enabled: 'false'
- type_id: 'pit_a4782ba5223c8a2527ba91aeb2bc4159'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'PIT'
- config_sets:
  - fsl_pit:
    - enableRunInDebug: 'false'
    - timingConfig:
      - clockSource: 'BusInterfaceClock'
      - clockSourceFreq: 'BOARD_BootClockRUN'
    - channels:
      - 0:
        - channel_id: 'CHANNEL_0'
        - channelNumber: '0'
        - enableChain: 'false'
        - timerPeriod: '5555'
        - startTimer: 'false'
        - enableInterrupt: 'false'
        - interrupt:
          - IRQn: 'PIT0_IRQn'
          - enable_interrrupt: 'enabled'
          - enable_priority: 'false'
          - priority: '0'
          - enable_custom_name: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const pit_config_t PIT_config = {
  .enableRunInDebug = false
};

static void PIT_init(void) {
  /* Initialize the PIT. */
  PIT_Init(PIT_PERIPHERAL, &PIT_config);
  /* Set channel 0 period to 5.555 ms (333300 ticks). */
  PIT_SetTimerPeriod(PIT_PERIPHERAL, PIT_CHANNEL_0, PIT_CHANNEL_0_TICKS);
}

/***********************************************************************************************************************
 * Initialization functions
 **********************************************************************************************************************/
void BOARD_InitPeripherals(void)
{
  /* Initialize components */
  GPIOB_init();
  I2C0_init();
  I2S0_init();
  UART0_init();
  UART3_init();
  ADC0_init();
  PIT_init();
}

/***********************************************************************************************************************
 * BOARD_InitBootPeripherals function
 **********************************************************************************************************************/
void BOARD_InitBootPeripherals(void)
{
  BOARD_InitPeripherals();
}
