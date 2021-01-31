/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Peripherals v8.0
processor: MK64FN1M0xxx12
package_id: MK64FN1M0VLL12
mcu_data: ksdk2_0
processor_version: 8.0.1
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
  /* It can be enabled later by EnableIRQ(PORTB_IRQn); function call. */
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
 * Initialization functions
 **********************************************************************************************************************/
void BOARD_InitPeripherals(void)
{
  /* Initialize components */
  GPIOB_init();
  I2C0_init();
  I2S0_init();
}

/***********************************************************************************************************************
 * BOARD_InitBootPeripherals function
 **********************************************************************************************************************/
void BOARD_InitBootPeripherals(void)
{
  BOARD_InitPeripherals();
}
