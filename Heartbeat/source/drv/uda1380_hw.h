/*
 * uda1380.h
 *
 *  Created on: 30 Jan 2021
 *      Author: grein
 */

#ifndef DRV_UDA1380_HW_H_
#define DRV_UDA1380_HW_H_
#include <stdint.h>
#include <stdbool.h>

//Address
//#define A1_HIGH	//Comment if A1 pin is not high
#define UDA1380_I2C_ADDR_A1_HIGH  0x1A
#define UDA1380_I2C_ADDR_A1_LOW   0x18
#ifdef A1_HIGH
#define UDA1380_I2C_ADDR UDA1380_I2C_ADDR_A1_HIGH
#else
#define UDA1380_I2C_ADDR UDA1380_I2C_ADDR_A1_LOW
#endif

typedef enum
{
	//System Settings
	UDA1380_CLK_ADDR = 0x00,
	UDA1380_IBUSCFG_ADDR = 0x01,
	UDA1380_PWRCTR_ADDR = 0x02,
	UDA1380_AMIXCFG_ADDR = 0x03,
	UDA1380_HPHAMPCFG_ADDR = 0x04,
	//Interpolation filter
	UDA1380_MVOLCTR_ADDR = 0x10,
	UDA1380_MIXVOLCTR_ADDR = 0x11,
	UDA1380_MODESEL_ADDR = 0x12,
	UDA1380_DEEMP_ADDR = 0x13,
	UDA1380_MIX_ADDR = 0x14,
	//Decimator
	UDA1380_DECVOLCTR_ADDR = 0x20,
	UDA1380_PGA_ADDR = 0x21,
	UDA1380_ADC_ADDR = 0x22,
	UDA1380_AGC_ADDR = 0x23,
	//Software Reset
	UDA1380_RESET_ADDR = 0x7F,
	//Headphone Driver and Interpolation Filter status bits
	UDA1380_IFILTSTAT_ADDR = 0x18,
	//Decimator status bits
	UDA1380_DECSTAT_ADDR = 0x28
}uda1380_addr_t;

//Register masks
//Clock Settings (UDA1380_CLK_ADDR)
#define EN_ADC 0x0800 //Logic 0: clock to ADC disabled. Logic 1: clock to ADC running. Default value 0.
#define EN_DEC 0x0400 //Logic 0: clock to the decimator disabled. Logic 1: clock to the decimator running. Default value 1
#define EN_DAC 0x0200 //Logic 0: clock to FSDAC disabled. Logic 1: clock to the FSDAC running. Default value 0.
#define EN_INT 0x0100 //Logic 0: clock to the interpolator disabled. Logic 1: clock to the interpolator running. Default value 1.
#define ADC_CLK 0x0020 //Logic 0: SYSCLK is used. Logic 1: WSPLL is used. Default value 0.
#define DAC_CLK 0x0010 //Logic 0: SYSCLK is used. Logic 1: WSPLL is used. Default value 0.
#define SYS_DIV 0x000C //See below
#define PLL 0x0003 //See below
//--> SYS_DIV
// Selects the proper division factor for the SYSCLK input in such a way that a128fs clock will be generated
// from the SYSCLK clock signal. The 128fs clock is needed for clocking the decimator and interpolator.
// Default value SYS_DIV_256FS.
#define SYS_DIV_256FS (0x00 << 2)
#define SYS_DIV_384FS (0x01 << 2)
#define SYS_DIV_512FS (0x02 << 2)
#define SYS_DIV_768FS (0x03 << 2)
//--> PLL
// Selects the WSPLL input frequency range. These set the proper divider setting for the WSPLL. The input is the
// WSI signal, the output inside the IC is a 128fs and a 256fs clock. Default value "PLL_25K_50K"
#define PLL_6K25_12K5 (0x00 << 0)
#define PLL_12K5_25K (0x01 << 0)
#define PLL_25K_50K (0x02 << 0)
#define PLL_50K_100K (0x03 << 0)

// I2S-bus input and output settings (UDA1380_IBUSCFG_ADDR)
#define SFORI 0x0700 //See below
#define SEL_SOURCE 0x0040 //source digital output interface mode set to -> Logic 0:  decimator. Logic 1: digital mixer output. Default value 0.
#define SSIM 0x0010 //mode of digital output interface is set to -> Logic 0:  slave. Logic 1:master. Default value 0.
#define SFORO 0x0007 //See below
//--> SFORI
// Digital data input formats. A 3-bit value to select the digital input data
// format (DATAI input). Default value "SFORI_I2S_BUS"
//--> SFORO
// Digital data output formats. A 3-bit value to set the digital data output format
// (on pin DATAO). Default value "SFORO_i2S_BUS"
#define SFORX_I2S_BUS 0x00
#define SFORX_LSB_16 0x01
#define SFORX_LSB_18 0x02
#define SFORX_LSB_20 0x03
#define SFORX_LSB_24 0x04
#define SFORX_MSB 0x05
#define SFORI_I2S_BUS (SFORX_I2S_BUS << 8)
#define SFORI_LSB_16 (SFORX_LSB_16 << 8)
#define SFORI_LSB_18 (SFORX_LSB_18 << 8)
#define SFORI_LSB_20 (SFORX_LSB_20 << 8)
//#define SFORI_LSB_24 (SFORX_LSB_24 << 8) // NOT AVAILABLE
#define SFORI_MSB (SFORX_MSB << 8)
#define SFORO_I2S_BUS (SFORX_I2S_BUS << 0)
#define SFORO_LSB_16 (SFORX_LSB_16 << 0)
#define SFORO_LSB_18 (SFORX_LSB_18 << 0)
#define SFORO_LSB_20 (SFORX_LSB_20 << 0)
#define SFORO_LSB_24 (SFORX_LSB_24 << 0)
#define SFORO_MSB (SFORX_MSB << 0)

//Power control settings (UDA1380_PWRCTR_ADDR)
#define PON_PLL 0x8000 //Logic 0: power-off; Logic 1: power-on. Default value 0.
#define PON_HP 0x2000 //Logic 0: headphone driver is powered-off; Logic 1: headphone driver is powered-on. Default value 0.
#define PON_DAC 0x0400 //Logic 0: DAC is powered-off; Logic 1: DAC is powered-on. Default value 0.
#define PON_BIAS 0x0100 //ADC, AVC and FSDAC bias circuits are -> Logic 0: powered-off; Logic 1: Power-on. Default value 0.
#define EN_AVC 0x0080 //Logic 0: Analog mixer is disabled; Logic 1: analog mixer is enabled. Default value 0.
#define PON_AVC 0x0040 //Logic 0: analog mixer powered-off; logic 1: analog mixer powered-on. Default value 0.
#define PON_LNA 0x0010 //Logic 0: LNA and SDC are powered-off; Logic 1: LNA and SDC are powered-on. Default value 0.
#define PON_PGAL 0x0008  //Logic 0: left PGA is powered-off; Logic 1: left PGA is powered-on. Default value 0.
#define PON_ADCL 0x0004 //Logic 0: left ADC is powered-off; Logic 1: left ADC is powered-on. Default value 0.
#define PON_PGAR 0x0002 //Logic 0: right PGA is powered-off; Logic 1: right PGA is powered-on. Default value 0.
#define PON_ADCR 0x0001 //Logic 0: right ADC is powered-off; when this bit is logic 1: right ADC is powered-on. Default value 0

//Analog mixer settings (UDA1380_AMIXCFG_ADDR)
#define AVCL 0x3F00 //Value to program the left master volume attenuation. The range is from +16.5 to −48 and −∞ dB in steps of 1.5 dB. Default −∞
#define AVCR 0x003F //Value to program the left master volume attenuation. The range is from +16.5 to −48 and −∞ dB in steps of 1.5 dB. Default −∞
//--> dB Macro
#define AVCx_dB_MAX 16.5
#define AVCx_dB_MIN -48
#define AVCx_dB_step 1.5
#define AVCx_dB(x)  (((uint16_t)((AVCx_dB_MAX-((x)<AVCx_dB_MIN?AVCx_dB_MIN:(x)>AVCx_dB_MAX?AVCx_dB_MAX:(x)))/AVCx_dB_step))&0x3F)
#define AVCL_dB(x) (AVCx_dB(x) << 8)
#define AVCR_dB(x) (AVCx_dB(x) << 0)

//Headphone amplifier settings (UDA1380_HPHAMPCFG_ADDR)
#define EN_SCP 0x0002 //Logic 0: short-circuit protection is disabled. Logic 1: short-circuit protection is enabled. Default value 1.

//Master volume control (UDA1380_MVOLCTR_ADDR)
#define MVCR 0xFF00 //Value to program the right channel volume attenuation. The range is from 0 to −78 dB and −∞ dB in steps of 0.25 dB. Default value 0dB
#define MVCL 0x00FF //Value to program the left channel volume attenuation. The range is from 0 to −78 dB and −∞ dB in steps of 0.25 dB. Default value 0dB
//-->dB Macro
#define MVCx_dB_MAX 0
#define MVCx_dB_MIN -78
#define MVCx_dB_step 0.25
#define MVCx_dB(x)  (((uint16_t)((MVCx_dB_MAX-((x)<MVCx_dB_MIN?MVCx_dB_MIN:(x)>MVCx_dB_MAX?MVCx_dB_MAX:(x)))/MVCx_dB_step))&0xFF)
#define MVCR_dB(x) (MVCx_dB(x) << 8)
#define MVCL_dB(x) (MVCx_dB(x) << 0)

//Mixer volume control (UDA1380_MIXVOLCTR_ADDR)
#define VC2 0xFF00 //Value to program the channel 2 volume attenuation. The range is 0 to −72 dB and −∞ dB in steps of 0.25 dB. Default −∞
#define VC1 0x00FF //Value to program the channel 1 volume attenuation. The range is 0 to −72 dB and −∞ dB in steps of 0.25 dB. Default 0
//-->dB Macro
#define VCx_dB_MAX 16.5
#define VCx_dB_MIN -48
#define VC1_dB(x) (MVCR_dB(x))
#define VC2_dB(x) (MVCL_dB(x))

// Mode, bass boost and treble (UDA1380_MODESEL_ADDR)
#define FMINMAX 0xC000 //See below
#define TRL 0x3000 //See below
#define BBL 0x0F00 //See below
#define TRR 0x0030 //See below
#define BBR 0x000F //See below
//--> FMINMAX
// value to program the mode of the sound processing filters of bass boost and treble.
#define FMINMAX_FLAT 0x00
#define FMINMAX_MIN 0x01
#define FMINMAX_MAX 0x03
//--> TRL / TRR
// Value to program the mode of the sound processing filter of treble. The used setting depends on FMINMAX. Default TRL_A
#define TRx_0 0x00  //Flat: 0dB. Min/Max: 0dB
#define TRx_2 0x01  //Flat: 0db. Min/Max: 2db
#define TRx_4 0x02  //Flat: 0db. Min/Max: 4db
#define TRx_6 0x03  //Flat: 0db. Min/Max: 6db
#define TRL_0 (TRx_0 << 12)
#define TRL_2 (TRx_2 << 12)
#define TRL_4 (TRx_4 << 12)
#define TRL_6 (TRx_6 << 12)
#define TRR_0 (TRx_0 << 4)
#define TRR_2 (TRx_2 << 4)
#define TRR_4 (TRx_4 << 4)
#define TRR_6 (TRx_6 << 4)
//--> BBL / BBR
// value to program the bass boost setting, which can be set
// for left and right independently. The used set depends on the bits M1 and M0. Default value BBx_MIN0dB_MAX0dB
#define BBx_MIN0dB_MAX0dB 0x00 //Flat: 0dB. Min: 0dB. Max: 0dB
#define BBx_MIN2dB_MAX2dB 0x01 //Flat: 0dB. Min: 2dB. Max: 2dB
#define BBx_MIN4dB_MAX4dB 0x02 //Flat: 0dB. Min: 4dB. Max: 4dB
#define BBx_MIN6dB_MAX6dB 0x03 //Flat: 0dB. Min: 6dB. Max: 6dB
#define BBx_MIN8dB_MAX8dB 0x04 //Flat: 0dB. Min: 8dB. Max: 8dB
#define BBx_MIN10dB_MAX10dB 0x05 //Flat: 0dB. Min: 10dB. Max: 10dB
#define BBx_MIN12dB_MAX12dB 0x06 //Flat: 0dB. Min: 12dB. Max: 12dB
#define BBx_MIN14dB_MAX14dB 0x07 //Flat: 0dB. Min: 14dB. Max: 14dB
#define BBx_MIN16dB_MAX16dB 0x08 //Flat: 0dB. Min: 16dB. Max: 16dB
#define BBx_MIN18dB_MAX18dB 0x09 //Flat: 0dB. Min: 18dB. Max: 18dB
#define BBx_MIN18dB_MAX20dB 0x0A //Flat: 0dB. Min: 18dB. Max: 20dB
#define BBx_MIN18dB_MAX22dB 0x0B //Flat: 0dB. Min: 18dB. Max: 22dB
#define BBx_MIN18dB_MAX24dB 0x0C //Flat: 0dB. Min: 18dB. Max: 24dB
#define BBL_MIN0dB_MAX0dB (BBx_MIN0dB_MAX0dB << 8)
#define BBL_MIN2dB_MAX2dB (BBx_MIN2dB_MAX2dB << 8)
#define BBL_MIN4dB_MAX4dB (BBx_MIN4dB_MAX4dB << 8)
#define BBL_MIN6dB_MAX6dB (BBx_MIN6dB_MAX6dB << 8)
#define BBL_MIN8dB_MAX8dB (BBx_MIN8dB_MAX8dB << 8)
#define BBL_MIN10dB_MAX10dB (BBx_MIN10dB_MAX10dB << 8)
#define BBL_MIN12dB_MAX12dB (BBx_MIN12dB_MAX12dB << 8)
#define BBL_MIN14dB_MAX14dB (BBx_MIN14dB_MAX14dB << 8)
#define BBL_MIN16dB_MAX16dB (BBx_MIN16dB_MAX16dB << 8)
#define BBL_MIN18dB_MAX18dB (BBx_MIN18dB_MAX18dB << 8)
#define BBL_MIN18dB_MAX20dB (BBx_MIN18dB_MAX20dB << 8)
#define BBL_MIN18dB_MAX22dB (BBx_MIN18dB_MAX22dB << 8)
#define BBL_MIN18dB_MAX24dB (BBx_MIN18dB_MAX24dB << 8)
#define BBR_MIN0dB_MAX0dB (BBx_MIN0dB_MAX0dB << 0)
#define BBR_MIN2dB_MAX2dB (BBx_MIN2dB_MAX2dB << 0)
#define BBR_MIN4dB_MAX4dB (BBx_MIN4dB_MAX4dB << 0)
#define BBR_MIN6dB_MAX6dB (BBx_MIN6dB_MAX6dB << 0)
#define BBR_MIN8dB_MAX8dB (BBx_MIN8dB_MAX8dB << 0)
#define BBR_MIN10dB_MAX10dB (BBx_MIN10dB_MAX10dB << 0)
#define BBR_MIN12dB_MAX12dB (BBx_MIN12dB_MAX12dB << 0)
#define BBR_MIN14dB_MAX14dB (BBx_MIN14dB_MAX14dB << 0)
#define BBR_MIN16dB_MAX16dB (BBx_MIN16dB_MAX16dB << 0)
#define BBR_MIN18dB_MAX18dB (BBx_MIN18dB_MAX18dB << 0)
#define BBR_MIN18dB_MAX20dB (BBx_MIN18dB_MAX20dB << 0)
#define BBR_MIN18dB_MAX22dB (BBx_MIN18dB_MAX22dB << 0)
#define BBR_MIN18dB_MAX24dB (BBx_MIN18dB_MAX24dB << 0)

//Master mute, channel de-emphasis and mute (UDA1380_DEEMP_ADDR)
#define MTM 0x4000 // Logic 0: no soft mute of master. Logic 1: soft mute of master. Default value 1.
#define MT2 0x0800 //After enabling the mixer, bit MT2 must be set to logic 0.  Logic 0: no soft mute of channel 2. Logic 1: soft mute of channel 2
#define DE2 0x0700 //See below
#define MT1 0x0008 //After enabling the mixer, bit MT2 must be set to logic 0.  Logic 0: no soft mute of channel 2. Logic 1: soft mute of channel 2
#define DE1 0x0007 //See below
//--> DE1/DE2
// value to enable the digital de-emphasis filter for channel 1 / 2. Default value DEx_OFF
#define DEx_OFF 0x00
#define DEx_32K 0x01
#define DEx_44K1 0x02
#define DEx_48K 0x03
#define DEx_96K 0x04
#define DE1_OFF (DEx_OFF << 8 )
#define DE1_32K (DEx_32K << 8 )
#define DE1_44K1 (DEx_44K1 << 8 )
#define DE1_48K (DEx_48K << 8 )
#define DE1_96K (DEx_96K << 8 )
#define DE2_OFF (DEx_OFF << 0 )
#define DE2_32K (DEx_32K << 0 )
#define DE2_44K1 (DEx_44K1 << 0 )
#define DE2_48K (DEx_48K << 0 )
#define DE2_96K (DEx_96K << 0 )

//Mixer, silence detector and oversampling settings (UDA1380_MIX_ADDR)
#define DA_POL_INV 0x8000// Logic 0: DAC output not inverted. Logic 1: DAC output inverted. Default value 0.
#define SEL_NS 0x4000// Logic 0: select 3rd-order noise shaper. Logic 1: select 5th-order noise shaper. Default value 0.
#define MIX 0x3000// See below
#define SILENCE 0x0080 //Read datasheet
#define SDET_ON 0x0040 // Logic 0: silence detection circuit disabled. Logic 1: silence detection circuit enabled. Default value 0
#define SD_VALUE 0x0030 // See below
#define OS 0x0003 // See below
//--> MIX
// value to select the digital mixer settings inside the interpolation filter. Default value 0
#define MIX_NOMIX (0x00 << 12) //No mixing
#define MIX_MODE1 (0x02 << 12) //volume of channel 1 is forced to 0 dB and volume of channel 2 is forced to −∞ dB
#define MIX_MODE2 (0x01 << 12) //Mixing is done before the sound processing. Read datasheet
#define MIX_MODE3 (0x03 << 12) //mixing is done after the sound processing: input signals are automatically scaled in
//order to prevent clipping during adding
//--> SD
// value to program the silence detector, the number of ‘ZERO’ samples counted before the silence
// detector signals whether there has been digital silence.
#define SD_3200SAMP (0x00 << 4)
#define SD_4800SAMP (0x01 << 4)
#define SD_9600SAMP (0x10 << 4)
#define SD_19200SAMP (0x11 << 4)
//--> OS
// Value to select the oversampling input mode. Default value 00, see Table 50.
#define OS_SS (0x00 << 0) //single-speed input is normal input; mixing possible; default
#define OS_DS (0x01 << 0) //double-speed input is after first half-band; no mixing possible
#define OS_QS (0x10 << 0) //quad-speed input is in front of noise shaper; no mixing possible

//Decimator volume control (UDA1380_DECVOLCTR_ADDR)
#define ML_DEC 0xFF00 //Value to program the gain of the decimator for left. The ranges are +24 to −63.5 dB and −∞ dB in steps of 0.5 dB.
#define MR_DEC 0x00FF //Value to program the gain of the decimator for right. The ranges are +24 to −63.5 dB and −∞ dB in steps of 0.5 dB.
//->dB Macro
#define Mx_DEC_dB_MAX 24
#define Mx_DEC_dB_MIN -63.5
#define Mx_DEC_dB_step 0.5
#define Mx_DEC_dB(x)  (((uint16_t)((Mx_DEC_dB_MAX-((x)<Mx_DEC_dB_MIN?Mx_DEC_dB_MIN:(x)>Mx_DEC_dB_MAX?Mx_DEC_dB_MAX:(x)))/Mx_DEC_dB_step))&0xFF)
#define ML_DEC_dB(x) (Mx_DEC_dB(x) << 8)
#define MR_DEC_dB(x) (Mx_DEC_dB(x) << 0)

//PGA settings and mute (UDA1380_PGA_ADDR)
#define MT_ADC  0x8000
#define PGA_GAINCTRL_R 0x0F00 //Value to program the gain of the input amplifier. Gain range from 0 to 24 dB in steps of 3 dB. Default value 0000
#define PGA_GAINCTRL_L 0x000F //Value to program the gain of the input amplifier. Gain range from 0 to 24 dB in steps of 3 dB. Default value 0000
//-->dB Macro
#define PGA_GAINCTRLRx_dB_MAX 24
#define PGA_GAINCTRLRx_dB_MIN 0
#define PGA_GAINCTRLRx_dB_STEP 3
#define PGA_GAINCTRLRx_dB(x) (((uint16_t)((PGA_GAINCTRLRx_dB_MAX-((x)<PGA_GAINCTRLRx_dB_MIN?PGA_GAINCTRLRx_dB_MIN:(x)>PGA_GAINCTRLRx_dB_MAX?PGA_GAINCTRLRx_dB_MAX:(x)))/PGA_GAINCTRLRx_dB_STEP))&0x0F)
#define PGA_GAINCTRLRR_dB(x) (PGA_GAINCTRLRx_dB(x) << 8)
#define PGA_GAINCTRLRL_dB(x) (PGA_GAINCTRLRx_dB(x) << 0)

//ADC Settings (UDA1380_ADC_ADDR)
#define ADCPOL_INV  0x1000 //Logic 0: polarity of ADC non-inverting. Logic 1: polarity of ADC inverting. Default value 0.
#define VGA_CTRL 0x0F00 //See below
#define SEL_LNA 0x0008//Logic 0: select line input. Logic 1: select LNA for the left ADC. Default value 0.
#define SEL_MIC 0x0004 //Logic 0: select right channel ADC. Logic 1: select left channel ADC (for instance for microphone input). Default value 0.
#define SKIP_DCFIL 0x002 //Logic 0: DC filter enabled. Logic 1: DC filter bypassed. Default value 1.
#define EN_DCFIL 0x0001 //Logic 0: DC filter disabled. Logic 1: DC filter enabled. Default value 0.
//-->VGA_CTRL
// value to program the gain of the LNA in the microphone input channel. The range is 0 to 30 dB in steps of 2 dB.
//-->dB Macro
#define VGA_CTRL_dB_MAX 30
#define VGA_CTRL_dB_MIN 0
#define VGA_CTRL_dB_STEP 2
#define VGA_CTRL_dB(x) ((((uint16_t)((((x) < VGA_CTRL_dB_MIN?VGA_CTRL_dB_MIN:(x)>VGA_CTRL_dB_MAX?VGA_CTRL_dB_MAX:(x)) - VGA_CTRL_dB_MIN)/VGA_CTRL_dB_STEP))&0x0F) << 8)

//AGC Settings (UDA1380_AGC_ADDR)
#define AGC_TIME 0x0700 // See Below
#define AGC_LEVEL 0x000C // See Below
#define AGC_EN 0x0001 //Logic 0: AGC off, manual gain control via the left and right decimator volume control.
// Logic 1: AGC enabled, with manual microphone gain setting via VGA. Default value 0
//-->AGC_TIME
// value to set the AGC time constants, being the
// attack and decay time constants. Read Datasheet
#define AGC_TIME_A (0x00 << 8)
#define AGC_TIME_B (0x01 << 8)
#define AGC_TIME_C (0x02 << 8)
#define AGC_TIME_D (0x03 << 8)
#define AGC_TIME_E (0x04 << 8)
#define AGC_TIME_F (0x05 << 8)
#define AGC_TIME_G (0x06 << 8)
#define AGC_TIME_H (0x07 << 8)
//->AGC_LEVEL
// value to set the AGC target level. Default value 00
#define AGC_LEVEL_N5D5 (0x00 << 2) // AGC TARGET LEVEL VALUE -5.5 dBFS
#define AGC_LEVEL_N8 (0x01 << 2) // AGC TARGET LEVEL VALUE -8 dBFS
#define AGC_LEVEL_N11D5 (0x10 << 2) // AGC TARGET LEVEL VALUE -11.5 dBFS
#define AGC_LEVEL_N14 (0x11 << 2) // AGC TARGET LEVEL VALUE -14 dBFS

//Headphone driver and interpolation filter (read-out) (UDA1380_IFILTSTAT_ADDR)
#define HP_STCTV 0x0400 //Logic 0: headphone driver is not short-circuit protected. Logic 1: headphone driver short-circuit protection is activated.
#define HP_STCTL 0x0200 //Logic 0: left channel headphone driver is not short-circuit protected. Logic 1: left channel headphone driver short-circuit protection is activated
#define MP_STCTR 0x0100 //Logic 0: right channel headphone driver is not short-circuit protected. Logic 1: right channel headphone driver short-circuit protection is activated
#define SDETR2 0x0040 //Logic 0: interpolator on channel 2 right input has detected no silence. Logic 1: interpolator on channel 2 right input has detected silence.
#define SDETL2 0x0020 //Logic 0: interpolator on channel 2 left input has detected no silence. Logic 1: interpolator on channel 2 left input has detected silence.
#define SDTER1 0x0010 //Logic 0: interpolator on channel 1 right input has detected no silence. Logic 1: interpolator on channel 1 right input has detected silence.
#define SDETL1 0x0008 //Logic 0: interpolator on channel 1 left input has detected no silence. Logic 1: interpolator on channel 1 left input has detected silence.
#define MUTE_STATE_M 0x0004 //Logic 0: interpolator is not muted. Logic 1: interpolator is muted.
#define MUTE_STATE_CH2 0x0002 //Logic 0: interpolator channel 2 is not muted. Lgic 1: interpolator channel 2 is muted
#define MUTE_STATE_CH1 0x0001 //Logic 0: interpolator channel 1 is not muted. Lgic 1: interpolator channel 1 is muted

//Decimator read-out (UDA1380_DECSTAT_ADDR)
#define AGC_STAT 0x0010 //Logic 0: AGC gain <8 dB. Logic 1: AGC gain >=8 dB.
#define MIT_ADC_STAT 0x0004 //Logic 0: decimator has not muted. Logic 1: decimator has muted.
#define OVERFLOW 0x0001 //Logic 0: no overflow detected (read-out). Logic 1: overflow detected (read-out).

//Default configuration
//Enable all power
#define UDA1380_PWRCTR_INIT_VALUE  (PON_PLL | PON_HP | PON_DAC | PON_BIAS | EN_AVC | PON_AVC | PON_LNA | PON_PGAL | PON_ADCL | PON_PGAR | PON_ADCR)
#define UDA1380_PWRCTR_INIT_MASK (PON_PLL | PON_HP | PON_DAC | PON_BIAS | EN_AVC | PON_AVC | PON_LNA | PON_PGAL | PON_ADCL | PON_PGAR | PON_ADCR)
//Enable all clocks, WSPLL was enabled, ADC and DAC clocks from WSPLL. PLL Config for 12.5 to 25 KHz
#define UDA1380_CLK_INIT_VALUE (EN_ADC | EN_DEC | EN_DAC | EN_INT | ADC_CLK | DAC_CLK | PLL_12K5_25K)
#define UDA1380_CLK_INIT_MASK (EN_ADC | EN_DEC | EN_DAC | EN_INT | ADC_CLK | DAC_CLK | PLL)
//Use digital mixer, I2S format
#define UDA1380_IBUSCFG_INIT_VALUE (SFORI_I2S_BUS | SFORO_I2S_BUS)
#define UDA1380_IBUSCFG_INIT_MASK (SFORI | SFORO)
//Analog Mixer, Maximum Gain
#define UDA1380_AMIXCFG_INIT_VALUE (AVCL_dB(AVCx_dB_MAX) | AVCR_dB(AVCx_dB_MAX))
#define UDA1380_AMIXCFG_INIT_MASK (AVCL | AVCR)
//Enable Short Circuit Protection
#define UDA1380_HPHAMPCFG_INIT_VALUE EN_SCP
#define UDA1380_HPHAMPCFG_INIT_MASK EN_SCP
//Master Volume, Maximum
#define UDA1380_MVOLCTR_INIT_VALUE (MVCL_dB(MVCx_dB_MAX) | MVCR_dB(MVCx_dB_MAX))
#define UDA1380_MVOLCTR_INIT_MASK (MVCL | MVCR)
//Mix Volume, Maximum
#define UDA1380_MIXVOLCTR_INIT_VALUE (VC1_dB(VCx_dB_MAX) | VC2_dB(VCx_dB_MAX))
#define UDA1380_MIXVOLCTR_INIT_MASK (VC1 | VC2)
//Mode, all flat
#define UDA1380_MODESEL_INIT_VALUE FMINMAX_FLAT
#define UDA1380_MODESEL_INIT_MASK FMINMAX
//Disable Mute, Disable de-emphasis
#define UDA1380_DEEMP_INIT_VALUE (DE1_OFF | DE2_OFF)
#define UDA1380_DEEMP_INIT_MASK (MTM | MT2 | MT1 | DE2 | DE1)
//Do not invert polarity, Mixer off, everything off
#define UDA1380_MIX_INIT_VALUE MIX_NOMIX
#define UDA1380_MIX_INIT_MASK (DA_POL_INV | MIX)
//ADC Decimator volume, Maximum
#define UDA1380_DECVOLCTR_INIT_VALUE (ML_DEC_dB(Mx_DEC_dB_MAX) | MR_DEC_dB(Mx_DEC_dB_MAX))
#define UDA1380_DECVOLCTR_INIT_MASK (ML_DEC | MR_DEC)
//No muting, all to MAX
#define UDA1380_PGA_INIT_VALUE (PGA_GAINCTRLRR_dB(PGA_GAINCTRLRx_dB_MAX) | PGA_GAINCTRLRL_dB(PGA_GAINCTRLRx_dB_MAX))
#define UDA1380_PGA_INIT_MASK (MT_ADC | PGA_GAINCTRL_R | PGA_GAINCTRL_L)
//Non Invert, maximum gain, DC Filter Enabled
#define UDA1380_ADC_INIT_VALUE  (VGA_CTRL_dB(VGA_CTRL_dB_MAX) |SKIP_DCFIL)
#define UDA1380_ADC_INIT_MASK (ADCPOL_INV | VGA_CTRL | SKIP_DCFIL)

#endif /* DRV_UDA1380_HW_H_ */
