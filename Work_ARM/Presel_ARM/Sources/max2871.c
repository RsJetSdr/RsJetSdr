/*
 * max2871.c
 *
 *  Created on: 28 февр. 2017 г.
 *      Author: Piton
 */

#include "events.h"
#include "MAX2871.h"

void OutPLL(uint32_t data);




/*
 * Fref = 50 MHz
 * Fpfd = 25 MHz
 * Fstep = 1 MHz
 * Fout = 4500 MHz
 */
uint32_t  Regs_Max2870[6]={
		0x005A0000,
		0x200080C9,
		0x00009E42,
		0x0000000B,
		0x618F403C,
		0x00400005
};


uint32_t  Regs_MAX2871[6]={
	R5_MAX2871(
		0,//VAS_DLY
		PLL_ON,//SDPLL
		AUTO_INT_ON,//F01
		LD_PIN_MODE_HIGH,//LD
		MUX_OUT,   //MUXOUT
		0,//ADCS Normal Mode
		0 //ADCM ADC Disabled
		),

	R4_MAX2871(
		0,//SDLDO Enable LDO
		0,//SDDIV Enable N_DIV
		0,//SDREF Enable R_DIV
		520, //BS Band Select 50 KHz
		1,   //FB Fundamental
		0,//DIVA
		0,//SDVCO Enable VCO
		1,//MTLD
		0,//BDIV
		RF_OFF,//RFB_EN
		OUTPUT_POWER_M4DBM,//BPWR
		RF_ON,//RFA_EN
		OUTPUT_POWER_M4DBM //APWR
		),
 //VAS VCO Auto Select
	R3_MAX2871(
		0,			 //nVCO  Manual   Select VCO
	    VAS_EN,      //VCO Auto Select Enable
		VAS_TEMP_DI, //VAS temperature compensation
		CSM_OFF,		 //Cycle Slip Mode Disable
		0,//MUTEDEL
		0,//CDM
		1 //CDIV
		),

	R2_MAX2871(
		1,//LDS
		0,//SDN
		MUX_OUT,//MUXOUT
		DOUBLER,//REFx2
		0,//RDIV2
		2,//R_DIV
		1,//D_BUF
		CHARGE_PUMP_CURRENT_5_12MA,//ICP
		MODE_FRAC,	//LDF
		0,	//LDP
		1,	//POL
		0,	//SHDN
		0,	//TRI
		0	//RST
		),

	R1_MAX2871(
		1,	//CPL
		0,	//CPT Normal Mode
		1,	//
		F_PFD	//MOD
		),

	R0_MAX2871(
		MODE_FRAC, //if mode Frac
		180,//N_CNT
		1//FRAC
		)
};

/*
uint16_t SetFreqMHz_Frac(uint16_t target_freq,Pw_out_t Pw)
{

		uint32_t RFdiv = 0;
		uint32_t pwen =  Pw ? 1:0;
		uint32_t pw = Pw ? Pw-1: 0 ;
	    uint16_t vco_freq = (target_freq > MAX_VCO_FREQ) ? MAX_VCO_FREQ : target_freq ;

	    while (vco_freq < MIN_VCO_FREQ)
	       {
	           vco_freq *= 2;
	           RFdiv++;
	       }

	uint32_t  reg=R4_MAX2871(
			0,//SDLDO Enable LDO
			0,//SDDIV Enable N_DIV
			0,//SDREF Enable R_DIV
			500, //BS Band Select 50 KHz
			1,   //FB Fundamental
			RFdiv,//DIVA
			0,//SDVCO Enable VCO
			1,//MTLD
			0,//BDIV
			RF_OFF,//RFB_EN
			OUTPUT_POWER_M4DBM,//BPWR
			pwen,		  //RFA_EN
			pw  	  //APWR
			);

	OutPLL(reg);


	    reg=  R0_MAX2871(
	    		MODE_FRAC,
				vco_freq/F_PFD,	//N_CNT
				vco_freq%F_PFD	//FRAC
	    		);
	OutPLL(reg);
    return vco_freq;
}
*/




uint32_t SetFreqMHz(uint16_t target_freq)
{

		uint32_t RFdiv = 0;
	    uint16_t vco_freq = (target_freq > MAX_VCO_FREQ) ? MAX_VCO_FREQ : target_freq ;

	    while (vco_freq < MIN_VCO_FREQ)
	       {
	           vco_freq *= 2;
	           RFdiv++;
	       }
	    uint32_t    reg=  R0_MAX2871(
	    		MODE_FRAC,
				vco_freq/F_PFD,	//N_CNT
				vco_freq%F_PFD	//FRAC
	    		);
    return reg;
}







