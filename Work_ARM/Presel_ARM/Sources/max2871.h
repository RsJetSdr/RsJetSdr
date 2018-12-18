/*
 * max2871.h
 *
 *  Created on: 28 февр. 2017 г.
 *      Author: Piton
 */

#ifndef SOURCES_MAX2871_H_
#define SOURCES_MAX2871_H_
/**
 * MAX2871
 */
// Table of frequency ranges for each VCO value.
// The values were derived from sampling multiple
// units over a temperature range of -10 to 40 deg C.
/*
typedef std::map<uint8_t,uhd::range_t> vco_map_t;
static const vco_map_t max2871_vco_map =
    boost::assign::map_list_of
    (0,uhd::range_t(2767776024.0,2838472816.0))
    (1,uhd::range_t(2838472816.0,2879070053.0))
    (1,uhd::range_t(2879070053.0,2921202504.0))
    (3,uhd::range_t(2921202504.0,2960407579.0))
    (4,uhd::range_t(2960407579.0,3001687422.0))
    (5,uhd::range_t(3001687422.0,3048662562.0))
    (6,uhd::range_t(3048662562.0,3097511550.0))
    (7,uhd::range_t(3097511550.0,3145085864.0))
    (8,uhd::range_t(3145085864.0,3201050835.0))
    (9,uhd::range_t(3201050835.0,3259581909.0))
    (10,uhd::range_t(3259581909.0,3321408729.0))
    (11,uhd::range_t(3321408729.0,3375217285.0))
    (12,uhd::range_t(3375217285.0,3432807972.0))
    (13,uhd::range_t(3432807972.0,3503759088.0))
    (14,uhd::range_t(3503759088.0,3579011283.0))
    (15,uhd::range_t(3579011283.0,3683570865.0))
    (20,uhd::range_t(3683570865.0,3711845712.0))
    (21,uhd::range_t(3711845712.0,3762188221.0))
    (22,uhd::range_t(3762188221.0,3814209551.0))
    (23,uhd::range_t(3814209551.0,3865820020.0))
    (24,uhd::range_t(3865820020.0,3922520021.0))
    (25,uhd::range_t(3922520021.0,3981682709.0))
    (26,uhd::range_t(3981682709.0,4043154280.0))
    (27,uhd::range_t(4043154280.0,4100400020.0))
    (28,uhd::range_t(4100400020.0,4159647583.0))
    (29,uhd::range_t(4159647583.0,4228164842.0))
    (30,uhd::range_t(4228164842.0,4299359879.0))
    (31,uhd::range_t(4299359879.0,4395947962.0))
    (33,uhd::range_t(4395947962.0,4426512061.0))
    (34,uhd::range_t(4426512061.0,4480333656.0))
    (35,uhd::range_t(4480333656.0,4526297331.0))
    (36,uhd::range_t(4526297331.0,4574689510.0))
    (37,uhd::range_t(4574689510.0,4633102021.0))
    (38,uhd::range_t(4633102021.0,4693755616.0))
    (39,uhd::range_t(4693755616.0,4745624435.0))
    (40,uhd::range_t(4745624435.0,4803922123.0))
    (41,uhd::range_t(4803922123.0,4871523881.0))
    (42,uhd::range_t(4871523881.0,4942111286.0))
    (43,uhd::range_t(4942111286.0,5000192446.0))
    (44,uhd::range_t(5000192446.0,5059567510.0))
    (45,uhd::range_t(5059567510.0,5136258187.0))
    (46,uhd::range_t(5136258187.0,5215827295.0))
    (47,uhd::range_t(5215827295.0,5341282949.0))
    (49,uhd::range_t(5341282949.0,5389819310.0))
    (50,uhd::range_t(5389819310.0,5444868434.0))
    (51,uhd::range_t(5444868434.0,5500079705.0))
    (52,uhd::range_t(5500079705.0,5555329630.0))
    (53,uhd::range_t(5555329630.0,5615049833.0))
    (54,uhd::range_t(5615049833.0,5676098527.0))
    (55,uhd::range_t(5676098527.0,5744191577.0))
    (56,uhd::range_t(5744191577.0,5810869917.0))
    (57,uhd::range_t(5810869917.0,5879176194.0))
    (58,uhd::range_t(5879176194.0,5952430629.0))
    (59,uhd::range_t(5952430629.0,6016743964.0))
    (60,uhd::range_t(6016743964.0,6090658690.0))
    (61,uhd::range_t(6090658690.0,6128133570.0));
*/


//==============================================================
#define R0_MAX2871(\
		INT,\
		N_CNT,\
		FRAC)(\
 ((unsigned long)(INT & 1)<<31)|\
 ((unsigned long)(N_CNT & 0xFFFF)<<15)|\
 ((unsigned long)(FRAC & 0xFFF)<<3)|\
 	 (unsigned long)0)

typedef enum{ MODE_FRAC, MODE_INT } pll_mode_t; //[31]

//==============================================================
#define R1_MAX2871(\
		CPL,\
		CPT,\
		PHASE,\
		MOD)(\
 ((unsigned long)(CPL & 3)<<29)|\
 ((unsigned long)(CPT & 3)<<27)|\
 ((unsigned long)(PHASE & 0xFFF)<<15)|\
 ((unsigned long)(MOD & 0xFFF)<<3)|\
 	 (unsigned long)1)
//==============================================================
#define R2_MAX2871(\
		LDS,\
		SDN,\
		MUXOUT,\
		REFx2,\
		RDIV2,\
		R_DIV,\
		D_BUF,\
		ICP,\
		LDF,\
		LDP,\
		POL,\
		SHDN,\
		TRI,\
		RST)(\
 ((unsigned long)(LDS & 1)<<31)|\
 ((unsigned long)(SDN & 3)<<29)|\
 ((unsigned long)(MUXOUT & 7)<<26)|\
 ((unsigned long)(REFx2 & 1)<<25)|\
 ((unsigned long)(RDIV2 & 1)<<24)|\
 ((unsigned long)(R_DIV & 0x3FF)<<14)|\
 ((unsigned long)(D_BUF & 1) <<13)|\
 ((unsigned long)(ICP & 0xF)<<9)|\
 ((unsigned long)(LDF & 1)<<8)|\
 ((unsigned long)(LDP & 1)<<7)|\
 ((unsigned long)(POL & 1)<<6)|\
 ((unsigned long)(SHDN & 1)<<5)|\
 ((unsigned long)(TRI & 1)<<4)|\
 ((unsigned long)(RST & 1)<<3)|\
 	 (unsigned long)2)
//--------
       typedef enum {
           LOW_NOISE_AND_SPUR_LOW_NOISE,
		   LOW_NOISE_AND_SPUR_RESERV,
           LOW_NOISE_AND_SPUR_LOW_SPUR_1,
           LOW_NOISE_AND_SPUR_LOW_SPUR_2
       } low_noise_and_spur_t;  //[30:29]

       /* Charge Pump Currents  */
          typedef enum{
              CHARGE_PUMP_CURRENT_0_32MA,
              CHARGE_PUMP_CURRENT_0_64MA,
              CHARGE_PUMP_CURRENT_0_96MA,
              CHARGE_PUMP_CURRENT_1_28MA,
              CHARGE_PUMP_CURRENT_1_60MA,
              CHARGE_PUMP_CURRENT_1_92MA,
              CHARGE_PUMP_CURRENT_2_24MA,
              CHARGE_PUMP_CURRENT_2_56MA,
              CHARGE_PUMP_CURRENT_2_88MA,
              CHARGE_PUMP_CURRENT_3_20MA,
              CHARGE_PUMP_CURRENT_3_52MA,
              CHARGE_PUMP_CURRENT_3_84MA,
              CHARGE_PUMP_CURRENT_4_16MA,
              CHARGE_PUMP_CURRENT_4_48MA,
              CHARGE_PUMP_CURRENT_4_80MA,
              CHARGE_PUMP_CURRENT_5_12MA
          } charge_pump_current_t; //[12:9]
          typedef enum{FRAC_LD,INT_LD } loc_dec_f; //[8]
//==============================================================
//Default: 0x0000000B
#define R3_MAX2871(\
		VCO,\
		VAS_SHDN,\
		VAS_TEMP,\
		CSM,\
		MUTEDEL,\
		CDM,\
		CDIV)(\
 ((unsigned long)(VCO & 0x3F)<<26)|\
 ((unsigned long)(VAS_SHDN & 1)<<25)|\
 ((unsigned long)(VAS_TEMP & 1)<<24)|\
 ((unsigned long)(CSM & 1)<<18)|\
 ((unsigned long)(MUTEDEL & 1)<<17)|\
 ((unsigned long)(CDM & 3)<<15)|\
 ((unsigned long)(CDIV & 0xFFF)<<3)|\
 	 (unsigned long)3)


       typedef enum{ VAS_EN,  VAS_DI           } vas_mode_t; //[25]
       typedef enum{ VAS_TEMP_DI,  VAS_TEMP_EN } vas_temp_t; //[24]
       typedef enum{ CSM_OFF, CSM_ON            } csm_mode_t; //[18]


//==============================================================
//Default: 0x6180B23C
#define R4_MAX2871(\
		SDLDO,\
		SDDIV,\
		SDREF,\
		BS,\
		FB,\
		DIVA,\
		SDVCO,\
		MTLD,\
		BDIV,\
		RFB_EN,\
		BPWR,\
		RFA_EN,\
		APWR)(\
((unsigned long)(SDLDO & 1)<<28)|\
((unsigned long)(SDDIV & 1)<<27)|\
((unsigned long)(SDREF & 1)<<26)|\
((unsigned long)((BS>>8) & 0x3)<<24)|\
((unsigned long)(FB & 1)<<23)|\
((unsigned long)(DIVA & 7)<<20)|\
((unsigned long)(BS & 0xFF)<<12)|\
((unsigned long)(SDVCO & 1)<<11)|\
((unsigned long)(MTLD & 1)<<10)|\
((unsigned long)(BDIV & 1)<<9)|\
((unsigned long)(RFB_EN & 1)<<8)|\
((unsigned long)(BPWR & 3)<<6)|\
((unsigned long)(RFA_EN & 1)<<5)|\
((unsigned long)(APWR & 3)<<3)|\
	(unsigned long)4)

       /* Output Powers */
        typedef enum{
            OUTPUT_POWER_M4DBM,
            OUTPUT_POWER_M1DBM,
            OUTPUT_POWER_2DBM,
            OUTPUT_POWER_5DBM
        } output_power_t; //[7:6] [4:3]

        typedef enum{RF_OFF,RF_ON} output_enable_t;

//==============================================================
//Default: 0x00400005
#define R5_MAX2871(VAS_DLY,\
		SDPLL,\
		F01,\
		LD,\
		MUXOUT,\
		ADCS,\
		ADCM)(\
((unsigned long)(VAS_DLY & 3)<<29)|\
((unsigned long)(SDPLL & 1)<<25)|\
((unsigned long)(F01 & 1)<<24)|\
((unsigned long)(LD & 3)<<22)|\
((unsigned long)((MUXOUT>>3) & 1)<<18)|\
((unsigned long)(ADCS & 1)<<6)|\
((unsigned long)(ADCM & 7)<<3)|\
	(unsigned long)5)

       typedef enum{ PLL_ON,PLL_OFF   } pll_enamle_t; //[25]
       typedef enum{ AUTO_INT_OFF,AUTO_INT_ON} mode_auto_t; //[24]


          /* LD Pin Modes   */
           typedef enum{
               LD_PIN_MODE_LOW,
               LD_PIN_MODE_DLD,
               LD_PIN_MODE_ALD,
               LD_PIN_MODE_HIGH
           } ld_pin_mode_t; //[23:22]

           /* MUXOUT Modes */
           typedef enum{
               MUXOUT_TRI_STATE,
               MUXOUT_HIGH,
               MUXOUT_LOW,
               MUXOUT_RDIV,
               MUXOUT_NDIV,
               MUXOUT_ALD,
               MUXOUT_DLD,
               MUXOUT_SYNC,
               MUXOUT_SPI=0b1100
           } muxout_mode_t; //[18] and reg_2[28:26]


//==============================================================
#define R6_MAX2871(DIE,POR,ADC,ADCV,VASA,VCO)(\
	(unsigned long)6)








       typedef enum {
           CLOCK_DIV_MODE_CLOCK_DIVIDER_OFF,
           CLOCK_DIV_MODE_FAST_LOCK,
           CLOCK_DIV_MODE_PHASE
       } clock_divider_mode_t;


 typedef enum {   MIN_VCO_FREQ = 3000, MAX_VCO_FREQ = 6000} vco_freq_t;

 typedef enum
 {
   pw_off  = 0,
   pw_m4db,
   pw_m1db,
   pw_2db,
   pw_5db
 } Pw_out_t;


#define MUX_OUT 	MUXOUT_DLD  //MUXOUT_RDIV
#define DOUBLER   	0     	 	// Disable
#define REFCLK   	26
#define MOD_FRAC 	26
#define M_FRAC	 	0
#define M_INT	 	1
#define MODE_PLL    M_FRAC
#define F_PFD       (REFCLK/1)





 void InitPLL(void);
 uint16_t SetFreqMHz_Frac(uint16_t target_freq,Pw_out_t Pw);
 uint32_t SetFreqMHz(uint16_t target_freq);
#endif /* SOURCES_MAX2871_H_ */
