#ifndef _ADF4351H_
#define _ADF4351H_

#define R0_ADF4351(INT,FRAC)\
(((unsigned long)INT<<15)|((unsigned long)FRAC<<3)|\
(unsigned long)0)

#define R1_ADF4351(PHASE_ADJ,PRES,PHASE,MOD)\
(((unsigned long)PHASE_ADJ<<28)|\
 ((unsigned long)PRES<<27)|\
 ((unsigned long)PHASE<<15)|\
 ((unsigned long)MOD<<3)|\
(unsigned long)1)
//Warning !!!!!!!!!!!!!!!!!
// if RF < 3.6GHz
#define PRES_4 0
/// if RF > 3.6GHz
#define PRES_8 1

#define R2_ADF4351(MODE,MUXOUT,REFx2,RDIV2,RCNT,D_BUF,ICP,LDF,LDP,POL,PWD,CP_THREE,RES)\
(((unsigned long)MODE<<29)|\
 ((unsigned long)MUXOUT<<26)|\
 ((unsigned long)REFx2<<25)|\
 ((unsigned long)RDIV2<<24)|\
 ((unsigned long)RCNT<<14)|\
 ((unsigned long)D_BUF<<13)|\
 ((unsigned long)ICP<<9)|\
 ((unsigned long)LDF<<8)|\
 ((unsigned long)LDP<<7)|\
 ((unsigned long)POL<<6)|\
 ((unsigned long)PWD<<5)|\
 ((unsigned long)CP_THREE<<4)|\
 ((unsigned long)RES<<3)|\
(unsigned long)2)
//Mode
#define  LOW_NOISE 0
#define  LOW_SPUR  3
//MuxOUT
#define  TREE_STATE  	0
#define  DVdd           1
#define  DGND           2
#define  R_DIV          3
#define  N_DIV          4
#define  A_LOCK         5
#define  D_LOCK         6

#define  ICP_310uA      0
#define  ICP_630uA      1
#define  ICP_940uA      2
#define  ICP_1_25mA     3
#define  ICP_1_56mA     4
#define  ICP_1_88mA     5
#define  ICP_2_19mA     6
#define  ICP_2_5mA      7
#define  ICP_2_81mA     8
#define  ICP_3_13mA     9
#define  ICP_3_44mA     10
#define  ICP_3_75mA     11
#define  ICP_4mA        12
#define  ICP_4_38mA     13
#define  ICP_4_69mA     14
#define  ICP_5mA        15

#define  LDF_FRAC	 0
#define  LDF_INT        1

#define  LDP_10nS	 0
#define  LDP_6nS        1

#define  MODE_FRAC 0
#define  MODE_INT     3


// LDF_INT  -> LDP_6nS
// LDF_FRAC -> LDP_10nS

#define  LD_NEG		   0
#define  LD_POS        1

#define  PW_UP         0
#define  PW_DOWN       1

#define  DI_BIT     0
#define  EN_BIT     1


#define R3_ADF4351(BSCM,ABP,CHARGE,CSR,CLK_MODE,CLK_DIV)\
	(((unsigned long)BSCM<<23)|\
	 ((unsigned long)ABP<<22)|\
	 ((unsigned long)CHARGE<<21)|\
 	 ((unsigned long)CSR<<18)|\
 	 ((unsigned long)CLK_MODE<<15)|\
 	 ((unsigned long)CLK_DIV<<3)|\
(unsigned long)3)


#define R4_ADF4351(FEEDBACK,RF_DIV,BAND_DIV,VCO_PWD,MTLD,\
				   AUX_OUT,AUX_EN,AUX_PW,RF_EN,RF_PW)\
(((unsigned long)FEEDBACK<<23)|\
	((unsigned long)RF_DIV<<20)|\
	((unsigned long)BAND_DIV<<12)|\
	((unsigned long)VCO_PWD<<11)|\
	((unsigned long)MTLD<<10)|\
	((unsigned long)AUX_OUT<<9)|\
	((unsigned long)AUX_EN<<8)|\
	((unsigned long)AUX_PW<<6)|\
	((unsigned long)RF_EN<<5)|\
	((unsigned long)RF_PW<<3)|\
(unsigned long)4)

#define R5_ADF4351(LD_PIN)(((unsigned long)LD_PIN<<22)|\
((unsigned long)3<<19)|\
(unsigned long)5)

#define LD_LOCK 1
#define LD_LOW 	2
#define LD_HIGH 3

#define  MASK_REG0 0xFFFFFFE7UL
#endif