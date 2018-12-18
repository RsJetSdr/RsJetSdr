#ifndef _CONSTH_
#define	_CONSTH_
#include "PE_Types.h"

#define DELAY_5mkS	 65
#define DELAY_10mkS	130
#define DELAY_20mkS	260
#define DELAY_25mkS	325
#define DELAY_30mkS	390
#define DELAY_50mkS	650
#define DELAY_60mkS	790
#define DELAY_100mkS	1300

enum {
     atn0  = 0b111111,
     atn05 = 0b111110,
     atn1  = 0b111101,
     atn2  = 0b111011,
     atn4  = 0b110111,
     atn8  = 0b101111,
     atn16  =0b011111
};
     
//==================================     
     


enum {
//  atn_0	=	atn0,
//  atn_1	=	atn1,
//  atn_2	=	atn2,
  atn3	=	atn2  & atn1,
//  atn4 =   atn4,
  atn5	=	atn4  & atn1,
  atn6	=	atn4  & atn2,
  atn7	=	atn4  & atn2 & atn1,
//  atn_8	=	atn8,
  atn9	=	atn8  & atn1,
 atn10	=	atn8  & atn2,
 atn11	=	atn8  & atn2 & atn1,
 atn12	=	atn8  & atn4,
 atn13	=	atn8  & atn4 & atn1,
 atn14	=	atn8  & atn4 & atn2,
 atn15	=	atn8  & atn4 & atn2 & atn1,
// atn_16	=	atn16,
 atn17	=	atn16 & atn1,
 atn18	=	atn16 & atn2,
 atn19	=	atn16 & atn2 & atn1,
 atn20	=	atn16 & atn4,
 atn21	=	atn16 & atn4 & atn1,
 atn22	=	atn16 & atn4 & atn2,
 atn23	=	atn16 & atn4 & atn2 & atn1,
 atn24	=	atn16 & atn8,
 atn25	=	atn16 & atn8 & atn1,
 atn26	=	atn16 & atn8 & atn2,
 atn27	=	atn16 & atn8 & atn2 & atn1,
 atn28	=	atn16 & atn8 & atn4,
 atn29	=	atn16 & atn8 & atn4 & atn1,
 atn30	=	atn16 & atn8 & atn4 & atn2,
 atn31	=	atn16 & atn8 & atn4 & atn2 & atn1
};
//-------------------------------------

extern const word Rec_chan_RS[][2];
extern const word Rec_chan_6000[][2];

extern const word  IF2_RS[];
extern const byte Atn_tbl[];
typedef struct
{
uint32_t SW_PRESEL :12;
uint32_t SW_IN :2;
uint32_t SW_OUT :2;
}Sw_Struct,*pSw_Struct;
#define IF_970  2
#define IF_2140 1

extern const Sw_Struct mSw_Struct[];



//extern const word  Fros[];
//extern const short Pres_chan[][5];

//#define Rec_chan        Rec_chan_RS
#define Rec_chan        Rec_chan_6000
#define Fros 			IF2_RS


#define DELAY_CIF  (DELAY_100mkS * 20)            
#define DELAY_PLL  (DELAY_100mkS * 2)

//#define MOD_PLL	16
//#define MOD_PLL1	32



//------------------------------------------------------------
/*
#define REFCLK       26
#define M_FRAC		 0	
#define M_INT		 1	

#define DOUBLER  	 0     	 	// Disable 
#define MODE_PLL    M_FRAC


#if  (MODE_PLL == M_FRAC)
	#define RDIV2		 0  	 		// Ref DIV
	
	#define  MCSR		  0
	#define FLOCK 		  0 	 //  ENable
	#define CNT_FAST       1   //

	#if RDIV2 == 0
		#define MOD_FRAC 	26
		#define ICP                  ICP_4mA
		#define RCNT   		1
		#define  LDF 		LDF_FRAC
		#define BSC     		52
	#else
		#define MOD_FRAC 	13
		#define ICP                 ICP_5mA
		#define RCNT   		1
		#define  LDF 		LDF_FRAC
		#define BSC     		26
	#endif
#else
	#define RDIV2		 	1  	 		// Ref DIV
	#define RCNT   			13
	#define ICP                      ICP_3_75mA
	#define  LDF 		     LDF_INT
	#define BSC     			2
	#define MOD_FRAC 		2

	#define  MCSR		  	0
	#define FLOCK 		  	0 	 //  ENable
	#define CNT_FAST       	4   //

#endif
*/

/*
0	-4
1	-1
2	 2
3	 5
*/

#define PW_IF1    1  //-1  dBm
#define PW_IF2    0 // -4  dBm



//------------------------------------------------------------
typedef struct _ReceiverStruct
{
     word  IF1;   		//
     word  IF2;
     word Pres;		//
     word Atn_In;
     word Atn_Out;
     word Atn_Gain;
}ReceiverStruct,*pReceiverStruct;

#endif
