/*
 * const.c
 *
 *  Created on: 03 ���. 2016 �.
 *      Author: Piton
 */
#include "const.h"





#define SW1 0b101010U //000 0 0  1
#define SW2 0b100110U //010 2 2  2
#define SW3 0b011010U //100 4 1  3
#define SW4 0b010110U //110 6 3  4
#define SW5 0b101001U //001 1 4  5
#define SW6 0b100101U //011 3 6  6
#define SW7 0b011001U //101 5 5  7
#define SW8 0b010101U //111 7 7  8


#define   F20_36 	((SW8<<6)| SW2)
#define   F36_52 	((SW1<<6)| SW2)
#define   F52_84 	((SW7<<6)| SW2)
#define  F84_132	((SW2<<6)| SW2)
#define F132_228	((SW6<<6)| SW2)
#define F228_372	((SW3<<6)| SW2)
#define F372_612	((SW5<<6)| SW2)
#define F612_1044   ((SW4<<6)| SW2)

#define F1044_1444  ((SW8<<6)| SW8)
#define F1444_1844	((SW8<<6)| SW7)
#define F1844_2244	((SW8<<6)| SW6)
#define F2244_2644  ((SW8<<6)| SW5)
#define F2644_3000  ((SW8<<6)| SW4)

#define F3000_4500  ((SW8<<6)| SW3)
#define F4500_6000  ((SW8<<6)| SW2)
#define FLT_ALL     ((SW8<<6)| SW1)

const Sw_Struct mSw_Struct[17]={
		    {F20_36, 1, 2}, //0
		    {F20_36, 1, 2}, //1
		    {F36_52, 1, 2}, //2
		    {F52_84, 1, 2}, //3
		   {F84_132, 1, 2}, //4
		  {F132_228, 1, 2}, //5
		  {F228_372, 1, 2}, //6
		  {F372_612, 1, 2}, //7
		 {F612_1044, 1, 2}, //8

		{F1044_1444, 2, 1}, //9
		{F1444_1844, 2, 1}, //10
		{F1844_2244, 2, 1}, //11
		{F2244_2644, 2, 1}, //12
		{F2644_3000, 2, 1}, //13
	    {F3000_4500, 2, 1}, //14
		{F4500_6000, 2, 1}, //15
		{FLT_ALL, 2, 1}     //16

};

const byte Atn_tbl[]={
 atn0,atn1,atn2,atn3,atn4,atn5,atn6,atn7,
 atn8,atn9,atn10,atn11,atn12,atn13,atn14,atn15,
 atn16,atn17,atn18,atn19,atn20,atn21,atn22,atn23,
 atn24,atn25,atn26,atn27,atn28,atn29,atn30,atn31,
};

const word Rec_chan_6000[][2]=
{
		//�������� ������� +140 MHz      IF1 = 2245
		{ 14,5},
		{844,4},
		{940,5},
		{988,4},
		{1212,5},
		{1308,4},

		//��� �������� ������� -140 MHz IF1  999
		{1444,1},
		{1596,2},
		{1612,0},
		{1692,1},
		{1708,2},
		{1900,1},
		{1916,2},
		{1932,1},
		{2172,0},
		{2252,2},
		{2300,1},
		{2348,2},
		{2364,1},
		{2444,2},
		{2508,1},
		{2556,2},
		{2572,0},
		{2716,1},
		{2844,0},
		{2892,1},
		{3052,2},
		{3084,1},
		{3132,2},
		{3436,1},
		{3484,2},
		{3516,1},
		{3628,2},
		{3660,1},
		{3692,2},
		{3724,1},
		{3980,2},
		{4092,1},
		{4156,2},
		{4204,1},
		{4556,2},
		{4588,1},
		{4764,2},
		{4876,1},

		//    IF1 2245
		{4996,9},//[2]
		{5100,8},
		{5196,9},
		{5436,8},
		{5452,9},
		{5468,8},
		{5772,9},
		{5964,8},
		{6004,9},//[2]

		{0,0}  //End

};


const word  IF2_RS[]=
{
//��� �������� 	������� -140 MHz 999
	1129,	//	0 	IF1  999
	1139,   //  1   IF1  999
	1149,   //  2   IF1  999
	1139,   //  3   IF1  999
//�������� 		������� +140 MHz   2245
	2095,    // 4 IF1 2245
	2105,    // 5 IF1 2245
	2115,    // 6 IF1 2245
	2105,    // 7 IF1 2245
 //�������� 		������� +140 MHz   2245
	2095,    //  8 IF1 2245
	2105,    //  9 IF1 2245
	2115,    //  10 IF1 2245
	2105,    //  11 IF1 2245

};




const word Rec_chan_RS[][2]=
{
//�������� ������� +140 MHz IF=1240
	{  14,6}, //[1]
	{ 232,3}, //[2]
	{ 282,6}, //[3]
	{ 326,3}, //[4]
	{ 389,6}, //[5]
	{ 396,3}, //[6]
	{ 470,6}, //[7]
	{ 480,3}, //[8]
	{ 564,6}, //[9]
	{ 574,3}, //[10]
	{ 623,5}, //[11]
	{ 629,3}, //[12]
	{ 776,6}, //[13]
	{ 790,3}, //[14]
	{ 815,5}, //[15]
	{ 871,6}, //[16]
	{ 899,3}, //[17]
	{ 916,6}, //[18]
	{ 930,3}, //[19]
	{1021,5}, //[20]
	{1029,6}, //[21]
	{1037,5}, //[22]
	{1142,6}, //[23]
	{1151,5}, //[24]
	{1235,6}, //[25]
	{1244,5}, //[26]
	{1322,6}, //[27]
	{1333,5}, //[28]
	{1392,6}, //[29]
	{1403,5}, //[30]
	{1429,4}, //[31]

//��� �������� ������� -140 MHz IF=970
	{1444,1},//[1]
	{1489,0},//[2]
	{1550,1},//[3]
	{1559,2},//[4]
	{1568,1},//[5]
	{1569,0},//[6]
	{1653,1},//[7]
	{1662,0},//[8]
	{1718,2},//[9]
	{1739,0},//[10]
	{1858,2},//[11]
	{1879,0},//[12]
	{1927,1},//[13]
	{1939,0},//[14]
	{2020,1},//[15]
	{2032,2},//[16]
	{2044,0},//[17]
	{2192,2},//[18]
	{2220,0},//[19]
	{2269,1},//[20]
	{2283,2},//[21]
	{2297,1},//[22]
	{2309,0},//[23]
	{2388,1},//[24]
	{2402,2},//[25]
	{2416,1},//[26]
	{2423,0},//[27]
	{2472,2},//[28]
	{2500,0},//[29]
	{2821,1},//[30]
	{2838,2},//[31]
	{2856,1},//[32]
	{2978,2},//[33]
	{2996,1},//[34]
	{3005,1},
	{0,0}  //End
};

const word  IF2_RS_TEMP[]=
{
//��� �������� 	������� -140 MHz 970
	1103,	//	0 	1103 MHz
	1139,   //  1   IF1  999
	1117,	//	2	1117 MHz
//�������� 		������� +140 MHz   2140
	1986,	//	3	1986 MHz
	2120,	//	4	1993 MHz
	2105,    // 5 IF1 2245
	2028,	//	6	2014 MHz
 //�������� 		������� +140 MHz   2140
 //	2000	//	7 2000 MHz
	2105    //  7 IF1 2245

};