//---------------------------------------------------------------------------

#ifndef Rs_FX3_USBH
#define Rs_FX3_USBH
//---------------------------------------------------------------------------

#include <windows.h>
#include <stdio.h>
#include <mutex>
#include <cyapi.h>
#pragma hdrstop

#define		MAX_TRANSFER	8

#define		VENDOR_ID	    0x04B4
#define		PRODUCT_BOOT1   0x00F3
#define		PRODUCT_BOOT2   0x00BC
#define		PRODUCT_APL		0x00F1

#define VND_CMD_ID_CHECK        (0xB0)
#define VND_CMD_CFGSTAT 		(0xB1)
#define VND_CMD_CFGLOAD 		(0xB2)
#define VND_CMD_SPI_WRITE       (0xC2)
#define VND_CMD_SPI_READ        (0xC3)
#define VND_CMD_UART            (0xAE)
#define VND_CMD_TASK            (0xAB)
//reg
#define MB_RX_CONTRL0  	    0
//Bit flags
//85 95
#define ADC_RAND				(1<<0)
#define ADC_DITHER              (1<<1)
#define ADC_GAIN                (1<<2)
#define BAND                    (1<<3)
#define RUN_DDC                 (1<<28)
#define ADC_BYPASS              (1<<30)
#define MASTER_RESET			(1<<31)


//reg
#define MB_RAW_CTRL         1
//flags
#define MODE_STOP           (0<<13)
#define MODE_RAW            (1<<13)
#define MODE_BURST          (2<<13)



/*
	140 MHz - (81.92 MHz * 1.75)
	81.92/4 = 20.48
	+ 		   3.36
			  23.84
*/

#define CENTER_REC_140MHZ  23840000
#define CENTER_FFT_140MHZ  140000000
//reg
#define MB_RX_FREQ      	4
#define MB_RX_SCALE_IQ      5
#define MB_RX_DECIM         6
//reg
#define MB_RX_MUX           7
//flags
#define FLAG_DSP_RX_MUX_SWAP_IQ   (1 << 0)
#define FLAG_DSP_RX_MUX_REAL_MODE (1 << 1)
#define FLAG_DSP_RX_MUX_INVERT_Q  (1 << 2)
#define FLAG_DSP_RX_MUX_INVERT_I  (1 << 3)
#define FIR_BYPASS				  (1<<4)
//reg
#define MB_ADC_REG          8
//flags
#define RANDOM					(1<<0)
#define DITHER					(1<<1)
#define GAIN					(1<<2)
//reg
#define MB_ATTN_REG         9






//#define 	RsCyFx3_FIRMWARE "RS_JET_CYFX.img"
#define 	RsCyFx3_FIRMWARE "RsJetSdr_fx3.img"
//#define 	RsCyFx3_FPGA     "top_rs_fx3.rbf"
//#define 	RsCyFx3_FPGA     "rs_fx3.rbf"
//#define 	RsCyFx3_FPGA     "rs_fx3_5.rbf"     //Новая плата
#define 	RsCyFx3_FPGA     "D_PR_USB30_V1.rbf"
//#define  BULK_END_POINT   1

#define CMD_EVENT    WM_USER+1024
#define RAW_EVENT    CMD_EVENT+1
#define LOW_EVENT    CMD_EVENT+2
#define HIGH_EVENT   CMD_EVENT+3
#define ERR_EVENT    CMD_EVENT+4
#define END_EVENT    CMD_EVENT+5
typedef enum {  _STOP,_RAW,_BURST} Mode_REG;

#pragma pack(push,1)
typedef struct
{
   uint16_t  wValue;
   uint16_t wIndex;
   uint16_t Len;
   uint8_t *inData;
}cmd_USB,*pcmd_USB;

typedef struct{
	uint8_t   *buf;       /* Pointer to the data */
	uint8_t   opCode;     /* Vendor request code */
	uint16_t 	wValue;
	uint16_t 	wIndex;
	UINT    addr;       /* Read/Write address */
	long    size;       /* Size of the read/write */
	bool    isRead;     /* Read or write */
}Data_EP0,*pData_EP0;

typedef int16_t	Frac16;
typedef int32_t  Frac32;

typedef struct {
   Frac16     real;
   Frac16     imag;
} CFrac16;

typedef struct {
   Frac32     real;
   Frac32     imag;
} CFrac32;

#pragma pack(pop)
 //========================================
class RsFx3_USB
{
public:
	HANDLE hMutex;
	RsFx3_USB(HANDLE extWin);
	~RsFx3_USB();
	CCyUSBEndPoint *StreameptIn;


	BOOL	OpenDevice(char*);
	void	CloseDevice();
	bool    bSuperSpeed;

//	const HANDLE DrvHandle() { return mUSBDevice->DeviceHandle(); }
	const HANDLE DrvHandle() { return  (HANDLE)(mUSBDevice); }

	const	char *lastError(void) { return last_error_str; };
	void    SetSampleRate(double rate);
	double  GetSampleRate(void) {return SampleRate;}
	double  SetFreqDDC(const double requested_freq);
	bool 	StopCommand();
	bool 	RawCommand();
	bool 	BurstCommand(USHORT size =2048);
	bool    SelectBand(bool band);
	bool    Command(cmd_USB *pData);
	bool    Get_Id(UCHAR *pData);
	bool	OutCmdReg(UCHAR adr, ULONG data);
	bool	OutSerial(UCHAR *pData, DWORD *Snt, int Echo_Size = 0);
	wchar_t * getSerialNumber() {return mUSBDevice->SerialNumber;}
private:
//	mutable std::recursive_mutex _accessMutex;
//	mutable std::recursive_mutex _regMutex;

	mutable std::mutex _accessMutex;
	mutable std::mutex _regMutex;

	CCyUSBDevice *mUSBDevice;
	ULONG MB_CONTROL0_BR;
	ULONG REG_ADC;
	double SampleRate;
	double FreqDDC;
	HANDLE FextWin;
	bool bFirmvare;
	bool bFpga;
	char * last_error;
	int ep_count;
	char *last_error_str;
	double FsamleAdc;

	void  get_freq_and_freq_word(
		const double requested_freq,
		const double tick_rate,
		double &actual_freq,
		int &freq_word);
	//	int		loadFirmware(const char * filename);
	int		loadFpga(const char * filename);

	int findRsDevice();

	bool    SetCmdFPGA(cmd_USB *pData);
	wchar_t *pSerialNum;
	uint8_t Str_Id[32];
};
extern RsFx3_USB  *pUSBDev;
#endif
