#ifndef RSFFTH
#define RSFFTH
#include <math.h>
#include <cstdlib> 
#include "liquid/liquid.h"

enum{
	 RECTANGULAR=0,HANNING,WELCH,PARZEN,BARTLETT,HAMMING,BLACKMAN2,BLACKMAN3,BLACKMAN4,
	 EXPONENTIAL,RIEMANN,BLACKMANHARRIS};

class FFT
{
private:
	fftplan plan_fwd;
	int nfft;
	int half_sz;
	
	void MakeWindow(int wtype, int size, float * window);
	inline float MagCPX(liquid_float_complex z) {return (float)(z.real * z.real + z.imag * z.imag);}
	inline float SqrMagCPX(liquid_float_complex z) {return  sqrtf(z.real * z.real + z.imag * z.imag);}
	
	void * aligned_malloc(size_t size, size_t alignement)
	{
		void * p = malloc(size + --alignement);
		void * p1 = (void*)(((size_t)p + alignement) & ~alignement);

		((char*)p1)[-1] = (char)((char*)p1 - (char*)p);

		return p1;
	}

	void aligned_free(void * pMem)
	{
		char * pDelete = (char*)pMem - ((char*)pMem)[-1];
		free(pDelete);
	}

public:
	FFT(int size,int w_type);
	~FFT();
	void DoFFTWMagnCh(int skip,
					  float baseline,     //(1.0e-15)
					  float correction,   //0
					  char *magDb);		  //result -128...+127 dB

//	void DoFFTWInverse(CPX * in, CPX * out, int size);
	float *pBufWin;
	liquid_float_complex *cpxbuf;

};

#endif
