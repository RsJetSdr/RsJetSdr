#include "rs_fft.h"
//=========================================================================
FFT::FFT(int size,int w_type)
{
	FFT::nfft = size;
	half_sz = size/2;


	//===========  Create Windows
	pBufWin = new float[nfft];
	MakeWindow(w_type, size, pBufWin);
	for (int i = 0; i<size; i++)
		pBufWin[i] = pBufWin[i] / 32768.0f;

	//===========  Create FFT
	cpxbuf = (liquid_float_complex *)std::malloc(sizeof(liquid_float_complex) * nfft);

//	cpxbuf = (liquid_float_complex *)aligned_malloc(sizeof(liquid_float_complex) * nfft,16);
//	cpxbuf = new liquid_float_complex[nfft];

	plan_fwd = fft_create_plan(size, 
		cpxbuf, 
		cpxbuf, 
		LIQUID_FFT_FORWARD, 
		0);

	//buf = mallocCPX(size);

}
//=========================================================================
FFT::~FFT() {

		fft_destroy_plan(plan_fwd);
		delete []pBufWin;
		 if (cpxbuf) 
			 std::free(cpxbuf);
		// aligned_free(cpxbuf);
		//delete[]cpxbuf;
	
}
//=========================================================================
void FFT::DoFFTWMagnCh( int skip,float baseline,float correction,char *magDb)
 {
	fft_execute(plan_fwd);
  
	for (int i=half_sz+skip; i < nfft; i++)
	{
		 *magDb++ =
		 (char)(10.0f * log10f(MagCPX(*(cpxbuf+i)) + baseline)  + correction);
	}
	for (int i=0; i < half_sz-skip; i++)
	{
		 *magDb++ =
		 (char)(10.0f * log10f(MagCPX(*(cpxbuf+i)) + baseline)  + correction);
	}
 }
//=============================================================================
void FFT::MakeWindow(int wtype, int size, float * window)
{
	int i, j, midn, midp1, midm1;
	float freq, rate, sr1, angle, expn, expsum, cx, two_pi;

	midn = size / 2;
	midp1 = (size + 1) / 2;
	midm1 = (size - 1) / 2;
	two_pi = 8.0f * atanf(1.0f);
	freq = two_pi / size;
	rate = 1.0f /  midn;
	angle = 0.0f;
	expn = logf(2.0f) / midn + 1.0f;
	expsum = 1.0f;

	switch (wtype)
	{
		case 0: // RECTANGULAR_WINDOW
			for (i = 0; i < size; i++)
				window[i] = 1.0f;
			break;
		case 1:	// HANNING_WINDOW
			for (i = 0, j = size - 1, angle = 0.0; i <= midn; i++, j--, angle += freq)
				window[j] = (window[i] = 0.5f - 0.5f * cosf(angle));
			break;
		case 2: // WELCH_WINDOW
			for (i = 0, j = size - 1; i <= midn; i++, j--)
				window[j] = (window[i] = 1.0f - sqrtf((float)((i - midm1) / midp1)));
			break;
		case 3: // PARZEN_WINDOW
			for (i = 0, j = size - 1; i <= midn; i++, j--)
				window[j] = (window[i] = 1.0f - (fabsf((float)(i - midm1) / midp1)));
			break;
		case 4: // BARTLETT_WINDOW
			for (i = 0, j = size - 1, angle = 0.0; i <= midn; i++, j--, angle += rate)
				window[j] = (window[i] = angle);
			break;
		case 5: // HAMMING_WINDOW
			for (i = 0, j = size - 1, angle = 0.0; i <= midn; i++, j--, angle += freq)
				window[j] = (window[i] = 0.5f - 0.46f * cosf(angle));
			break;
		case 6:	// BLACKMAN2_WINDOW
			for (i = 0, j = size - 1, angle = 0.0; i <= midn; i++, j--, angle += freq) {
				cx = cosf(angle);
				window[j] = (window[i] = (.34401f + (cx * (-.49755f + (cx * .15844f)))));
			}
			break;
		case 7: // BLACKMAN3_WINDOW
			for (i = 0, j = size - 1, angle = 0.0; i <= midn; i++, j--, angle += freq) {
				cx = cosf(angle);
				window[j] = (window[i] = (.21747f + (cx * (-.45325f + (cx * (.28256f - (cx * .04672f)))))));
			}
			break;
		case 8: // BLACKMAN4_WINDOW
			for (i = 0, j = size - 1, angle = 0.0; i <= midn; i++, j--, angle += freq)
			{
				cx = cosf(angle);
				window[j] = (window[i] =
							(.084037f +
							(cx *
							(-.29145f +
							(cx *
							(.375696f + (cx * (-.20762f + (cx * .041194f)))))))));
			}
			break;
		case 9: // EXPONENTIAL_WINDOW
			for (i = 0, j = size - 1; i <= midn; i++, j--) {
				window[j] = (window[i] = expsum - 1.0f);
				expsum *= expn;
			}
			break;
		case 10: // RIEMANN_WINDOW
			sr1 = two_pi / size;
			for (i = 0, j = size - 1; i <= midn; i++, j--) {
				if (i == midn) window[j] = (window[i] = 1.0f);
				else {
					cx = sr1 * (midn - i);
					window[i] = sinf(cx) / cx;
					window[j] = window[i];
				}
			}
			break;
		case 11: // BLACKMANHARRIS_WINDOW
			{
				float
						a0 = 0.35875F,
						a1 = 0.48829F,
						a2 = 0.14128F,
						a3 = 0.01168F;


				for (i = 0; i<size;i++)
				{
					window[i] = a0 - a1* cosf(two_pi*(i+0.5f)/size)
							+ a2* cosf(2.0f*two_pi*(i+0.5f)/size)
							- a3* cosf(3.0f*two_pi*(i+0.5f)/size);
				}
			}
			break;
		default:
			for (i = 0; i < size; i++)
				window[i] = 1.0f;
			return;
	}
}

