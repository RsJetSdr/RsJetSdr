//
#include "pch.h"
#include "RsJetSuite.h"
#include "liquid/liquid.h"

#ifdef _SFML_
#include <SFML/Window.hpp>
#include <SFML/OpenGL.hpp>
#endif
int gSize; 
liquid_float_complex test_data[2048];
int main()
{
#ifdef _SFML_
	// Request a 24-bits depth buffer when creating the window
	sf::ContextSettings contextSettings;
	contextSettings.depthBits = 24;

	// Create the main window
	sf::Window window(sf::VideoMode(640, 480), "SFML window with OpenGL", sf::Style::Default, contextSettings);

	// Make it the active window for OpenGL calls
	window.setActive();

	// Set the color and depth clear values
	glClearDepth(1.f);
	glClearColor(0.f, 0.f, 0.f, 1.f);

	// Enable Z-buffer read and write
	glEnable(GL_DEPTH_TEST);
	glDepthMask(GL_TRUE);

	// Disable lighting and texturing
	glDisable(GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);

	// Configure the viewport (the same size as the window)
	glViewport(0, 0, window.getSize().x, window.getSize().y);

	// Setup a perspective projection
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	GLfloat ratio = static_cast<float>(window.getSize().x) / window.getSize().y;
	glFrustum(-ratio, ratio, -1.f, 1.f, 1.f, 500.f);

	// Define a 3D cube (6 faces made of 2 triangles composed by 3 vertices)
	GLfloat cube[] =
	{
		// positions    // colors (r, g, b, a)
		-50, -50, -50,  0, 0, 1, 1,
		-50,  50, -50,  0, 0, 1, 1,
		-50, -50,  50,  0, 0, 1, 1,
		-50, -50,  50,  0, 0, 1, 1,
		-50,  50, -50,  0, 0, 1, 1,
		-50,  50,  50,  0, 0, 1, 1,

		 50, -50, -50,  0, 1, 0, 1,
		 50,  50, -50,  0, 1, 0, 1,
		 50, -50,  50,  0, 1, 0, 1,
		 50, -50,  50,  0, 1, 0, 1,
		 50,  50, -50,  0, 1, 0, 1,
		 50,  50,  50,  0, 1, 0, 1,

		-50, -50, -50,  1, 0, 0, 1,
		 50, -50, -50,  1, 0, 0, 1,
		-50, -50,  50,  1, 0, 0, 1,
		-50, -50,  50,  1, 0, 0, 1,
		 50, -50, -50,  1, 0, 0, 1,
		 50, -50,  50,  1, 0, 0, 1,

		-50,  50, -50,  0, 1, 1, 1,
		 50,  50, -50,  0, 1, 1, 1,
		-50,  50,  50,  0, 1, 1, 1,
		-50,  50,  50,  0, 1, 1, 1,
		 50,  50, -50,  0, 1, 1, 1,
		 50,  50,  50,  0, 1, 1, 1,

		-50, -50, -50,  1, 0, 1, 1,
		 50, -50, -50,  1, 0, 1, 1,
		-50,  50, -50,  1, 0, 1, 1,
		-50,  50, -50,  1, 0, 1, 1,
		 50, -50, -50,  1, 0, 1, 1,
		 50,  50, -50,  1, 0, 1, 1,

		-50, -50,  50,  1, 1, 0, 1,
		 50, -50,  50,  1, 1, 0, 1,
		-50,  50,  50,  1, 1, 0, 1,
		-50,  50,  50,  1, 1, 0, 1,
		 50, -50,  50,  1, 1, 0, 1,
		 50,  50,  50,  1, 1, 0, 1,
	};

	// Enable position and color vertex components
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);
	glVertexPointer(3, GL_FLOAT, 7 * sizeof(GLfloat), cube);
	glColorPointer(4, GL_FLOAT, 7 * sizeof(GLfloat), cube + 3);

	// Disable normal and texture coordinates vertex components
	glDisableClientState(GL_NORMAL_ARRAY);
	glDisableClientState(GL_TEXTURE_COORD_ARRAY);

	// Create a clock for measuring the time elapsed
	sf::Clock clock;

	// Start the game loop
	while (window.isOpen())
	{
		// Process events
		sf::Event event;
		while (window.pollEvent(event))
		{
			// Close window: exit
			if (event.type == sf::Event::Closed)
				window.close();

			// Escape key: exit
			if ((event.type == sf::Event::KeyPressed) && (event.key.code == sf::Keyboard::Escape))
				window.close();

			// Resize event: adjust the viewport
			if (event.type == sf::Event::Resized)
				glViewport(0, 0, event.size.width, event.size.height);
		}

		// Clear the color and depth buffers
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Apply some transformations to rotate the cube
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glTranslatef(0.f, 0.f, -200.f);
		glRotatef(clock.getElapsedTime().asSeconds() * 50, 1.f, 0.f, 0.f);
		glRotatef(clock.getElapsedTime().asSeconds() * 30, 0.f, 1.f, 0.f);
		glRotatef(clock.getElapsedTime().asSeconds() * 90, 0.f, 0.f, 1.f);

		// Draw the cube
		glDrawArrays(GL_TRIANGLES, 0, 36);

		// Finally, display the rendered frame on screen
		window.display();
	}

	return EXIT_SUCCESS;
#else
	RsJetSdr *mTest;
	int inch;


	HANDLE demod;
	float audioGain=0.0f;
	char *Str;
	Demod_Str mStr;
	mStr.Type = DemodFM;
	mStr.freq = 103700000;
	mStr.SampleRate = 5120000;
	//	mStr.SampleRate = 10240000;

	mStr.BandWidth = 160000;

	mStr.attn.att1 = 1;
	mStr.attn.att2 = 2;
	mStr.attn.att3 = 3;
	mStr.attn.att_low = 4;
	mStr.attn.mode = Mode_ATT(0);

/*
	std::cout << "sizeof(mStr) " << sizeof(mStr) << std::endl;
	std::cout << "sizeof(Type (Enum)) " << sizeof(mStr.Type) << std::endl;
	std::cout << "sizeof(*DemodPar) " << sizeof(mStr.DemodPar) << std::endl;
	std::cout << "sizeof(freq) " << sizeof(mStr.freq) << std::endl;
	std::cout << "sizeof(SampleRate) " << sizeof(mStr.SampleRate) << std::endl;
	std::cout << "sizeof(BandWidth) " << sizeof(mStr.BandWidth) << std::endl;
	std::cout << "sizeof(AttnStr) " << sizeof(AttnStr) << std::endl;
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
*/

	CreateRsJetSdr(&mTest);
    //Open Hardware 
	HANDLE handle = mTest->OpenUSBDevice(0, 0, &Str);

	std::cout << Str << std::endl;

	int cnt = 0;
	SdrIQData iqData;
	
	if (handle) {
	
		mTest->StartDevice(mStr.SampleRate);
		
		demod =	mTest->SetModem(&mStr);

		audioGain = mTest->GetAudioGain(demod);
		if (audioGain) {
			audioGain = audioGain + 0.5f;
			mTest->SetAudioGain(demod, audioGain);
		}
		audioGain = mTest->GetAudioGain(demod);

		
		while (!_kbhit()){
			
			gSize = mTest->GetIQData(&iqData);
			if (gSize > 0) {

				memcpy(&test_data, iqData.data, sizeof(test_data));
				cnt++;
				gSize = 0;
			}
		}
		inch = getch();
		//demodulation off
		mStr.Type = Demod_Off;
		demod = mTest->SetModem(&mStr);

	
		if (inch != 'q') {
			
			ControlStruct mControl;

			memset(&mControl, 0, sizeof(ControlStruct));
			mControl.FreqStart = 30;
			mControl.FreqEnd = 100;
			mControl.nFFT = Scan_10KHz;
			
			while (!_kbhit()) {
				std::this_thread::sleep_for(std::chrono::milliseconds(1000));
				mTest->GetSpectr_10KHz(&mControl);

			}
			
			inch = getch();
			mTest->StopSpectr();


			mStr.Type = DemodFMS;
			mStr.freq = 99200000;
			mStr.SampleRate = 10240000;
//			mStr.SampleRate = 20480000;

			mTest->StartDevice(mStr.SampleRate);
			demod = mTest->SetModem(&mStr);
			
			
			mTest->SetAudioSampleRate(demod, 48000);
			std::this_thread::sleep_for(std::chrono::milliseconds(200));
			std::cout <<"AudioSampleRate: "<< mTest->GetAudioSampleRate(demod) << std::endl;
			
			auto t1 = std::chrono::high_resolution_clock::now();
			auto t2 = t1;

			while (!_kbhit()) {
				gSize = mTest->GetIQData(&iqData);
				t2 = std::chrono::high_resolution_clock::now();
				auto timePeriod = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
				if (timePeriod >= 2000)
				{
					if (mStr.freq < 108000000) {
						mStr.freq += 100000;
					} else 
						mStr.freq = 99200000;

					demod = mTest->SetModem(&mStr);
				}
			}
		}

	}
	
	mTest->Release();
	std::cout << (cnt /100) << std::endl;
	std::cout << "audioGain " << audioGain << std::endl;
	std::cout << "Hello World!\n";

#endif
}



