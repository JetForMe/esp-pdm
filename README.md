# ESP PDM for Arduino

**NOTE:** *This library is not yet ready for use.*

A simple asynchronous PDM input library for ESP microcontrollers.

# Usage

```c++
#include <ArduinoESPPDM.h>

PDMIn				gPDMIn(GPIO_NUM_10, GPIO_NUM_11);



void
setup()
{
	bool success = gPDMIn.start(16 * 1000, I2S_DATA_BIT_WIDTH_16BIT, 10 * 16 * 1000);
}

void
loop()
{
	const size_t	kSampleCount = 4 * 1024;
	
	unsigned int samplesAvailable = gPDMIn.samplesAvailable();
	if (samplesAvailable >= kSampleCount)
	{
		//	Wait up to 1000 ms for the buffer to fill and get some data…
		
		size_t samplesRead = 0;
		void* bytes = gPDMIn.getSamples(&samplesRead, kSampleCount, pdMS_TO_TICKS(1000));
		
		//	Dump the sample values to the serial port. Use the Arduino IDE
		//	Serial Plotter to see a crude graph…
		
		const int16_t* data = static_cast<const int16_t*>(bytes);
		for (int i = 0; i < samplesRead; i++)
		{
			Serial.println(data[i]);
		}
		
		//	Deallocate the bytes allocated by getSamples()…
		
		gPDMIn.deleteBytes(bytes);
	}
}
```
