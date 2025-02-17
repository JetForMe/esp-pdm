#include <Arduino.h>

#include "PDMIn.h"


PDMIn				gPDMIn;


void
setup()
{
	Serial.begin(115200);
	uint32_t timeout = 5000;
	uint32_t startTime = millis();
	while (!Serial && millis() - startTime < timeout)
	{
		continue;
	}
	Serial.println("PDM Input Demo");
	
	//	Blink the LED to say “hello…”

	pinMode(LED_BUILTIN, OUTPUT);
	for (int i = 0; i < 3; i++)
	{
		digitalWrite(LED_BUILTIN, HIGH);
		delay(100);
		digitalWrite(LED_BUILTIN, LOW);
		delay(100);
	}
	
	//	Setup PDM. 16 kHz, 16-bit samples, with a buffer large enough
	//	for 10 seconds of data. The clock and data pins correspond to
	//	the A3 and A4 pins on the Matrix Portal S3…
	
	bool success = gPDMIn.start(GPIO_NUM_10, GPIO_NUM_11, 16 * 1000, I2S_DATA_BIT_WIDTH_16BIT, 10 * 16 * 1000);
	if (!success)
	{
		Serial.println("PDM init failed");
	}
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
		
		const int16_t* data = static_cast<const int16_t*>(bytes);
		for (int i = 0; i < samplesRead; i++)
		{
			Serial.println(data[i]);
		}
		
		gPDMIn.deleteBytes(bytes);
	}

	digitalWrite(LED_BUILTIN, HIGH);
	delay(10);
	digitalWrite(LED_BUILTIN, LOW);
	delay(10);
}
