#pragma once


#include <Arduino.h>
#include <driver/i2s_pdm.h>
#include <esp_attr.h>
#include <freertos/ringbuf.h>



class
PDMIn
{
public:
					PDMIn(gpio_num_t inClockPin, gpio_num_t inDataPin);

					/**
						@param	inBufferSize	The size, in samples, of the input ring buffer to read samples into.
					*/

	bool			start(uint32_t inSampleRate, i2s_data_bit_width_t inBitDepth, bool inMono, size_t inBufferSize = 32 * 1000);
	
	void			stop();
	
	size_t			samplesAvailable() const;
	
					/**
						Retrieves up to ``inMaxCount`` samples of data. The pointer
						returned by this method must be deallocated with a call
						to ``deleteBytes()``.
						
						If the call times out, whatever bytes are available are returned.
						
						@param	outCount		Returns the number of samples retrieved.
						@param	inMaxCount		Limits the maximum number of samples retrieved.
						@param	inTimeout		How long to wait for the desired number of samples in ms.
					*/
					
	void*			getSamples(size_t& outCount, size_t inMaxCount, uint32_t inTimeout);
					
					/**
						Delete the bytes allocated by ``getSamples()``.
					*/
					
	void			deleteBytes(void* inBytes);
	
protected:
	virtual bool	callback(i2s_event_data_t* inEvent);
	
private:
	static	bool  IRAM_ATTR rxQueueOverflowCallback(i2s_chan_handle_t inChannel, i2s_event_data_t* inEvent, void* inCtx);


public:
	gpio_num_t					mClockPin;
	gpio_num_t					mDataPin;
	i2s_chan_handle_t 			mChannel;
	i2s_data_bit_width_t		mBitDepth;
	RingbufHandle_t				mBuffer;
};
