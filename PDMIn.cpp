#include "PDMIn.h"

#include <driver/i2s_pdm.h>
#include <esp_attr.h>
#include <freertos/ringbuf.h>





PDMIn::PDMIn()
	:
	mChannel(NULL),
	mBitDepth(I2S_DATA_BIT_WIDTH_16BIT),
	mBuffer(NULL)
{
}


bool
PDMIn::start(gpio_num_t inClkPin, gpio_num_t inDataPin, uint32_t inSampleRate, i2s_data_bit_width_t inBitDepth, size_t inBufferSize)
{
	mBitDepth = inBitDepth;
	mBuffer = ::xRingbufferCreate(inBufferSize * (inBitDepth / 8), RINGBUF_TYPE_BYTEBUF);

	//	Create the channel…

	bool mono = true;
	i2s_slot_mode_t slotMode = mono ? I2S_SLOT_MODE_MONO : I2S_SLOT_MODE_STEREO;
	
    i2s_chan_config_t chanConfig = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
	chanConfig.dma_frame_num = 4092 / (slotMode * inBitDepth / 8);
	esp_err_t err = i2s_new_channel(&chanConfig, NULL, &mChannel);
	if (err != ESP_OK)
	{
		Serial.printf("i2s_new_channel() failed with %d\n", err);
		return false;
	}

	//	Configure it…

    i2s_pdm_rx_config_t config = {
        .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(inSampleRate),
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(inBitDepth, slotMode),
        .gpio_cfg =
        {
            .clk = inClkPin,
            .din = inDataPin,
            .invert_flags =
            {
                .clk_inv = false,
            },
        },
    };

    err = i2s_channel_init_pdm_rx_mode(mChannel, &config);
	if (err != ESP_OK)
	{
		Serial.printf("i2s_channel_init_pdm_rx_mode() failed with %d\n", err);
		return false;
	}

	//	Set up the input buffer overflow callback…

	i2s_event_callbacks_t cbs =
	{
		.on_recv = NULL,
		.on_recv_q_ovf = rxQueueOverflowCallback,
		.on_sent = NULL,
		.on_send_q_ovf = NULL,
	};
	
	err = i2s_channel_register_event_callback(mChannel, &cbs, this);
	if (err != ESP_OK)
	{
		Serial.printf("i2s_channel_register_event_callback failed with %d\n", err);
		return false;
	}

	//	Enable the channel (this starts DMA)…

	err = i2s_channel_enable(mChannel);
	if (err != ESP_OK)
	{
		Serial.printf("i2s_channel_enable() failed with %d\n", err);
		return false;
	}

	return true;
}

bool
PDMIn::callback(i2s_event_data_t* inEvent)
{
	const uint8_t* buffer = static_cast<const uint8_t*>(inEvent->dma_buf);
	BaseType_t success = ::xRingbufferSendFromISR(mBuffer, buffer, inEvent->size, NULL);

	return false;
}

bool
PDMIn::rxQueueOverflowCallback(i2s_chan_handle_t inChannel, i2s_event_data_t* inEvent, void* inCtx)
{
	PDMIn* self = reinterpret_cast<PDMIn*>(inCtx);
	return self->callback(inEvent);
}

size_t
PDMIn::samplesAvailable() const
{
	unsigned int bytesAvailable = 0;
	vRingbufferGetInfo(gPDMIn.mBuffer, NULL, NULL, NULL, NULL, &bytesAvailable);
	
	return bytesAvailable / (mBitDepth / 8);
}

void*
PDMIn::getSamples(size_t& outCount, size_t inMaxCount, TickType_t inTimeout)
{
	size_t maxBytesToRead = inMaxCount * (mBitDepth / 8);
	size_t bytesRead = 0;
	void* bytes = ::xRingbufferReceiveUpTo(mBuffer, &bytesRead, inTimeout, maxBytesToRead);
	
	outCount = bytesRead / (mBitDepth / 8);
	
	return bytes;
}

void
PDMIn::deleteBytes(void* inBytes)
{
	::vRingbufferReturnItem(mBuffer, inBytes);
}

