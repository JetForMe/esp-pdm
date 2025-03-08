#include "ESPPDM.h"

#include <cstdio>

#include <driver/i2s_pdm.h>
#include <esp_attr.h>
#include <freertos/ringbuf.h>





PDMIn::PDMIn(gpio_num_t inClockPin, gpio_num_t inDataPin)
	:
	mClockPin(inClockPin),
	mDataPin(inDataPin),
	mChannel(nullptr),
	mBitDepth(I2S_DATA_BIT_WIDTH_16BIT),
	mBuffer(nullptr)
{
}


bool
IRAM_ATTR
PDMIn::start(uint32_t inSampleRate, i2s_data_bit_width_t inBitDepth, bool inMono, size_t inBufferSize)
{
	mBitDepth = inBitDepth;
	mBuffer = ::xRingbufferCreate(inBufferSize * (inBitDepth / 8), RINGBUF_TYPE_BYTEBUF);
	if (mBuffer == nullptr)
	{
		std::printf("Unable to create ring buffer\n");
		return false;
	}
	
	//	Create the channel and set the DMA buffer as large as it can be…

	i2s_slot_mode_t slotMode = inMono ? I2S_SLOT_MODE_MONO : I2S_SLOT_MODE_STEREO;
    i2s_chan_config_t chanConfig = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
	chanConfig.dma_frame_num = 4092 / (slotMode * inBitDepth / 8);
	
	esp_err_t err = ::i2s_new_channel(&chanConfig, NULL, &mChannel);
	if (err != ESP_OK)
	{
		std::printf("i2s_new_channel() failed with %d: %s\n", err, esp_err_to_name(err));
		return false;
	}

	//	Configure it…

    i2s_pdm_rx_config_t config =
    {
        .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(inSampleRate),
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(inBitDepth, slotMode),
        .gpio_cfg =
        {
            .clk = mClockPin,
            .din = mDataPin,
            .invert_flags =
            {
                .clk_inv = false,
            },
        },
    };

    err = ::i2s_channel_init_pdm_rx_mode(mChannel, &config);
	if (err != ESP_OK)
	{
		std::printf("i2s_channel_init_pdm_rx_mode() failed with %d: %s\n", err, esp_err_to_name(err));
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
	
	err = ::i2s_channel_register_event_callback(mChannel, &cbs, this);
	if (err != ESP_OK)
	{
		std::printf("i2s_channel_register_event_callback failed with %d: %s\n", err, esp_err_to_name(err));
		return false;
	}

	//	Enable the channel (this starts DMA)…

	err = ::i2s_channel_enable(mChannel);
	if (err != ESP_OK)
	{
		std::printf("i2s_channel_enable() failed with %d: %s\n", err, esp_err_to_name(err));
		return false;
	}

	return true;
}

void
PDMIn::stop()
{
	::i2s_channel_disable(mChannel);
	::i2s_del_channel(mChannel);
}

bool
IRAM_ATTR
PDMIn::callback(i2s_event_data_t* inEvent)
{
	std::printf("Audio callback\n");
	const uint8_t* buffer = static_cast<const uint8_t*>(inEvent->dma_buf);
	(void) ::xRingbufferSendFromISR(mBuffer, buffer, inEvent->size, NULL);

	return false;
}

bool
IRAM_ATTR
PDMIn::rxQueueOverflowCallback(i2s_chan_handle_t inChannel, i2s_event_data_t* inEvent, void* inCtx)
{
	PDMIn* self = reinterpret_cast<PDMIn*>(inCtx);
	return self->callback(inEvent);
}

size_t
PDMIn::samplesAvailable() const
{
	unsigned int bytesAvailable = 0;
	::vRingbufferGetInfo(mBuffer, NULL, NULL, NULL, NULL, &bytesAvailable);
	
	return bytesAvailable / (mBitDepth / 8);
}

void*
PDMIn::getSamples(size_t& outCount, size_t inMaxCount, uint32_t inTimeout)
{
	size_t maxBytesToRead = inMaxCount * (mBitDepth / 8);
	size_t bytesRead = 0;
	void* bytes = ::xRingbufferReceiveUpTo(mBuffer, &bytesRead, pdMS_TO_TICKS(inTimeout), maxBytesToRead);
	outCount = bytesRead / (mBitDepth / 8);
	
	//	Because `xRingbufferReceiveUpTo` returns nothing if it times out (annoyingly),
	//	make another call for whatever’s available…
	
	if (bytes == NULL)
	{
		unsigned int bytesAvailable = 0;
		bytesRead = 0;
		::vRingbufferGetInfo(mBuffer, NULL, NULL, NULL, NULL, &bytesAvailable);
		bytes = ::xRingbufferReceiveUpTo(mBuffer, &bytesRead, pdMS_TO_TICKS(inTimeout), bytesAvailable);
		outCount = bytesRead / (mBitDepth / 8);
	}
	
	return bytes;
}

void
PDMIn::deleteBytes(void* inBytes)
{
	if (inBytes != NULL)
	{
		::vRingbufferReturnItem(mBuffer, inBytes);
	}
}

