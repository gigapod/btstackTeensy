// hal_audio.h sink implementation for A2DP playback, driving a Teensy Audio
// Shield (or any I2S DAC) on I2S1 -- the *standard* Teensy audio pins, e.g.
// SGTL5000 Audio Shield. This is deliberately a completely separate I2S bus
// from the BT_PCM_* pins (I2S2 / SAI2), which are reserved for HFP voice
// audio to/from the module -- see hal_teensy_pcm.cpp. If you don't have an
// I2S DAC wired to I2S1, A2DP will still negotiate and decode SBC frames
// fine; the audio just has nowhere to go.
//
// Double-buffered like BTstack's own port/stm32-f4discovery* hal_audio
// implementations: an IntervalTimer stands in for the DMA transfer-complete
// IRQ that hardware audio HALs normally use to pace playback, called back
// into btstack_audio_embedded.c's callback (an expected/supported call
// context -- see btstack_run_loop_poll_data_sources_from_irq in
// platform/embedded/btstack_audio_embedded.c), and the actual buffer fill
// happens on the next main-loop iteration, not in the ISR.
//
// Caveat: I2S1 runs at the Teensy Audio Library's fixed native rate
// (~44.1 kHz); there's no resampler wired in here, so A2DP sources that
// negotiate SBC at 48/32/16 kHz instead of 44.1 kHz will play back at the
// wrong pitch/speed.

#include <Arduino.h>
#include <Audio.h>

extern "C" {
#include "hal_audio.h"
}

namespace {

constexpr unsigned kNumBuffers = 2;
constexpr unsigned kBufferFrames = AUDIO_BLOCK_SAMPLES; // 128

int16_t g_buffer[kNumBuffers][kBufferFrames * 2]; // interleaved stereo

void (*g_buffer_played_callback)(uint8_t buffer_index) = nullptr;
volatile unsigned g_play_index = 0;
volatile bool g_running = false;

AudioPlayQueue g_queue_left;
AudioPlayQueue g_queue_right;
AudioOutputI2S g_i2s_out;
AudioConnection g_patch_left(g_queue_left, 0, g_i2s_out, 0);
AudioConnection g_patch_right(g_queue_right, 0, g_i2s_out, 1);
IntervalTimer g_sink_timer;

void sink_timer_isr() {
    unsigned index = g_play_index;

    int16_t *left = g_queue_left.getBuffer();
    int16_t *right = g_queue_right.getBuffer();
    if (left != nullptr && right != nullptr) {
        const int16_t *interleaved = g_buffer[index];
        for (unsigned i = 0; i < kBufferFrames; i++) {
            left[i] = interleaved[2 * i + 0];
            right[i] = interleaved[2 * i + 1];
        }
        g_queue_left.playBuffer();
        g_queue_right.playBuffer();
    }

    g_play_index = (index + 1) % kNumBuffers;
    if (g_buffer_played_callback != nullptr) {
        g_buffer_played_callback((uint8_t)index);
    }
}

} // namespace

extern "C" {

void hal_audio_sink_init(uint8_t channels, uint32_t sample_rate, void (*buffer_played_callback)(uint8_t buffer_index)) {
    (void)channels; // always rendered as stereo -- see hal_audio_sink_get_num_output_buffers()
    (void)sample_rate;
    AudioMemory(8);
    g_buffer_played_callback = buffer_played_callback;
    g_play_index = 0;
}

uint32_t hal_audio_sink_get_frequency(void) {
    // Native Teensy Audio Library block rate; see file header caveat.
    return (uint32_t)AUDIO_SAMPLE_RATE_EXACT;
}

uint16_t hal_audio_sink_get_num_output_buffers(void) {
    return kNumBuffers;
}

uint16_t hal_audio_sink_get_num_output_buffer_samples(void) {
    return kBufferFrames;
}

int16_t *hal_audio_sink_get_output_buffer(uint8_t buffer_index) {
    return g_buffer[buffer_index];
}

void hal_audio_sink_start(void) {
    if (g_running) {
        return;
    }
    g_running = true;
    g_play_index = 0;
    // Period of one AUDIO_BLOCK_SAMPLES block at the native sample rate.
    float period_us = 1000000.0f * (float)kBufferFrames / AUDIO_SAMPLE_RATE_EXACT;
    g_sink_timer.begin(sink_timer_isr, period_us);
}

void hal_audio_sink_stop(void) {
    g_sink_timer.end();
    g_running = false;
    // AudioPlayQueue has no stop(): it just goes silent once its queue drains,
    // which happens naturally once the ISR above stops feeding it.
}

void hal_audio_sink_close(void) {
    hal_audio_sink_stop();
    g_buffer_played_callback = nullptr;
}

} // extern "C"
