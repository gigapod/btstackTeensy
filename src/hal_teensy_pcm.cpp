#include <Arduino.h>
#include <Audio.h>
#include <string.h>

#include "hal_teensy_pcm.h"

namespace {

AudioInputI2S2slave g_pcm_in;
AudioOutputI2S2slave g_pcm_out;
AudioRecordQueue g_record_queue;
AudioPlayQueue g_play_queue;
AudioConnection g_patch_in(g_pcm_in, 0, g_record_queue, 0);
AudioConnection g_patch_out(g_play_queue, 0, g_pcm_out, 0);

} // namespace

extern "C" {

void pcm_bridge_start(int wide_band) {
    (void)wide_band; // sample rate follows whatever clock the module drives on BT_PCM_CLK/SYNC
    AudioMemory(8);
    g_record_queue.begin();
}

void pcm_bridge_stop(void) {
    g_record_queue.end();
    g_record_queue.clear();
    // AudioPlayQueue has no stop(): it goes silent once its queue drains.
}

uint16_t pcm_bridge_read(int16_t *out, uint16_t max_samples) {
    uint16_t written = 0;
    while (written + 128 <= max_samples && g_record_queue.available() > 0) {
        int16_t *block = g_record_queue.readBuffer();
        memcpy(&out[written], block, 128 * sizeof(int16_t));
        g_record_queue.freeBuffer();
        written += 128;
    }
    return written;
}

void pcm_bridge_write(const int16_t *samples, uint16_t num_samples) {
    uint16_t offset = 0;
    while (offset < num_samples) {
        int16_t *block = g_play_queue.getBuffer();
        if (block == nullptr) {
            break; // queue full -- drop the remainder rather than block real time audio
        }
        uint16_t chunk = num_samples - offset;
        if (chunk >= 128) {
            memcpy(block, &samples[offset], 128 * sizeof(int16_t));
        } else {
            memcpy(block, &samples[offset], chunk * sizeof(int16_t));
            memset(&block[chunk], 0, (128 - chunk) * sizeof(int16_t));
        }
        g_play_queue.playBuffer();
        offset += 128;
    }
}

} // extern "C"
