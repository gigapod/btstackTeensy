#ifndef HAL_TEENSY_PCM_H
#define HAL_TEENSY_PCM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// HFP voice audio bridge over the Murata 1YN's 4-wire PCM interface (SAI2 /
// "I2S2" in Teensy Audio Library terms, slave mode -- the module drives
// BT_PCM_CLK/BT_PCM_SYNC, see README.md). This is intentionally decoupled
// from A2DP's I2S1 path (hal_audio_teensy.cpp) -- wire the two together (or
// to your own codec) in application code as your product needs.
//
// Call pcm_bridge_start() when a SCO/eSCO audio connection comes up (voice
// call answered) and pcm_bridge_stop() when it's torn down. wide_band
// selects the BCM PCM clock/sample rate pairing that must match
// ENABLE_BCM_PCM_WBS / the codec negotiated over HFP (CVSD = 8 kHz narrow
// band, mSBC = 16 kHz wide band).
void pcm_bridge_start(int wide_band);
void pcm_bridge_stop(void);

// Pull audio arriving FROM the module (the remote party's voice during a
// call). Returns the number of int16 mono samples written to `out`
// (0..max_samples). Route this to whatever local speaker path you have.
uint16_t pcm_bridge_read(int16_t *out, uint16_t max_samples);

// Push audio TO the module (e.g. from a local microphone) to be sent to the
// far end. `samples` must be a multiple of the Teensy Audio Library's block
// size (128) for best results; partial blocks are zero-padded.
void pcm_bridge_write(const int16_t *samples, uint16_t num_samples);

#ifdef __cplusplus
}
#endif

#endif // HAL_TEENSY_PCM_H
