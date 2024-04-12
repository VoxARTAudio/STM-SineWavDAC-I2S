/*
 * chorus.h
 *
 *  Created on: 30 wrz 2023
 *      Author: kwarc
 */

#ifndef CHORUS_H_
#define CHORUS_H_

#include "effect_interface.h"

/* Include necessary DSP libraries and dependencies */
#include "audio_dsp.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Define constants and structures */

/* Define chorus attribute controls */
typedef struct {
    float depth;
    float rate;
    float tone;
    float mix;
    int mode; // Using integer to represent mode_type
} chorus_attr_controls;

/* Define effect-specific attributes */
typedef struct {
    chorus_attr_controls ctrl;
} chorus_attr;

/* Define chorus effect structure */
typedef struct {
    effect_id effect_type;
    oscillator lfo1;
    oscillator lfo2;
    unicomb unicomb1;
    unicomb unicomb2;
    chorus_attr attr;
} chorus;

/* Function prototypes */

/* Initialize chorus effect */
void chorus_init(chorus *chorus_effect, float depth, float rate, float tone, float mix);

/* Process audio through the chorus effect */
void chorus_process(const chorus *chorus_effect, const dsp_input *in, dsp_output *out);

/* Get specific attributes of the chorus effect */
const chorus_attr *chorus_get_specific_attributes(const chorus *chorus_effect);

/* Set depth parameter of the chorus effect */
void chorus_set_depth(chorus *chorus_effect, float depth);

/* Set rate parameter of the chorus effect */
void chorus_set_rate(chorus *chorus_effect, float rate);

/* Set tone parameter of the chorus effect */
void chorus_set_tone(chorus *chorus_effect, float tone);

/* Set mix parameter of the chorus effect */
void chorus_set_mix(chorus *chorus_effect, float mix);

/* Set mode parameter of the chorus effect */
void chorus_set_mode(chorus *chorus_effect, int mode);

#ifdef __cplusplus
}
#endif

#endif /* CHORUS_H_ */

