/**
 * Audio System design tool code
 */

#ifndef AUDIOSYSTEM_H
#define AUDIOSYSTEM_H


#include <Audio.h>
#include <i2c_t3.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>


// GUItool: begin automatically generated code
AudioSynthWaveform       LFO1;      //xy=55,76.00000381469727
AudioAmplifier           Volume;           //xy=122.00000762939453,552.0000076293945
AudioEffectFade          vel_fade_midi;          //xy=175.0000228881836,343.0000057220459
AudioEffectFade          vel_change_low;          //xy=179.00001525878906,274.0000047683716
AudioEffectFade          vel_change_high;          //xy=180.00000762939453,309.00000381469727
AudioEffectEnvelope      env_pitch;      //xy=182,76
AudioEffectDelay         delay_fx;         //xy=266.4444389343262,595.5555419921875
AudioSynthKarplusStrong  pink;        //xy=331,153.00000190734863
AudioSynthWaveformModulated OSC1;   //xy=335.00000381469727,26.000003337860107
AudioSynthWaveformModulated OSC2;   //xy=337.00000762939453,67.00000667572021
AudioSynthWaveformModulated OSC3;   //xy=337,110.00001430511475
AudioSynthWaveform       LFO_ring;      //xy=345.00000762939453,351.0000057220459
AudioSynthWaveformDc     dc_ring;            //xy=345.00000762939453,392.0000057220459
AudioMixer4              velocity_env;         //xy=379.00000762939453,296.00000381469727
AudioMixer4              mix_delay1;         //xy=417.00000381469727,555.7999954223633
AudioMixer4              mix_delay2;         //xy=419.00000381469727,623.7999954223633
AudioMixer4              mix_ring_wet;         //xy=493,381
AudioEffectMultiply      ring;      //xy=548.0000076293945,294.00000381469727
AudioMixer4              mix_osc;         //xy=549.0000076293945,66.00000762939453
AudioSynthWaveform       LFO2;      //xy=608.0000152587891,445.22223234176636
AudioMixer4              mix_delay_wet;         //xy=610.4444274902344,517.466682434082
AudioEffectEnvelope      env_filter;      //xy=668.0000114440918,338.0000057220459
AudioAmplifier           filter_attenuator;           //xy=704.0000114440918,296.00000381469727
AudioOutputI2S           i2s2;           //xy=749.7500133514404,33.416707038879395
AudioEffectFreeverbStereo freeverbs;     //xy=772.5555686950684,516.000020980835
AudioEffectEnvelope      ADSR_vol;      //xy=826.0000114440918,93
AudioFilterStateVariable LPF;        //xy=866.0000133514404,306.0000057220459
AudioConnection          patchCord1(LFO1, env_pitch);
AudioConnection          patchCord2(Volume, delay_fx);
AudioConnection          patchCord3(Volume, 0, mix_delay_wet, 0);
AudioConnection          patchCord4(vel_fade_midi, 0, velocity_env, 2);
AudioConnection          patchCord5(vel_change_low, 0, velocity_env, 0);
AudioConnection          patchCord6(vel_change_high, 0, velocity_env, 1);
AudioConnection          patchCord7(delay_fx, 0, mix_delay1, 0);
AudioConnection          patchCord8(delay_fx, 1, mix_delay1, 1);
AudioConnection          patchCord9(delay_fx, 2, mix_delay1, 2);
AudioConnection          patchCord10(delay_fx, 3, mix_delay1, 3);
AudioConnection          patchCord11(delay_fx, 4, mix_delay2, 0);
AudioConnection          patchCord12(delay_fx, 5, mix_delay2, 1);
AudioConnection          patchCord13(pink, 0, mix_osc, 3);
AudioConnection          patchCord14(OSC1, 0, mix_osc, 0);
AudioConnection          patchCord15(OSC2, 0, mix_osc, 1);
AudioConnection          patchCord16(OSC3, 0, mix_osc, 2);
AudioConnection          patchCord17(LFO_ring, 0, mix_ring_wet, 0);
AudioConnection          patchCord18(dc_ring, 0, mix_ring_wet, 1);
AudioConnection          patchCord19(velocity_env, 0, ring, 0);
AudioConnection          patchCord20(mix_delay1, 0, mix_delay_wet, 1);
AudioConnection          patchCord21(mix_delay2, 0, mix_delay_wet, 2);
AudioConnection          patchCord22(mix_ring_wet, 0, ring, 1);
AudioConnection          patchCord23(ring, filter_attenuator);
AudioConnection          patchCord24(mix_osc, 0, i2s2, 0);
AudioConnection          patchCord25(mix_osc, 0, i2s2, 1);
AudioConnection          patchCord26(LFO2, env_filter);
AudioConnection          patchCord27(mix_delay_wet, freeverbs);
AudioConnection          patchCord28(env_filter, 0, LPF, 1);
AudioConnection          patchCord29(filter_attenuator, 0, LPF, 0);
AudioConnection          patchCord30(ADSR_vol, vel_change_low);
AudioConnection          patchCord31(ADSR_vol, vel_change_high);
AudioConnection          patchCord32(ADSR_vol, vel_fade_midi);
AudioConnection          patchCord33(LPF, 0, Volume, 0);
AudioControlSGTL5000     sgtl5000_1;     //xy=885.0000076293945,183.00000762939453
// GUItool: end automatically generated code

#endif  //AUDIOSYSTEM_H
