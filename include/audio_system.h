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
AudioSynthWaveform       LFO1;      //xy=59,30.0000057220459
AudioEffectEnvelope      env_pitch;      //xy=90.00000381469727,85.00000667572021
AudioEffectFade          vel_fade_midi;          //xy=117.0000228881836,320.0000057220459
AudioEffectFade          vel_change_low;          //xy=121.00001525878906,251.00000476837158
AudioEffectFade          vel_change_high;          //xy=122.00000762939453,286.00000381469727
AudioMixer4              mix_delay;         //xy=169.00001525878906,682.8000087738037
AudioMixer4              env_pitch_onoff;         //xy=250.00000381469727,82.00000762939453
AudioEffectDelay         delay_fx;         //xy=291.4444351196289,561.5555725097656
AudioSynthWaveform       LFO_ring;      //xy=358.0000305175781,374.0000057220459
AudioSynthWaveform       LFO2;      //xy=358.00000762939453,426.22222328186035
AudioSynthWaveformDc     dc_ring;            //xy=359.00000762939453,330.0000057220459
AudioMixer4              velocity_env;         //xy=372.00000762939453,267.00000381469727
AudioMixer4              mix_delay_wet;         //xy=382.4444351196289,701.4667587280273
AudioSynthNoisePink      pink;          //xy=416.00000381469727,149.00000190734863
AudioSynthWaveformModulated OSC1;   //xy=426.00000762939453,24.000003814697266
AudioSynthWaveformModulated OSC2;   //xy=429.0000114440918,66.00000667572021
AudioSynthWaveformModulated OSC3;   //xy=429.00000381469727,109.00001430511475
AudioAmplifier           Volume;           //xy=470.00000762939453,632.0000581741333
AudioEffectFreeverbStereo freeverbs;     //xy=500.00000762939453,795.0000114440918
AudioEffectEnvelope      env_filter;      //xy=532.0000076293945,442.0000057220459
AudioSynthWaveform       lfoVca;      //xy=550.0000076293945,236.00000381469727
AudioEffectMultiply      vca;      //xy=586.0000076293945,180.00000190734863
AudioMixer4              mix_ring_wet;         //xy=598.0000076293945,291.00000381469727
AudioMixer4              env_filter_onoff;         //xy=599.0000076293945,367.0000057220459
AudioMixer4              mix_osc;         //xy=616.0000114440918,40.0000057220459
AudioMixer4              mix_verb_wet_l;         //xy=738.0000076293945,714.0000095367432
AudioMixer4              mix_verb_wet_r;        //xy=741.0000076293945,793.0000114440918
AudioEffectEnvelope      ADSR_vol;      //xy=771.0000114440918,36.000003814697266
AudioEffectMultiply      ring;      //xy=789.0000076293945,215.00000381469727
AudioFilterStateVariable LPF;        //xy=806.0000114440918,414.0000057220459
AudioAmplifier           filter_attenuator;           //xy=862.0000152587891,284.00000381469727
AudioOutputI2S           i2s2;           //xy=882.7500114440918,615.4167079925537
AudioConnection          patchCord1(LFO1, env_pitch);
AudioConnection          patchCord2(LFO1, 0, env_pitch_onoff, 0);
AudioConnection          patchCord3(env_pitch, 0, env_pitch_onoff, 1);
AudioConnection          patchCord4(vel_fade_midi, 0, velocity_env, 2);
AudioConnection          patchCord5(vel_change_low, 0, velocity_env, 0);
AudioConnection          patchCord6(vel_change_high, 0, velocity_env, 1);
AudioConnection          patchCord7(mix_delay, 0, mix_delay_wet, 1);
AudioConnection          patchCord8(env_pitch_onoff, 0, OSC1, 0);
AudioConnection          patchCord9(env_pitch_onoff, 0, OSC2, 0);
AudioConnection          patchCord10(env_pitch_onoff, 0, OSC3, 0);
AudioConnection          patchCord11(delay_fx, 0, mix_delay, 0);
AudioConnection          patchCord12(delay_fx, 1, mix_delay, 1);
AudioConnection          patchCord13(delay_fx, 2, mix_delay, 2);
AudioConnection          patchCord14(delay_fx, 3, mix_delay, 3);
AudioConnection          patchCord15(LFO_ring, 0, mix_ring_wet, 1);
AudioConnection          patchCord16(LFO2, env_filter);
AudioConnection          patchCord17(LFO2, 0, env_filter_onoff, 0);
AudioConnection          patchCord18(dc_ring, 0, mix_ring_wet, 0);
AudioConnection          patchCord19(velocity_env, 0, vca, 0);
AudioConnection          patchCord20(mix_delay_wet, Volume);
AudioConnection          patchCord21(pink, 0, mix_osc, 3);
AudioConnection          patchCord22(OSC1, 0, mix_osc, 0);
AudioConnection          patchCord23(OSC2, 0, mix_osc, 1);
AudioConnection          patchCord24(OSC3, 0, mix_osc, 2);
AudioConnection          patchCord25(Volume, freeverbs);
AudioConnection          patchCord26(Volume, 0, mix_verb_wet_l, 0);
AudioConnection          patchCord27(Volume, 0, mix_verb_wet_r, 0);
AudioConnection          patchCord28(freeverbs, 0, mix_verb_wet_l, 1);
AudioConnection          patchCord29(freeverbs, 1, mix_verb_wet_r, 1);
AudioConnection          patchCord30(env_filter, 0, env_filter_onoff, 1);
AudioConnection          patchCord31(lfoVca, 0, vca, 1);
AudioConnection          patchCord32(vca, 0, ring, 0);
AudioConnection          patchCord33(mix_ring_wet, 0, ring, 1);
AudioConnection          patchCord34(env_filter_onoff, 0, LPF, 1);
AudioConnection          patchCord35(mix_osc, ADSR_vol);
AudioConnection          patchCord36(mix_verb_wet_l, 0, i2s2, 0);
AudioConnection          patchCord37(mix_verb_wet_r, 0, i2s2, 1);
AudioConnection          patchCord38(ADSR_vol, vel_change_low);
AudioConnection          patchCord39(ADSR_vol, vel_change_high);
AudioConnection          patchCord40(ADSR_vol, vel_fade_midi);
AudioConnection          patchCord41(ring, filter_attenuator);
AudioConnection          patchCord42(LPF, 0, delay_fx, 0);
AudioConnection          patchCord43(LPF, 0, mix_delay_wet, 0);
AudioConnection          patchCord44(filter_attenuator, 0, LPF, 0);
AudioControlSGTL5000     sgtl5000_1;     //xy=719.0000114440918,562.0000085830688
// GUItool: end automatically generated code

#endif  //AUDIOSYSTEM_H
