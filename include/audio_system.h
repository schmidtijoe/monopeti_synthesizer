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
AudioSynthWaveform       combine_LFO;      //xy=84,558
AudioEffectEnvelope      env_pitch;      //xy=90.00000381469727,85.00000667572021
AudioEffectDigitalCombine combine;       //xy=116.00000762939453,504.0000066757202
AudioEffectFade          vel_fade_midi;          //xy=117.0000228881836,320.0000057220459
AudioEffectFade          vel_change_low;          //xy=121.00001525878906,251.00000476837158
AudioEffectFade          vel_change_high;          //xy=122.00000762939453,286.00000381469727
AudioMixer4              mix_delay_wet;         //xy=162.4444351196289,787.4667587280273
AudioMixer4              mix_delay;         //xy=169.00001525878906,682.8000087738037
AudioMixer4              env_pitch_onoff;         //xy=237,35
AudioEffectDelay         delay_fx;         //xy=291.4444351196289,561.5555725097656
AudioSynthNoisePink      pink;          //xy=310,144.00000190734863
AudioSynthWaveformModulated OSC3;   //xy=359.00000381469727,107.00001335144043
AudioSynthWaveform       LFO_ring;      //xy=358.0000305175781,374.0000057220459
AudioSynthWaveform       LFO2;      //xy=358.00000762939453,426.22222328186035
AudioSynthWaveformDc     dc_ring;            //xy=359.00000762939453,330.0000057220459
AudioMixer4              velocity_env;         //xy=372.00000762939453,267.00000381469727
AudioMixer4              mix_combine_wet;         //xy=397.0000305175781,694.0000095367432
AudioSynthWaveformModulated OSC2;   //xy=408.00000381469727,67.00000667572021
AudioSynthWaveformModulated OSC1;   //xy=412.0000057220459,24.000003814697266
AudioEffectFreeverbStereo freeverbs;     //xy=500.00000762939453,795.0000114440918
AudioEffectEnvelope      env_filter;      //xy=532.0000076293945,442.0000057220459
AudioSynthWaveform       lfoVca;      //xy=548.0000076293945,230.00000381469727
AudioAmplifier           Volume;           //xy=564.0000076293945,643.0000591278076
AudioEffectMultiply      vca;      //xy=586.0000076293945,180.00000190734863
AudioMixer4              mix_ring_wet;         //xy=598.0000076293945,291.00000381469727
AudioMixer4              env_filter_onoff;         //xy=599.0000076293945,367.0000057220459
AudioMixer4              mix_osc;         //xy=616.0000114440918,40.0000057220459
AudioMixer4              mix_verb_wet_l;         //xy=738.0000076293945,714.0000095367432
AudioMixer4              mix_verb_wet_r;        //xy=741.0000076293945,793.0000114440918
AudioEffectEnvelope      ADSR_vol;      //xy=771.0000114440918,36.000003814697266
AudioEffectMultiply      ring;      //xy=789.0000076293945,215.00000381469727
AudioFilterStateVariable LPF;        //xy=798.0000114440918,443.0000057220459
AudioAmplifier           filter_attenuator;           //xy=862.0000152587891,284.00000381469727
AudioOutputI2S           i2s2;           //xy=882.7500114440918,615.4167079925537
AudioConnection          patchCord1(LFO1, env_pitch);
AudioConnection          patchCord2(LFO1, 0, env_pitch_onoff, 0);
AudioConnection          patchCord3(combine_LFO, 0, combine, 1);
AudioConnection          patchCord4(env_pitch, 0, env_pitch_onoff, 1);
AudioConnection          patchCord5(combine, 0, mix_combine_wet, 1);
AudioConnection          patchCord6(vel_fade_midi, 0, velocity_env, 2);
AudioConnection          patchCord7(vel_change_low, 0, velocity_env, 0);
AudioConnection          patchCord8(vel_change_high, 0, velocity_env, 1);
AudioConnection          patchCord9(mix_delay_wet, 0, mix_combine_wet, 0);
AudioConnection          patchCord10(mix_delay, 0, mix_delay_wet, 1);
AudioConnection          patchCord11(env_pitch_onoff, 0, OSC1, 0);
AudioConnection          patchCord12(env_pitch_onoff, 0, OSC2, 0);
AudioConnection          patchCord13(env_pitch_onoff, 0, OSC3, 0);
AudioConnection          patchCord14(delay_fx, 0, mix_delay, 0);
AudioConnection          patchCord15(delay_fx, 1, mix_delay, 1);
AudioConnection          patchCord16(delay_fx, 2, mix_delay, 2);
AudioConnection          patchCord17(delay_fx, 3, mix_delay, 3);
AudioConnection          patchCord18(pink, 0, mix_osc, 3);
AudioConnection          patchCord19(OSC3, 0, mix_osc, 2);
AudioConnection          patchCord20(LFO_ring, 0, mix_ring_wet, 1);
AudioConnection          patchCord21(LFO2, env_filter);
AudioConnection          patchCord22(LFO2, 0, env_filter_onoff, 0);
AudioConnection          patchCord23(dc_ring, 0, mix_ring_wet, 0);
AudioConnection          patchCord24(velocity_env, 0, vca, 0);
AudioConnection          patchCord25(mix_combine_wet, Volume);
AudioConnection          patchCord26(OSC2, 0, mix_osc, 1);
AudioConnection          patchCord27(OSC1, 0, mix_osc, 0);
AudioConnection          patchCord28(freeverbs, 0, mix_verb_wet_l, 1);
AudioConnection          patchCord29(freeverbs, 1, mix_verb_wet_r, 1);
AudioConnection          patchCord30(env_filter, 0, env_filter_onoff, 1);
AudioConnection          patchCord31(lfoVca, 0, vca, 1);
AudioConnection          patchCord32(Volume, freeverbs);
AudioConnection          patchCord33(Volume, 0, mix_verb_wet_l, 0);
AudioConnection          patchCord34(Volume, 0, mix_verb_wet_r, 0);
AudioConnection          patchCord35(vca, 0, ring, 0);
AudioConnection          patchCord36(mix_ring_wet, 0, ring, 1);
AudioConnection          patchCord37(env_filter_onoff, 0, LPF, 1);
AudioConnection          patchCord38(mix_osc, ADSR_vol);
AudioConnection          patchCord39(mix_verb_wet_l, 0, i2s2, 0);
AudioConnection          patchCord40(mix_verb_wet_r, 0, i2s2, 1);
AudioConnection          patchCord41(ADSR_vol, vel_change_low);
AudioConnection          patchCord42(ADSR_vol, vel_change_high);
AudioConnection          patchCord43(ADSR_vol, vel_fade_midi);
AudioConnection          patchCord44(ring, filter_attenuator);
AudioConnection          patchCord45(LPF, 0, delay_fx, 0);
AudioConnection          patchCord46(LPF, 0, mix_delay_wet, 0);
AudioConnection          patchCord47(LPF, 0, combine, 0);
AudioConnection          patchCord48(filter_attenuator, 0, LPF, 0);
AudioControlSGTL5000     sgtl5000_1;     //xy=868.0000076293945,542.0000076293945
// GUItool: end automatically generated code

#endif  //AUDIOSYSTEM_H
