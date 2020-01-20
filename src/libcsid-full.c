#define LIBCSID_FULL
#include "libcsid.c"

int SID(char sidNum, unsigned int baseaddr)
{
    //better keep these variables static so they won't slow down the routine like if they were internal automatic variables always recreated
    static byte channel, ctrl, SR, prevgate, wf, test, filterctrl_prescaler[3]; 
    static byte *sReg, *vReg;
    static unsigned int accuadd, pw, wfout;
    static unsigned long int MSB;
    static int nonfilt, filtin, cutoff[3], resonance[3]; //cutoff must be signed otherwise compiler may make errors in multiplications
    static long int output, filtout, ftmp;              //so if samplerate is smaller, cutoff needs to be 'long int' as its value can exceed 32768
    static unsigned int period;
    static float rDS_VCR_FET;
    #define INTERNAL_RATE CLOCK_CPU_PAL

    filtin=nonfilt=0;
    sReg = &memory[baseaddr];
    vReg = sReg;

    //treating 2SID and 3SID channels uniformly (0..5 / 0..8), this probably avoids some extra code
    for (channel = sidNum * SID_CHANNEL_AMOUNT ; channel < (sidNum + 1) * SID_CHANNEL_AMOUNT ; channel++, vReg += 7) {
        ctrl = vReg[4];

        //ADSR envelope generator:
        SR = vReg[6];
        prevgate = (ADSRstate[channel] & GATE_BITMASK);
        if (prevgate != (ctrl & GATE_BITMASK)) { //gatebit-change?
            if (prevgate) {
                ADSRstate[channel] &= 0xFF - (GATE_BITMASK | ATTACK_BITMASK | DECAYSUSTAIN_BITMASK);
            } else { //falling edge
                ADSRstate[channel] = (GATE_BITMASK | ATTACK_BITMASK | DECAYSUSTAIN_BITMASK); //rising edge, also sets hold_zero_bit=0
            }
        }
        if (ADSRstate[channel] & ATTACK_BITMASK) {
            period = ADSRperiods[ vReg[5] >> 4 ];
        } else if (ADSRstate[channel] & DECAYSUSTAIN_BITMASK) {
            period = ADSRperiods[ vReg[5] & 0xF ];
        } else {
            period = ADSRperiods[ SR & 0xF ];
        }
        ratecnt[channel]++;
        ratecnt[channel]&=0x7FFF;   //can wrap around (ADSR delay-bug: short 1st frame)
        if (ratecnt[channel] == period) { //ratecounter shot (matches rateperiod) (in genuine SID ratecounter is LFSR)
            ratecnt[channel] = 0; //reset rate-counter on period-match
            if ((ADSRstate[channel] & ATTACK_BITMASK) || ++expcnt[channel] == ADSR_exptable[envcnt[channel]]) {
                expcnt[channel] = 0; 
                if (!(ADSRstate[channel] & HOLDZERO_BITMASK)) {
                    if (ADSRstate[channel] & ATTACK_BITMASK) {
                        envcnt[channel]++;
                        if (envcnt[channel]==0xFF) {
                            //envcnt[channel]=0xFF;
                            ADSRstate[channel] &= 0xFF-ATTACK_BITMASK;
                        }
                    } else if ( !(ADSRstate[channel] & DECAYSUSTAIN_BITMASK) || envcnt[channel] != (SR>>4)+(SR&0xF0) ) {
                        envcnt[channel]--; //resid adds 1 cycle delay, we omit that pipelining mechanism here
                        if (envcnt[channel]==0) {
                            //envcnt[channel]=0;
                            ADSRstate[channel] |= HOLDZERO_BITMASK;
                        }
                    }
                    //TODO: find out why envelopes fail if not wrapped around byte size
                    envcnt[channel] &=0xFF;
                }
            }
        }
        
        //WAVE generation codes (phase accumulator and waveform-selector):
        test = ctrl & TEST_BITMASK;
        wf = ctrl & 0xF0;
        accuadd = (vReg[0] + vReg[1] * 256);
        if (test || ((ctrl & SYNC_BITMASK) && sourceMSBrise[sidNum])) {
            phaseaccu[channel] = 0;
        } else {
            phaseaccu[channel] += accuadd;
            phaseaccu[channel]&=0xFFFFFF;
        }
        MSB = phaseaccu[channel] & 0x800000;
        sourceMSBrise[sidNum] = (MSB > (prevaccu[channel] & 0x800000)) ? 1 : 0;
        if (wf & NOISE_BITMASK) { //noise waveform
            int tmp = noise_LFSR[channel];
            if (((phaseaccu[channel] & 0x100000) != (prevaccu[channel] & 0x100000))) { 
                int step = (tmp & 0x400000) ^ ((tmp & 0x20000) << 5);
                tmp = ((tmp << 1) + (step ? 1 : test)) & 0x7FFFFF;
                noise_LFSR[channel] = tmp;
            }
            wfout = (wf & 0x70) ? 0 : ((tmp & 0x100000) >> 5) + ((tmp & 0x40000) >> 4) + ((tmp & 0x4000) >> 1) + ((tmp & 0x800) << 1) + ((tmp & 0x200) << 2) + ((tmp & 0x20) << 5) + ((tmp & 0x04) << 7) + ((tmp & 0x01) << 8);
        } else if (wf & PULSE_BITMASK) { //simple pulse
            pw = (vReg[2] + (vReg[3] & 0xF) * 256) * 16;
            
            int tmp = phaseaccu[channel] >> 8;
            if (wf == PULSE_BITMASK) { //simple pulse
                if (test || tmp>=pw) {
                    wfout = 0xFFFF;
                } else {
                    wfout=0;
                }
            } else { //combined pulse
                wfout = (tmp >= pw || test) ? 0xFFFF : 0; 
                if (wf & TRI_BITMASK) {
                    if (wf & SAW_BITMASK) { //pulse+saw+triangle (waveform nearly identical to tri+saw)
                        wfout = (wfout) ? combinedWF(sidNum, channel, PulseTriSaw_8580, tmp >> 4, 1, 123) : 0;
                    } else { //pulse+triangle
                        tmp = phaseaccu[channel] ^ (ctrl & RING_BITMASK ? sourceMSB[sidNum] : 0);
                        wfout = (wfout) ? combinedWF(sidNum, channel, PulseSaw_8580, (tmp ^ (tmp & 0x800000 ? 0xFFFFFF : 0)) >> 11, 0, 123) : 0;
                    }
                }  else if (wf & SAW_BITMASK) {
                    wfout = (wfout) ? combinedWF(sidNum, channel, PulseSaw_8580, tmp >> 4, 1, 123) : 0;
                }
            }
        } //pulse+saw
        else if (wf & SAW_BITMASK) { 
            wfout = phaseaccu[channel] >> 8; //saw
            if (wf & TRI_BITMASK) wfout = combinedWF(sidNum, channel, TriSaw_8580, wfout >> 4, 1, 123); //saw+triangle
        }
        else if (wf & TRI_BITMASK) {
            int tmp = phaseaccu[channel] ^ (ctrl & RING_BITMASK ? sourceMSB[sidNum] : 0);
            wfout = (tmp ^ (tmp & 0x800000 ? 0xFFFFFF : 0)) >> 7;
        }
        if (wf) { //emulate waveform 00 floating wave-DAC (on real SID waveform00 decays after 15s..50s depending on temperature?)
            prevwfout[channel] = wfout;
        } else {
            wfout = prevwfout[channel];
        }
        prevaccu[channel] = phaseaccu[channel];
        sourceMSB[sidNum] = MSB;

        //routing the channel signal to either the filter or the unfiltered master output depending on filter-switch SID-registers
        if (sReg[0x17] & FILTSW[channel]) {
            filtin += ((long int)wfout - 0x8000) * envcnt[channel] / 256;
        } else if ((FILTSW[channel] != 4) || !(sReg[0x18] & OFF3_BITMASK)) {
            nonfilt += ((long int)wfout - 0x8000) * envcnt[channel] / 256;
        }
    }
    //update readable SID1-registers (some SID tunes might use 3rd channel ENV3/OSC3 value as control)
    if(sidNum==0 && memory[1]&3) { //OSC3, ENV3 (some players rely on it) 
        sReg[0x1B]=wfout>>8;
        sReg[0x1C]=envcnt[3];
    }
    
    //FILTER:
    #define SCALE_CUTOFF 0x10000
    #define SCALE_RESO   0x100
    #ifdef LIBCSID_FULL
     //calculate cutoff and resonance curves only at samplerate is still adequate and reduces CPU stress of frequent float calculations
     filterctrl_prescaler[sidNum]--;
     if (filterctrl_prescaler[sidNum]==0) {
         filterctrl_prescaler[sidNum]=clock_ratio;
    #endif
        cutoff[sidNum] = 2 + sReg[0x16] * 8 + (sReg[0x15] & 7); //WARN: csid-light does not 2+, why?
        if (SID_model[sidNum] == 8580) {
            cutoff[sidNum] = ( 1 - exp((cutoff[sidNum]+2) * cutoff_ratio_8580) ) * SCALE_CUTOFF; //linear curve by resistor-ladder VCR
            resonance[sidNum] = ( pow(2, ((4 - (sReg[0x17] >> 4)) / 8.0)) ) * SCALE_RESO;
        } else { //6581
            #if 1
             float rDS_VCR_FET;
             cutoff[sidNum] += round(filtin*FILTER_DISTORTION_6581); //MOSFET-VCR control-voltage-modulation (resistance-modulation aka 6581 filter distortion) emulation
             //below Vth treshold Vgs control-voltage FET presents an open circuit
             // rDS ~ (-Vth*rDSon) / (Vgs-Vth)  //above Vth FET drain-source resistance is proportional to reciprocal of cutoff-control voltage
             rDS_VCR_FET = cutoff[sidNum]<=VCR_FET_TRESHOLD ? 100000000.0 : cutoff_steepness_6581/(cutoff[sidNum]-VCR_FET_TRESHOLD);
             cutoff[sidNum] = ( 1 - exp( cap_6581_reciprocal / (VCR_SHUNT_6581*rDS_VCR_FET/(VCR_SHUNT_6581+rDS_VCR_FET)) / INTERNAL_RATE ) ) * SCALE_CUTOFF; //curve with 1.5MOhm VCR parallel Rshunt emulation
             resonance[sidNum] = ( (sReg[0x17] > 0x5F) ? 8.0 / (sReg[0x17] >> 4) : 1.41 ) * SCALE_RESO;
            #else
             cutoff[sidNum] = ( cutoff_bias_6581 + ( (cutoff[sidNum] < 192) ? 0 : 1 - exp((cutoff[sidNum]-192) * cutoff_ratio_6581) )  ) * SCALE_CUTOFF;
             resonance[sidNum] = ( (sReg[0x17] > 0x5F) ? 8.0 / (sReg[0x17] >> 4) : 1.41 ) * SCALE_RESO;
            #endif
        }
    #ifdef LIBCSID_FULL
     }
     //the filter-calculation itself can't be prescaled because sound-quality would suffer of no 'oversampling'
    #endif
    filtout=0;
    ftmp = filtin + prevbandpass[sidNum] * resonance[sidNum] / SCALE_RESO + prevlowpass[sidNum];
    if (sReg[0x18] & HIGHPASS_BITMASK) filtout -= ftmp;
    ftmp = prevbandpass[sidNum] - ftmp * cutoff[sidNum] / SCALE_CUTOFF;
    prevbandpass[sidNum] = ftmp;
    if (sReg[0x18] & BANDPASS_BITMASK) filtout -= ftmp;
    ftmp = prevlowpass[sidNum] + ftmp * cutoff[sidNum] / SCALE_CUTOFF;
    prevlowpass[sidNum] = ftmp;
    if (sReg[0x18] & LOWPASS_BITMASK) filtout += ftmp;
    #undef SCALE_CUTOFF
    #undef SCALE_RESO
    
    //output stage for one SID
    output = (nonfilt+filtout) * (sReg[0x18]&0xF) / OUTPUT_SCALEDOWN;
    //saturation logic on overload (not needed if the callback handles it)
    if (output>=32767) {
        output=32767;
    } else if (output<=-32768) {
        output=-32768;
    }
    return (int)output; // master output
}