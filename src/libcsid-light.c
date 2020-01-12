#include "libcsid.c"

//Applies mostly to csid light
//My SID implementation is similar to what I worked out in a SwinSID variant during 3..4 months of development. (So jsSID only took 2 weeks armed with this experience.)
//I learned the workings of ADSR/WAVE/filter operations mainly from the quite well documented resid and resid-fp codes.
//(The SID reverse-engineering sites were also good sources.)
//Note that I avoided many internal/automatic variables from the SID function, assuming better speed this way. (Not using stack as much, but I'm not sure and it may depend on platform...)
//(The same is true for CPU emulation and player-code.)
//
//Generator, pulse:
//One of my biggest success with the SwinSID-variant was that I could clean the high-pitched and thin sounds.
//(You might have faced with the unpleasant sound quality of high-pitched sounds without oversampling. We need so-called 'band-limited' synthesis instead.
// There are a lot of articles about this issue on the internet. In a nutshell, the harsh edges produce harmonics that exceed the 
// Nyquist frequency (samplerate/2) and they are folded back into hearable range, producing unvanted ringmodulation-like effect.)
//After so many trials with dithering/filtering/oversampling/etc. it turned out I can't eliminate the fukkin aliasing in time-domain, as suggested at pages.
//Oversampling (running the wave-generation 8 times more) was not a way at 32MHz SwinSID. It might be an option on PC but I don't prefer it in JavaScript.)
//The only solution that worked for me in the end, what I came up with eventually: The harsh rising and falling edges of the pulse are
//elongated making it a bit trapezoid. But not in time-domain, but altering the transfer-characteristics. This had to be done
//in a frequency-dependent way, proportionally to pitch, to keep the deep sounds crisp. The following code does this (my favourite testcase is Robocop3 intro):
//
//Generator, saw:
//The anti-aliasing (cleaning) of high-pitched sawtooth wave works by the same principle as mentioned above for the pulse,
//but the sawtooth has even harsher edge/transition, and as the falling edge gets longer, tha rising edge should became shorter, 
//and to keep the amplitude, it should be multiplied a little bit (with reciprocal of rising-edge steepness).
//The waveform at the output essentially becomes an asymmetric triangle, more-and-more approaching symmetric shape towards high frequencies.
//(If you check a recording from the real SID, you can see a similar shape, the high-pitch sawtooth waves are triangle-like...)
//But for deep sounds the sawtooth is really close to a sawtooth, as there is no aliasing there, but deep sounds should be sharp...
//
//Filter:
//Two integrator loop bi-quadratic filter, workings learned from resid code, but I kindof simplified the equations
//The phases of lowpass and highpass outputs are inverted compared to the input, but bandpass IS in phase with the input signal.
//The 8580 cutoff frequency control-curve is ideal (binary-weighted resistor-ladder VCRs), while the 6581 has a treshold, and below that it 
//outputs a constant ~200Hz cutoff frequency. (6581 uses MOSFETs as VCRs to control cutoff causing nonlinearity and some 'distortion' due to resistance-modulation.
//There's a cca. 1.53Mohm resistor in parallel with the MOSFET in 6581 which doesn't let the frequency go below 200..220Hz
//Even if the MOSFET doesn't conduct at all. 470pF capacitors are small, so 6581 can't go below this cutoff-frequency with 1.5MOhm.)
//
//Output:
//output stage for one SID
//when it comes to $D418 volume-register digi playback, I made an AC / DC separation for $D418 value in the SwinSID at low (20Hz or so) cutoff-frequency,
//and sent the AC (highpass) value to a 4th 'digi' channel mixed to the master output, and set ONLY the DC (lowpass) value to the volume-control.
//This solved 2 issues: Thanks to the lowpass filtering of the volume-control, SID tunes where digi is played together with normal SID channels,
//won't sound distorted anymore, and the volume-clicks disappear when setting SID-volume. (This is useful for fade-in/out tunes like Hades Nebula, where clicking ruins the intro.)

int SID(char num, unsigned int baseaddr) { //the SID emulation itself ('num' is the number of SID to iterate (0..2)
    
    //better keep these variables static so they won't slow down the routine like if they were internal automatic variables always recreated
    static byte channel, ctrl, SR, prevgate, wf, test; byte *sReg, *vReg;
    static unsigned int accuadd, MSB, pw, wfout;
    static int tmp, step, lim, nonfilt, filtin, filtout, output;
    static float period, steep, rDS_VCR_FET, cutoff[3], resonance[3], ftmp;
    
    filtin=nonfilt=0;
    sReg = &memory[baseaddr];
    vReg = sReg;
    
    //treating 2SID and 3SID channels uniformly (0..5 / 0..8), this probably avoids some extra code
    for (channel = num * SID_CHANNEL_AMOUNT ; channel < (num + 1) * SID_CHANNEL_AMOUNT ; channel++, vReg += 7) {
        ctrl = vReg[4];
        
        //ADSR envelope-generator:
        SR = vReg[6]; tmp = 0;
        prevgate = (ADSRstate[channel] & GATE_BITMASK);
        if (prevgate != (ctrl & GATE_BITMASK)) { //gatebit-change?
            if (prevgate) {
                ADSRstate[channel] &= 0xFF - (GATE_BITMASK | ATTACK_BITMASK | DECAYSUSTAIN_BITMASK);
            }else { //falling edge
                ADSRstate[channel] = (GATE_BITMASK | ATTACK_BITMASK | DECAYSUSTAIN_BITMASK); //rising edge, also sets hold_zero_bit=0
                //assume SR->GATE write order: workaround to have crisp soundstarts by triggering delay-bug
                //(this is for the possible missed CTRL(GATE) vs SR register write order situations (1MHz CPU is cca 20 times faster than samplerate)
                if ((SR & 0xF) > (prevSR[channel] & 0xF)) tmp = 1;
            }
        }
        prevSR[channel] = SR; //if(SR&0xF) ratecnt[channel]+=5;  //assume SR->GATE write order: workaround to have crisp soundstarts by triggering delay-bug
        ratecnt[channel] += clock_ratio;
        if (ratecnt[channel] >= 0x8000) ratecnt[channel] -= 0x8000; //can wrap around (ADSR delay-bug: short 1st frame)
        //set ADSR period that should be checked against rate-counter (depending on ADSR state Attack/DecaySustain/Release)
        if (ADSRstate[channel] & ATTACK_BITMASK) {
            step = vReg[5] >> 4;
        } else if (ADSRstate[channel] & DECAYSUSTAIN_BITMASK) {
            step = vReg[5] & 0xF;
        } else {
            step = SR & 0xF;
        }
        period = ADSRperiods[step];
        step = ADSRstep[step];
        if (ratecnt[channel] >= period && ratecnt[channel] < period + clock_ratio && tmp == 0) { //ratecounter shot (matches rateperiod) (in genuine SID ratecounter is LFSR)
            ratecnt[channel] -= period; //compensation for timing instead of simply setting 0 on rate-counter overload
            if ((ADSRstate[channel] & ATTACK_BITMASK) || ++expcnt[channel] == ADSR_exptable[envcnt[channel]]) {
                if (!(ADSRstate[channel] & HOLDZERO_BITMASK)) {
                    if (ADSRstate[channel] & ATTACK_BITMASK) {
                        envcnt[channel]+=step;
                        if (envcnt[channel]>=0xFF) {
                            envcnt[channel]=0xFF;
                            ADSRstate[channel] &= 0xFF-ATTACK_BITMASK;
                        }
                    } else if ( !(ADSRstate[channel] & DECAYSUSTAIN_BITMASK) || envcnt[channel] > (SR&0xF0) + (SR>>4) ) {
                        envcnt[channel]-=step;
                        if (envcnt[channel]<=0 && envcnt[channel]+step!=0) {
                            envcnt[channel]=0;
                            ADSRstate[channel]|=HOLDZERO_BITMASK;
                        }
                    }
                }
                expcnt[channel] = 0;
            }
        }
        envcnt[channel] &= 0xFF;
        
        //WAVE-generation code (phase accumulator and waveform-selector):
        test = ctrl & TEST_BITMASK;
        wf = ctrl & 0xF0;
        accuadd = (vReg[0] + vReg[1] * 256) * clock_ratio;
        if (test || ((ctrl & SYNC_BITMASK) && sourceMSBrise[num])) {
            phaseaccu[channel] = 0;
        } else { 
            phaseaccu[channel] += accuadd;
            if (phaseaccu[channel] > 0xFFFFFF) phaseaccu[channel] -= 0x1000000;
        }
        phaseaccu[channel] &= 0xFFFFFF; MSB = phaseaccu[channel] & 0x800000; sourceMSBrise[num] = (MSB > (prevaccu[channel] & 0x800000)) ? 1 : 0;
        if (wf & NOISE_BITMASK) { //noise waveform
            tmp = noise_LFSR[channel];
            if (((phaseaccu[channel] & 0x100000) != (prevaccu[channel] & 0x100000)) || accuadd >= 0x100000) {
                //clock LFSR all time if clockrate exceeds observable at given samplerate 
                step = (tmp & 0x400000) ^ ((tmp & 0x20000) << 5);
                tmp = ((tmp << 1) + (step ? 1 : test)) & 0x7FFFFF;
                noise_LFSR[channel]=tmp;
            }
            //we simply zero output when other waveform is mixed with noise. On real SID LFSR continuously gets filled by zero and locks up. ($C1 waveform with pw<8 can keep it for a while...)
            wfout = (wf & 0x70) ? 0 : ((tmp & 0x100000) >> 5) + ((tmp & 0x40000) >> 4) + ((tmp & 0x4000) >> 1) + ((tmp & 0x800) << 1) + ((tmp & 0x200) << 2) + ((tmp & 0x20) << 5) + ((tmp & 0x04) << 7) + ((tmp & 0x01) << 8);
        } else if (wf & PULSE_BITMASK) { //simple pulse
            pw = (vReg[2] + (vReg[3] & 0xF) * 256) * 16;  tmp = (int) accuadd >> 9;  
            if (0 < pw && pw < tmp) pw = tmp;
            tmp ^= 0xFFFF;
            if (pw > tmp) pw = tmp;  
            tmp = phaseaccu[channel] >> 8;
            if (wf == PULSE_BITMASK) { //simple pulse, band-limited
                step = (accuadd>=255)? 65535/(accuadd/256.0) : 0xFFFF;
                if (test) {
                    wfout=0xFFFF;
                } else if (tmp<pw) { //rising edge
                    lim=(0xFFFF-pw)*step;
                    if (lim>0xFFFF) lim=0xFFFF;
                    tmp=lim-(pw-tmp)*step;
                    wfout=(tmp<0)?0:tmp;
                } else { //falling edge
                    lim=pw*step;
                    if (lim>0xFFFF) lim=0xFFFF;
                    tmp=(0xFFFF-tmp)*step-lim;
                    wfout=(tmp>=0)?0xFFFF:tmp;
                } 
            } else { //combined pulse
                wfout = (tmp >= pw || test) ? 0xFFFF:0; //(this would be enough for a simple but aliased-at-high-pitches pulse)
                if (wf&TRI_BITMASK) { 
                    if (wf&SAW_BITMASK) { //pulse+saw+triangle (waveform nearly identical to tri+saw)
                        wfout = wfout? combinedWF(num,channel,PulseTriSaw_8580,tmp>>4,1,vReg[1]) : 0;
                    } else { //pulse+triangle
                        tmp=phaseaccu[channel]^(ctrl&RING_BITMASK?sourceMSB[num]:0);
                        wfout = (wfout)? combinedWF(num,channel,PulseSaw_8580,(tmp^(tmp&0x800000?0xFFFFFF:0))>>11,0,vReg[1]) : 0;
                    }
                } else if (wf&SAW_BITMASK) {
                    wfout = wfout? combinedWF(num,channel,PulseSaw_8580,tmp>>4,1,vReg[1]) : 0;
                }
            }
        } else if (wf&SAW_BITMASK) { //saw, band-limited
            wfout=phaseaccu[channel]>>8; //his row would be enough for simple but aliased-at-high-pitch saw
            if (wf&TRI_BITMASK) { //saw+triangle
                wfout = combinedWF(num,channel,TriSaw_8580,wfout>>4,1,vReg[1]);
            } else { //simple cleaned (bandlimited) saw
                steep=(accuadd/65536.0)/288.0;
                wfout += wfout*steep;
                if(wfout>0xFFFF) wfout=0xFFFF-(wfout-0x10000)/steep; 
            } 
        } else if (wf&TRI_BITMASK) { //triangle (this waveform has no harsh edges, so it doesn't suffer from strong aliasing at high pitches)
            tmp=phaseaccu[channel]^(ctrl&RING_BITMASK?sourceMSB[num]:0);
            wfout = (tmp^(tmp&0x800000?0xFFFFFF:0)) >> 7; 
        }
        wfout&=0xFFFF;
        if (wf) { //emulate waveform 00 floating wave-DAC (on real SID waveform00 decays after 15s..50s depending on temperature?)
            prevwfout[channel] = wfout;
        } else {
            wfout = prevwfout[channel];
        }
        prevaccu[channel] = phaseaccu[channel];
        sourceMSB[num] = MSB; //(So the decay is not an exact value. Anyway, we just simply keep the value to avoid clicks and support SounDemon digi later...)
        
        //routing the channel signal to either the filter or the unfiltered master output depending on filter-switch SID-registers
        if (sReg[0x17] & FILTSW[channel]) {
            filtin += ((int)wfout - 0x8000) * envcnt[channel] / 256;
        }else if ((FILTSW[channel] != 4) || !(sReg[0x18] & OFF3_BITMASK)) {
            nonfilt += ((int)wfout - 0x8000) * envcnt[channel] / 256;
        }         
    }
    //update readable SID1-registers (some SID tunes might use 3rd channel ENV3/OSC3 value as control)
    if(num==0 && memory[1]&3) { //OSC3, ENV3 (some players rely on it)  
        sReg[0x1B]=wfout>>8;
        sReg[0x1C]=envcnt[3];
    }  
    
    cutoff[num] = sReg[0x16] * 8 + (sReg[0x15] & 7);
    if (SID_model[num] == 8580) {
        cutoff[num] = ( 1 - exp((cutoff[num]+2) * cutoff_ratio_8580) ); //linear curve by resistor-ladder VCR
        resonance[num] = ( pow(2, ((4 - (sReg[0x17] >> 4)) / 8.0)) );
    } else { //6581
        cutoff[num] += round(filtin*FILTER_DISTORTION_6581); //MOSFET-VCR control-voltage-modulation (resistance-modulation aka 6581 filter distortion) emulation
        rDS_VCR_FET = cutoff[num]<=VCR_FET_TRESHOLD ? 100000000.0 //below Vth treshold Vgs control-voltage FET presents an open circuit
         : cutoff_steepness_6581/(cutoff[num]-VCR_FET_TRESHOLD); // rDS ~ (-Vth*rDSon) / (Vgs-Vth)  //above Vth FET drain-source resistance is proportional to reciprocal of cutoff-control voltage
        cutoff[num] = ( 1 - exp( cap_6581_reciprocal / (VCR_SHUNT_6581*rDS_VCR_FET/(VCR_SHUNT_6581+rDS_VCR_FET)) / samplerate ) ); //curve with 1.5MOhm VCR parallel Rshunt emulation
        resonance[num] = ( (sReg[0x17] > 0x5F) ? 8.0 / (sReg[0x17] >> 4) : 1.41 );
    }  
    filtout=0;
    ftmp = filtin + prevbandpass[num] * resonance[num] + prevlowpass[num];
    if (sReg[0x18] & HIGHPASS_BITMASK) filtout -= ftmp;
    ftmp = prevbandpass[num] - ftmp * cutoff[num];
    prevbandpass[num] = ftmp;
    if (sReg[0x18] & BANDPASS_BITMASK) filtout -= ftmp;
    ftmp = prevlowpass[num] + ftmp * cutoff[num];
    prevlowpass[num] = ftmp;
    if (sReg[0x18] & LOWPASS_BITMASK) filtout += ftmp;    
    
    output = (nonfilt+filtout) * (sReg[0x18]&0xF) / OUTPUT_SCALEDOWN;
    //saturation logic on overload (not needed if the callback handles it)
    if (output>=32767) {
        output=32767;
    } else if (output<=-32768) {
        output=-32768;
    }
    return (int)output; // master output
}