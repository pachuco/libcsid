#define LIBCSID_FULL
#include "libcsid.c"

//SID-emulation variables:
byte envcnt[9];
unsigned int clock_ratio=22; 
unsigned int ratecnt[9], prevwfout[9]; 
unsigned long int phaseaccu[9], prevaccu[9], sourceMSB[3], noise_LFSR[9];
long int prevlowpass[3], prevbandpass[3];
float cutoff_ratio_8580, cutoff_ratio_6581, cutoff_bias_6581;
//player-related variables:
int framecnt=0, frame_sampleperiod = DEFAULT_SAMPLERATE/PAL_FRAMERATE; 
//CPU (and CIA/VIC-IRQ) emulation constants and variables - avoiding internal/automatic variables to retain speed
char CPUtime=0;


//----------------------------- MAIN thread ----------------------------

void init (byte subt)
{
 static long int timeout; subtune = subt; initCPU(initaddr); initSID(); A=subtune; memory[1]=0x37; memory[0xDC05]=0;
 for(timeout=100000;timeout>=0;timeout--) { if (CPU()) break; } 
 if (timermode[subtune] || memory[0xDC05]) { //&& playaddf {   //CIA timing
  if (!memory[0xDC05]) {memory[0xDC04]=0x24; memory[0xDC05]=0x40;} //C64 startup-default
  frame_sampleperiod = (memory[0xDC04]+memory[0xDC05]*256)/clock_ratio; }
 else frame_sampleperiod = samplerate/PAL_FRAMERATE;  //Vsync timing
 printf("Frame-sampleperiod: %d samples  (%.2fX speed)\n",frame_sampleperiod,(double)(samplerate/PAL_FRAMERATE)/frame_sampleperiod); 
 //frame_sampleperiod = (memory[0xDC05]!=0 || (!timermode[subtune] && playaddf))? samplerate/PAL_FRAMERATE : (memory[0xDC04] + memory[0xDC05]*256) / clock_ratio; 
 if(playaddf==0) { playaddr = ((memory[1]&3)<2)? memory[0xFFFE]+memory[0xFFFF]*256 : memory[0x314]+memory[0x315]*256; printf("IRQ-playaddress:%4.4X\n",playaddr); }
 else { playaddr=playaddf; if (playaddr>=0xE000 && memory[1]==0x37) memory[1]=0x35; } //player under KERNAL (Crystal Kingdom Dizzy)
 initCPU(playaddr); framecnt=1; finished=0; CPUtime=0; 
}


void play(void* userdata, byte *stream, int len ) //called by SDL at samplerate pace
{ 
 static int i,j, output; static float average;
 
 for(i=0;i<len;i+=2) {
  framecnt--; if (framecnt<=0) { framecnt=frame_sampleperiod; finished=0; PC=playaddr; SP=0xFF; } // printf("%d  %f\n",framecnt,playtime); }
  average = 0.0 ;
  for (j=0; j<sampleratio; j++) {
   if (finished==0 && --cycles<=0) {
     pPC=PC; if (CPU()>=0xFE || ( (memory[1]&3)>1 && pPC<0xE000 && (PC==0xEA31 || PC==0xEA81) ) ) finished=1; //IRQ player ROM return handling
     if ( (addr==0xDC05 || addr==0xDC04) && (memory[1]&3) && timermode[subtune] ) {
      frame_sampleperiod = (memory[0xDC04] + memory[0xDC05]*256) / clock_ratio;  //dynamic CIA-setting (Galway/Rubicon workaround)
      if (!dynCIA) {dynCIA=1; printf("( Dynamic CIA settings. New frame-sampleperiod: %d samples  (%.2fX speed) )\n",frame_sampleperiod,(double)(samplerate/PAL_FRAMERATE)/frame_sampleperiod);}
     }
     if(storadd>=0xD420 && storadd<0xD800 && (memory[1]&3)) {  //CJ in the USA workaround (writing above $d420, except SID2/SID3)
      if ( !(SID_address[1]<=storadd && storadd<SID_address[1]+0x1F) && !(SID_address[2]<=storadd && storadd<SID_address[2]+0x1F) )
       memory[storadd&0xD41F]=memory[storadd]; //write to $D400..D41F if not in SID2/SID3 address-space
     }
   }
   average += SID(0,0xD400);
   if (SIDamount>=2) average += SID(1,SID_address[1]); 
   if (SIDamount==3) average += SID(2,SID_address[2]); 
  } 
  output = average / sampleratio; 
  stream[i]=output&0xFF; 
  stream[i+1]=output>>8; 
 }
 
  //mix = SID(0,0xD400); if (SID_address[1]) mix += SID(1,SID_address[1]); if(SID_address[2]) mix += SID(2,SID_address[2]);
  //return mix * volume * SIDamount_vol[SIDamount] + (Math.random()*background_noise-background_noise/2); 
}

//----------------------------- SID emulation -----------------------------------------



unsigned int TriSaw_8580[4096], PulseSaw_8580[4096], PulseTriSaw_8580[4096];
int ADSRperiods[16] = {9, 32, 63, 95, 149, 220, 267, 313, 392, 977, 1954, 3126, 3907, 11720, 19532, 31251};
const byte ADSR_exptable[256] = {1, 30, 30, 30, 30, 30, 30, 16, 16, 16, 16, 16, 16, 16, 16, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 4, 4, 4, 4, 4, //pos0:1  pos6:30  pos14:16  pos26:8
    4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, //pos54:4 //pos93:2
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };


void cSID_init(int samplerate)
{
    int i;
    clock_ratio = round(C64_PAL_CPUCLK/samplerate);
    cutoff_ratio_8580 = -2 * 3.14 * (12500.0 / 2048) / C64_PAL_CPUCLK;
    cutoff_ratio_6581 = -2 * 3.14 * (20000.0 / 2048) / C64_PAL_CPUCLK;
    cutoff_bias_6581 = 1 - exp( -2 * 3.14 * 220 / C64_PAL_CPUCLK ); //around 220Hz below treshold
    
    createCombinedWF(TriSaw_8580, 0.5, 2.2, 0.9);
    createCombinedWF(PulseSaw_8580, 0.23, 1.27, 0.55);
    createCombinedWF(PulseTriSaw_8580, 0.5, 1.6, 0.8);
    //createCombinedWF_old(TriSaw_8580, 0.8, 2.4, 0.64);
    //createCombinedWF_old(PulseSaw_8580, 1.4, 1.9, 0.68);
    //createCombinedWF_old(PulseTriSaw_8580, 0.8, 2.5, 0.64);
    
    for(i = 0; i < 9; i++) {
        ADSRstate[i] = HOLDZERO_BITMASK; envcnt[i] = 0; ratecnt[i] = 0; 
        phaseaccu[i] = 0; prevaccu[i] = 0; expcnt[i] = 0; 
        noise_LFSR[i] = 0x7FFFF8; prevwfout[i] = 0;
    }
    for(i = 0; i < 3; i++) {
        sourceMSBrise[i] = 0; sourceMSB[i] = 0;
        prevlowpass[i] = 0; prevbandpass[i] = 0;
    }
   initSID();
}


void initSID() { 
  int i;
  for(i=0xD400;i<=0xD7FF;i++) memory[i]=0; for(i=0xDE00;i<=0xDFFF;i++) memory[i]=0;
  for(i=0;i<9;i++) {ADSRstate[i]=HOLDZERO_BITMASK; ratecnt[i]=envcnt[i]=expcnt[i]=0;} 
 }

int SID(char num, unsigned int baseaddr)
{
    //better keep these variables static so they won't slow down the routine like if they were internal automatic variables always recreated
    static byte channel, ctrl, SR, prevgate, wf, test, filterctrl_prescaler[3]; 
    static byte *sReg, *vReg;
    static unsigned int period, accuadd, pw, wfout;
    static unsigned long int MSB;
    static int nonfilt, filtin, cutoff[3], resonance[3]; //cutoff must be signed otherwise compiler may make errors in multiplications
    static long int output, filtout, ftmp;              //so if samplerate is smaller, cutoff needs to be 'long int' as its value can exceed 32768

    filtin=nonfilt=0; sReg = &memory[baseaddr]; vReg = sReg;
    for (channel = num * SID_CHANNEL_AMOUNT ; channel < (num + 1) * SID_CHANNEL_AMOUNT ; channel++, vReg += 7) {
        ctrl = vReg[4];

        //ADSR envelope generator:
        {
            SR = vReg[6];
            prevgate = (ADSRstate[channel] & GATE_BITMASK);
            if (prevgate != (ctrl & GATE_BITMASK)) { //gatebit-change?
                if (prevgate) {
                    ADSRstate[channel] &= 0xFF - (GATE_BITMASK | ATTACK_BITMASK | DECAYSUSTAIN_BITMASK);
                } //falling edge
                else {
                    ADSRstate[channel] = (GATE_BITMASK | ATTACK_BITMASK | DECAYSUSTAIN_BITMASK); //rising edge, also sets hold_zero_bit=0
                }
            }
            if (ADSRstate[channel] & ATTACK_BITMASK) period = ADSRperiods[ vReg[5] >> 4 ];
            else if (ADSRstate[channel] & DECAYSUSTAIN_BITMASK) period = ADSRperiods[ vReg[5] & 0xF ];
            else period = ADSRperiods[ SR & 0xF ];
            ratecnt[channel]++; ratecnt[channel]&=0x7FFF;   //can wrap around (ADSR delay-bug: short 1st frame)
            if (ratecnt[channel] == period) { //ratecounter shot (matches rateperiod) (in genuine SID ratecounter is LFSR)
                ratecnt[channel] = 0; //reset rate-counter on period-match
                if ((ADSRstate[channel] & ATTACK_BITMASK) || ++expcnt[channel] == ADSR_exptable[envcnt[channel]]) {
                    expcnt[channel] = 0; 
                    if (!(ADSRstate[channel] & HOLDZERO_BITMASK)) {
                        if (ADSRstate[channel] & ATTACK_BITMASK) {
                            envcnt[channel]++;
                            if (envcnt[channel]==0xFF) ADSRstate[channel] &= 0xFF - ATTACK_BITMASK;
                        } 
                        else if ( !(ADSRstate[channel] & DECAYSUSTAIN_BITMASK) || envcnt[channel] != (SR>>4)+(SR&0xF0) ) {
                            envcnt[channel]--; //resid adds 1 cycle delay, we omit that pipelining mechanism here
                            if (envcnt[channel]==0) ADSRstate[channel] |= HOLDZERO_BITMASK;
                        }
                    }
                }
            }
        }
        
        //WAVE generation codes (phase accumulator and waveform-selector):
        test = ctrl & TEST_BITMASK;
        wf = ctrl & 0xF0;
        accuadd = (vReg[0] + vReg[1] * 256);
        if (test || ((ctrl & SYNC_BITMASK) && sourceMSBrise[num])) {
            phaseaccu[channel] = 0;
        } else {
            phaseaccu[channel] += accuadd; phaseaccu[channel]&=0xFFFFFF;
        }
        MSB = phaseaccu[channel] & 0x800000;
        sourceMSBrise[num] = (MSB > (prevaccu[channel] & 0x800000)) ? 1 : 0;
        if (wf & NOISE_BITMASK) {
            int tmp = noise_LFSR[channel];
            if (((phaseaccu[channel] & 0x100000) != (prevaccu[channel] & 0x100000))) { 
                int step = (tmp & 0x400000) ^ ((tmp & 0x20000) << 5);
                tmp = ((tmp << 1) + (step ? 1 : test)) & 0x7FFFFF;
                noise_LFSR[channel] = tmp;
            }
            wfout = (wf & 0x70) ? 0 : ((tmp & 0x100000) >> 5) + ((tmp & 0x40000) >> 4) + ((tmp & 0x4000) >> 1) + ((tmp & 0x800) << 1) + ((tmp & 0x200) << 2) + ((tmp & 0x20) << 5) + ((tmp & 0x04) << 7) + ((tmp & 0x01) << 8);
        } else if (wf & PULSE_BITMASK) {
            pw = (vReg[2] + (vReg[3] & 0xF) * 256) * 16;
            
            int tmp = phaseaccu[channel] >> 8;
            if (wf == PULSE_BITMASK) {
                if (test || tmp>=pw) wfout = 0xFFFF;
                else {
                    wfout=0;
                }
            }
            else { //combined pulse
                wfout = (tmp >= pw || test) ? 0xFFFF : 0; 
                if (wf & TRI_BITMASK) {
                    if (wf & SAW_BITMASK) {
                        wfout = (wfout) ? combinedWF(num, channel, PulseTriSaw_8580, tmp >> 4, 1, 123) : 0;
                    } //pulse+saw+triangle (waveform nearly identical to tri+saw)
                    else {
                        tmp = phaseaccu[channel] ^ (ctrl & RING_BITMASK ? sourceMSB[num] : 0);
                        wfout = (wfout) ? combinedWF(num, channel, PulseSaw_8580, (tmp ^ (tmp & 0x800000 ? 0xFFFFFF : 0)) >> 11, 0, 123) : 0;
                    }
                } //pulse+triangle
                else if (wf & SAW_BITMASK) wfout = (wfout) ? combinedWF(num, channel, PulseSaw_8580, tmp >> 4, 1, 123) : 0;
            }
        } //pulse+saw
        else if (wf & SAW_BITMASK) { 
            wfout = phaseaccu[channel] >> 8; //saw
            if (wf & TRI_BITMASK) wfout = combinedWF(num, channel, TriSaw_8580, wfout >> 4, 1, 123); //saw+triangle
        }
        else if (wf & TRI_BITMASK) {
            int tmp = phaseaccu[channel] ^ (ctrl & RING_BITMASK ? sourceMSB[num] : 0);
            wfout = (tmp ^ (tmp & 0x800000 ? 0xFFFFFF : 0)) >> 7;
        }
        if (wf) prevwfout[channel] = wfout;
        else {
            wfout = prevwfout[channel];
        } //emulate waveform 00 floating wave-DAC
        prevaccu[channel] = phaseaccu[channel];
        sourceMSB[num] = MSB;
        if (sReg[0x17] & FILTSW[channel]) filtin += ((long int)wfout - 0x8000) * envcnt[channel] / 256;
        else if ((FILTSW[channel] != 4) || !(sReg[0x18] & OFF3_BITMASK)) 
                nonfilt += ((long int)wfout - 0x8000) * envcnt[channel] / 256;
    }
    //update readable SID1-registers (some SID tunes might use 3rd channel ENV3/OSC3 value as control)
    if(num==0, memory[1]&3) { sReg[0x1B]=wfout>>8; sReg[0x1C]=envcnt[3]; } //OSC3, ENV3 (some players rely on it) 
    
    //FILTER:
    filterctrl_prescaler[num]--;
    if (filterctrl_prescaler[num]==0)
    {  //calculate cutoff and resonance curves only at samplerate is still adequate and reduces CPU stress of frequent float calculations
     filterctrl_prescaler[num]=clock_ratio;
     cutoff[num] = 2 + sReg[0x16] * 8 + (sReg[0x15] & 7);
     if (SID_model[num] == 8580) {
         cutoff[num] = ( 1 - exp(cutoff[num] * cutoff_ratio_8580) ) * 0x10000;
         resonance[num] = ( pow(2, ((4 - (sReg[0x17] >> 4)) / 8.0)) ) * 0x100; //resonance could be taken from table as well
     } else {
         cutoff[num] = (  cutoff_bias_6581 + ( (cutoff[num] < 192) ? 0 : 1 - exp((cutoff[num]-192) * cutoff_ratio_6581) )  ) * 0x10000;
         resonance[num] = ( (sReg[0x17] > 0x5F) ? 8.0 / (sReg[0x17] >> 4) : 1.41 ) * 0x100;
     }  
    }
    filtout=0; //the filter-calculation itself can't be prescaled because sound-quality would suffer of no 'oversampling'
    ftmp = filtin + prevbandpass[num] * resonance[num] / 0x100 + prevlowpass[num];
    if (sReg[0x18] & HIGHPASS_BITMASK) filtout -= ftmp;
    ftmp = prevbandpass[num] - ftmp * cutoff[num] / 0x10000;
    prevbandpass[num] = ftmp;
    if (sReg[0x18] & BANDPASS_BITMASK) filtout -= ftmp;
    ftmp = prevlowpass[num] + ftmp * cutoff[num] / 0x10000;
    prevlowpass[num] = ftmp;
    if (sReg[0x18] & LOWPASS_BITMASK) filtout += ftmp;    

    //output stage for one SID
    output = (nonfilt+filtout) * (sReg[0x18]&0xF) / OUTPUT_SCALEDOWN;
    if (output>=32767) output=32767; else if (output<=-32768) output=-32768; //saturation logic on overload (not needed if the callback handles it)
    return (int)output; // master output
}



unsigned int combinedWF(char num, char channel, unsigned int* wfarray, int index, char differ6581, byte freqh)
{
    if(differ6581 && SID_model[num]==6581) index &= 0x7FF; 
    return wfarray[index];
}


void createCombinedWF(unsigned int* wfarray, float bitmul, float bitstrength, float treshold) {
    int i,j,k;
    for (i=0; i<4096; i++) {
        wfarray[i]=0;
        for (j=0; j<12;j++) {
            float bitlevel=((i>>j)&1);
            for (k=0; k<12; k++) if (!((i>>k)&1)) bitlevel -= bitmul / pow(bitstrength, fabs(k-j));
            wfarray[i] += (bitlevel>=treshold)? pow(2,j) : 0;
        }
        wfarray[i]*=12;
    }
}

void createCombinedWF_old(unsigned int* wfarray, float bitmul, float bitstrength, float treshold)
{
    int  i,j,k;
    for (i=0; i<4096; i++) { 
        wfarray[i]=0;
        for (j=0; j<12;j++) {
            float bitlevel=0;
            for (k=0; k<12; k++) bitlevel += ( bitmul/pow(bitstrength,fabs(k-j)) ) * (((i>>k)&1)-0.5);
            wfarray[i] += (bitlevel>=treshold)? pow(2,j) : 0;
        }
        wfarray[i]*=12; 
    }
}