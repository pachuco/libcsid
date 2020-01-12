// Based on cSID - an attempt at a usable simple API
// cSID by Hermit (Mihaly Horvath), (Year 2016..2017) http://hermit.sidrip.com
// License: WTF - Do what the fuck you want with this code, but please mention me as its original author.

// Let's see if we can clean up this thing and make it more maintainable without breaking it...

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "libcsid.h"

typedef unsigned char byte;

// global constants and variables
#define C64_PAL_CPUCLK 985248.0
#define SID_CHANNEL_AMOUNT 3
#define PAL_FRAMERATE 49.4 //50.06 //50.0443427 //50.1245419 //(C64_PAL_CPUCLK/63/312.5), selected carefully otherwise some ADSR-sensitive tunes may suffer more.


int OUTPUT_SCALEDOWN = SID_CHANNEL_AMOUNT * 16 + 26; //compensation for main volume and also filter reso emphasis

//SID-emulation variables:
byte ADSRstate[9], expcnt[9], sourceMSBrise[9], prevSR[9];
const byte FILTSW[9] = {1,2,4,1,2,4,1,2,4};
unsigned long int prevwfout[9], sourceMSB[3], noise_LFSR[9];
long int prevlowpass[3], prevbandpass[3];
short int envcnt[9];
//player-related variables:
int SIDamount=1, SID_model[3]={8580,8580,8580}, requested_SID_model=-1, sampleratio;
unsigned int initaddr, playaddr, playaddf, SID_address[3]={0xD400,0,0};
byte memory[MAX_DATA_LEN], timermode[0x20], SIDtitle[0x20], SIDauthor[0x20], SIDinfo[0x20];
int subtune=0, tunelength=-1;
int subtune_amount = 0; 
long int samplerate = DEFAULT_SAMPLERATE;

//CPU (and CIA/VIC-IRQ) emulation constants and variables
const byte flagsw[]={0x01,0x21,0x04,0x24,0x00,0x40,0x08,0x28}, branchflag[]={0x80,0x40,0x01,0x02};
unsigned int PC=0, pPC=0, addr=0, storadd=0;
short int A=0, T=0, SP=0xFF; 
byte X=0, Y=0, IR=0, ST=0x00;  //STATUS-flags: N V - B D I Z C
char cycles=0, finished=0, dynCIA=0;

 #define VCR_SHUNT_6581 1500 //1500 //kOhm //cca 1.5 MOhm Rshunt across VCR FET drain and source (causing 220Hz bottom cutoff with 470pF integrator capacitors in old C64)
 #define VCR_FET_TRESHOLD 350 //192 //Vth (on cutoff numeric range 0..2048) for the VCR cutoff-frequency control FET below which it doesn't conduct
 #define CAP_6581 0.470 //0.470 //nF //filter capacitor value for 6581
 #define FILTER_DARKNESS_6581 33.0 //22.0 //the bigger the value, the darker the filter control is (that is, cutoff frequency increases less with the same cutoff-value)
 #define FILTER_DISTORTION_6581 0.0032 //0.0016 //the bigger the value the more of resistance-modulation (filter distortion) is applied for 6581 cutoff-control
 float cutoff_ratio_8580, cutoff_steepness_6581, cap_6581_reciprocal;
#ifdef LIBCSID_FULL
 //SID-emulation variables:
 float clock_ratio=22; 
 unsigned int ratecnt[9];
 unsigned long int phaseaccu[9], prevaccu[9];
 //float cutoff_ratio_6581, cutoff_bias_6581;
 //player-related variables:
 int framecnt=0, frame_sampleperiod = DEFAULT_SAMPLERATE/PAL_FRAMERATE; 
 //CPU (and CIA/VIC-IRQ) emulation constants and variables - avoiding internal/automatic variables to retain speed
 char CPUtime=0;
#else
 #define CLOCK_RATIO_DEFAULT C64_PAL_CPUCLK/DEFAULT_SAMPLERATE  //(50.0567520: lowest framerate where Sanxion is fine, and highest where Myth is almost fine)
 
 float clock_ratio=CLOCK_RATIO_DEFAULT;
 //SID-emulation variables:
 unsigned long int prevwavdata[9];
 long int phaseaccu[9], prevaccu[9];
 float ratecnt[9];
 //player-related variables:
 float framecnt=0, frame_sampleperiod = DEFAULT_SAMPLERATE/PAL_FRAMERATE; 
 //CPU (and CIA/VIC-IRQ) emulation constants and variables - avoiding internal/automatic variables to retain speed
 float CPUtime=0.0;
#endif

enum {
    GATE_BITMASK=0x01,     SYNC_BITMASK=0x02,         RING_BITMASK=0x04,   TEST_BITMASK=0x08,
    TRI_BITMASK=0x10,      SAW_BITMASK=0x20,          PULSE_BITMASK=0x40,  NOISE_BITMASK=0x80,
    HOLDZERO_BITMASK=0x10, DECAYSUSTAIN_BITMASK=0x40, ATTACK_BITMASK=0x80, LOWPASS_BITMASK=0x10,
    BANDPASS_BITMASK=0x20, HIGHPASS_BITMASK=0x40,     OFF3_BITMASK=0x80
};

//function prototypes
void cSID_init(int samplerate);
int SID(char num, unsigned int baseaddr); void initSID();
void initCPU (unsigned int mempos);  byte CPU (); 
void init (byte subtune);
void play(signed short *stream, int len );
unsigned int combinedWF(char num, char channel, unsigned int* wfarray, int index, char differ6581, byte freqh);
void createCombinedWF(unsigned int* wfarray, float bitmul, float bitstrength, float treshold);


//----------------------------- MAIN thread ----------------------------

void init (byte subt) {
    static long int timeout;
    subtune = subt;
    initCPU(initaddr);
    initSID();
    A=subtune;
    memory[1]=0x37;
    memory[0xDC05]=0;
    for(timeout=100000;timeout>=0;timeout--) if (CPU()) break; 
    
    if (timermode[subtune] || memory[0xDC05]) { //&& playaddf {   //CIA timing
        if (!memory[0xDC05]) {memory[0xDC04]=0x24; memory[0xDC05]=0x40;} //C64 startup-default
        frame_sampleperiod = (memory[0xDC04]+memory[0xDC05]*256)/clock_ratio;
    } else {
        frame_sampleperiod = samplerate/PAL_FRAMERATE;  //Vsync timing
    }
    //frame_sampleperiod = (memory[0xDC05]!=0 || (!timermode[subtune] && playaddf))? samplerate/PAL_FRAMERATE : (memory[0xDC04] + memory[0xDC05]*256) / clock_ratio; 
    if(playaddf==0) {
        playaddr = ((memory[1]&3)<2)? memory[0xFFFE]+memory[0xFFFF]*256 : memory[0x314]+memory[0x315]*256;
        printf("IRQ-playaddress:%4.4X\n",playaddr);
    } else { //player under KERNAL (Crystal Kingdom Dizzy)
        playaddr=playaddf;
        if (playaddr>=0xE000 && memory[1]==0x37) memory[1]=0x35;
    }
    initCPU(playaddr);
    framecnt=1;
    finished=0;
    CPUtime=0; 
}

void play(signed short *stream, int len ) { 
    int i,j,k, output;
    
    for(i=0; i<len; i++) {
        output = 0;
        framecnt--;
        if (framecnt<=0) {
            framecnt=frame_sampleperiod;
            finished=0;
            PC=playaddr;
            SP=0xFF;
        }
        // printf("%d  %f\n",framecnt,playtime); }
        
        #ifdef LIBCSID_FULL
         for (j=0; j<sampleratio; j++) {
             if (finished==0 && --cycles<=0) {
                 pPC=PC;
                 //RTS,RTI and IRQ player ROM return handling
                 if (CPU()>=0xFE || ( (memory[1]&3)>1 && pPC<0xE000 && (PC==0xEA31 || PC==0xEA81) ) ) {
                     finished=1;
                 }
                 if ( (addr==0xDC05 || addr==0xDC04) && (memory[1]&3) && timermode[subtune] ) {
                     frame_sampleperiod = (memory[0xDC04] + memory[0xDC05]*256) / clock_ratio;  //dynamic CIA-setting (Galway/Rubicon workaround)
                     dynCIA=1;
                 }
                 if(storadd>=0xD420 && storadd<0xD800 && (memory[1]&3)) {  //CJ in the USA workaround (writing above $d420, except SID2/SID3)
                     if ( !(SID_address[1]<=storadd && storadd<SID_address[1]+0x1F) && !(SID_address[2]<=storadd && storadd<SID_address[2]+0x1F) ) {
                         memory[storadd&0xD41F]=memory[storadd]; //write to $D400..D41F if not in SID2/SID3 address-space
                     }
                 }
             }
             for (k=0; k<SIDamount; k++) output += SID(k, SID_address[k]);
         } 
         output /= sampleratio;
        #else
         if (finished==0) { 
             while (CPUtime<=clock_ratio) {
                 pPC=PC;
                 //RTS,RTI and IRQ player ROM return handling
                 if (CPU()>=0xFE || ( (memory[1]&3)>1 && pPC<0xE000 && (PC==0xEA31 || PC==0xEA81) ) ) {
                     finished=1;
                     break;
                 } else {
                     CPUtime+=cycles;
                 }
                 if ( (addr==0xDC05 || addr==0xDC04) && (memory[1]&3) && timermode[subtune] ) {
                     frame_sampleperiod = (memory[0xDC04] + memory[0xDC05]*256) / clock_ratio;  //dynamic CIA-setting (Galway/Rubicon workaround)
                     dynCIA=1;
                 }
                 if(storadd>=0xD420 && storadd<0xD800 && (memory[1]&3)) {  //CJ in the USA workaround (writing above $d420, except SID2/SID3)
                     if ( !(SID_address[1]<=storadd && storadd<SID_address[1]+0x1F) && !(SID_address[2]<=storadd && storadd<SID_address[2]+0x1F) ) {
                         memory[storadd&0xD41F]=memory[storadd]; //write to $D400..D41F if not in SID2/SID3 address-space
                     }
                 }
                 //Whittaker player workarounds (if GATE-bit triggered too fast, 0 for some cycles then 1)
                 if(addr==0xD404 && !(memory[0xD404]&GATE_BITMASK)) ADSRstate[0]&=0x3E;
                 if(addr==0xD40B && !(memory[0xD40B]&GATE_BITMASK)) ADSRstate[1]&=0x3E;
                 if(addr==0xD412 && !(memory[0xD412]&GATE_BITMASK)) ADSRstate[2]&=0x3E;
             }
             CPUtime-=clock_ratio;
         }
         for (k=0; k<SIDamount; k++) output += SID(k, SID_address[k]);
        #endif
        stream[i]=output; 
    }
    
     //mix = SID(0,0xD400); if (SID_address[1]) mix += SID(1,SID_address[1]); if(SID_address[2]) mix += SID(2,SID_address[2]);
     //return mix * volume * SIDamount_vol[SIDamount] + (Math.random()*background_noise-background_noise/2); 
}

//--------------------------------- CPU emulation -------------------------------------------

void initCPU(unsigned int mempos) {
    PC=mempos; A=0; X=0; Y=0; ST=0; SP=0xFF;
} 

//My CPU implementation is based on the instruction table by Graham at codebase64.
//After some examination of the table it was clearly seen that columns of the table (instructions' 2nd nybbles)
// mainly correspond to addressing modes, and double-rows usually have the same instructions.
//The code below is laid out like this, with some exceptions present.
//Thanks to the hardware being in my mind when coding this, the illegal instructions could be added fairly easily...
byte CPU () { //the CPU emulation for SID/PRG playback (ToDo: CIA/VIC-IRQ/NMI/RESET vectors, BCD-mode)
    //'IR' is the instruction-register, naming after the hardware-equivalent
    IR=memory[PC]; cycles=2; storadd=0; //'cycle': ensure smallest 6510 runtime (for implied/register instructions)
    
    //nybble2:  1/5/9/D:accu.instructions, 3/7/B/F:illegal opcodes
    if(IR&1) {
        switch (IR&0x1F) { //addressing modes (begin with more complex cases), PC wraparound not handled inside to save codespace
            case 0x01:
            case 0x03: //(zp,x)
                addr  = memory[memory[++PC]+X];
                addr += memory[memory[PC]+X+1]*256;
                cycles=6;
                break;
            case 0x11:
            case 0x13: //(zp),y (5..6 cycles, 8 for R-M-W)
                addr  = memory[memory[++PC]];
                addr += memory[memory[PC]+1]*256 + Y;
                cycles=6;
                break;
            case 0x19:
            case 0x1B: //abs,y //(4..5 cycles, 7 cycles for R-M-W)
                addr  = memory[++PC];
                addr += memory[++PC]*256 + Y;
                cycles=5;
                break;
            case 0x1D: //abs,x //(4..5 cycles, 7 cycles for R-M-W)
                addr  = memory[++PC];
                addr += memory[++PC]*256 + X;
                cycles=5;
                break;
            case 0x0D:
            case 0x0F: //abs
                addr  = memory[++PC];
                addr += memory[++PC]*256;
                cycles=4;
                break;
            case 0x15: //zp,x
                addr = memory[++PC] + X;
                cycles=4;
                break; 
            case 0x05:
            case 0x07: //zp
                addr = memory[++PC];
                cycles=3;
                break; 
            case 0x17: 
                if ((IR&0xC0)!=0x80) { //zp,x for illegal opcodes
                    addr = memory[++PC] + X;
                    cycles=4;
                } else { //zp,y for LAX/SAX illegal opcodes
                    addr = memory[++PC] + Y;
                    cycles=4;
                }
                break;
            case 0x1F:
                if ((IR&0xC0)!=0x80) { //abs,x for illegal opcodes
                    addr  = memory[++PC];
                    addr += memory[++PC]*256 + X;
                    cycles=5;
                } else { //abs,y for LAX/SAX illegal opcodes
                    addr  = memory[++PC];
                    addr += memory[++PC]*256 + Y;
                    cycles=5;
                }
                break;
            case 0x09:
            case 0x0B: //immediate
                addr = ++PC;
                cycles=2;
        }
        
        addr&=0xFFFF;
        switch (IR&0xE0) {
            case 0x60: 
               if ((IR&0x1F)!=0xB) { //ADC / RRA (ROR+ADC)
                    if((IR&3)==3) {
                        T = (memory[addr]>>1)+(ST&1)*128;
                        ST&=124;
                        ST|=(T&1);
                        memory[addr]=T;
                        cycles+=2;
                    }
                    T=A;
                    A+=memory[addr]+(ST&1);
                    ST&=60;
                    ST|=(A&128)|(A>255);
                    A&=0xFF;
                    ST |= (!A)<<1 | ( !((T^memory[addr])&0x80) & ((T^A)&0x80) ) >> 1;
                } else { //V-flag set by intermediate ADC mechanism: (A&mem)+mem
                    A&=memory[addr];
                    T+=memory[addr]+(ST&1);
                    ST&=60;
                    ST |= (T>255) | ( !((A^memory[addr])&0x80) & ((T^A)&0x80) ) >> 1;
                    T=A;
                    A=(A>>1)+(ST&1)*128;
                    ST|=(A&128)|(T>127);
                    ST|=(!A)<<1;
                }
                // ARR (AND+ROR, bit0 not going to C, but C and bit7 get exchanged.)
                break;
            case 0xE0: //SBC / ISC(ISB)=INC+SBC
                if((IR&3)==3 && (IR&0x1F)!=0xB) {
                    memory[addr]++;cycles+=2;
                }
                T=A;
                A-=memory[addr]+!(ST&1);
                ST&=60;
                ST|=(A&128)|(A>=0);
                A&=0xFF;
                ST |= (!A)<<1 | ( ((T^memory[addr])&0x80) & ((T^A)&0x80) ) >> 1;
                break; 
            case 0xC0:
                if((IR&0x1F)!=0xB) { // CMP / DCP(DEC+CMP)
                    if ((IR&3)==3) {
                        memory[addr]--;
                        cycles+=2;
                    } 
                    T=A-memory[addr];
                } else { //SBX (AXS) (CMP+DEX at the same time)
                    X=T=(A&X)-memory[addr];
                }
                ST&=124;
                ST|=(!(T&0xFF))<<1|(T&128)|(T>=0);
                break;
            case 0x00:
                if ((IR&0x1F)!=0xB) { //ORA / SLO(ASO)=ASL+ORA
                    if ((IR&3)==3) {
                        ST&=124;
                        ST|=(memory[addr]>127);
                        memory[addr]<<=1;
                        cycles+=2;
                    }  
                    A|=memory[addr];
                    ST&=125;
                    ST|=(!A)<<1|(A&128);
                } else { //ANC (AND+Carry=bit7)
                    A&=memory[addr];
                    ST&=124;
                    ST|=(!A)<<1|(A&128)|(A>127);
                }
                break;
            case 0x20:
                if ((IR&0x1F)!=0xB) { //AND / RLA (ROL+AND)
                    if ((IR&3)==3) {
                        T=(memory[addr]<<1)+(ST&1);
                        ST&=124;
                        ST|=(T>255);
                        T&=0xFF;
                        memory[addr]=T;
                        cycles+=2;
                    }  
                    A&=memory[addr];
                    ST&=125;
                    ST|=(!A)<<1|(A&128); 
                } else { //ANC (AND+Carry=bit7)
                    A&=memory[addr];
                    ST&=124;
                    ST|=(!A)<<1|(A&128)|(A>127);
                }
                break;
            case 0x40:
                if ((IR&0x1F)!=0xB) { //EOR / SRE(LSE)=LSR+EOR
                    if ((IR&3)==3) {
                        ST&=124;
                        ST|=(memory[addr]&1);
                        memory[addr]>>=1;
                        cycles+=2;
                    }
                    A^=memory[addr];
                    ST&=125;
                    ST|=(!A)<<1|(A&128);
                } else { //ALR(ASR)=(AND+LSR)
                    A&=memory[addr];
                    ST&=124;
                    ST|=(A&1);
                    A>>=1;
                    A&=0xFF;
                    ST|=(A&128)|((!A)<<1);
                }
                break;
            case 0xA0:
                if ((IR&0x1F)!=0x1B) { //LDA / LAX (illegal, used by my 1 rasterline player)
                        A=memory[addr];
                        if ((IR&3)==3) X=A;
                    } else { // LAS (LAR)
                        A=X=SP=memory[addr]&SP;
                    }
                    ST&=125;
                    ST|=((!A)<<1) | (A&128);
                    break;
            case 0x80:
                if ((IR&0x1F)==0xB) { //XAA (TXA+AND), highly unstable on real 6502!
                    A = X & memory[addr];
                    ST&=125;
                    ST|=(A&128) | ((!A)<<1);
                } else if ((IR&0x1F)==0x1B) { //TAS(SHS) (SP=A&X, mem=S&H} - unstable on real 6502
                    SP=A&X;
                    memory[addr]=SP&((addr>>8)+1);
                } else { //STA / SAX (at times same as AHX/SHX/SHY) (illegal) 
                    memory[addr]=A & (((IR&3)==3)?X:0xFF);
                    storadd=addr;
                }
                break;
        }
    //nybble2:  2:illegal/LDX, 6:A/X/INC/DEC, A:Accu-shift/reg.transfer/NOP, E:shift/X/INC/DEC
    } else if(IR&2) {
        switch (IR&0x1F) { //addressing modes
            case 0x1E: //abs,x / abs,y
                addr  = memory[++PC];
                addr += memory[++PC]*256 + ( ((IR&0xC0)!=0x80) ? X:Y );
                cycles=5;
                break;
            case 0x0E: //abs
                addr  = memory[++PC];
                addr += memory[++PC]*256;
                cycles=4;
                break;
            case 0x16: //zp,x / zp,y
                addr = memory[++PC] + ( ((IR&0xC0)!=0x80) ? X:Y );
                cycles=4;
                break;
            case 0x06: //zp
                addr = memory[++PC];
                cycles=3;
                break;
            case 0x02: //imm.
                addr = ++PC;
                cycles=2;
        }
        
        addr&=0xFFFF; 
        switch (IR&0xE0) {
            case 0x00:
                ST&=0xFE;
            case 0x20:
                if((IR&0xF)==0xA) { //ASL/ROL (Accu)
                    A=(A<<1)+(ST&1);
                    ST&=124;
                    ST|=(A&128)|(A>255);
                    A&=0xFF;
                    ST|=(!A)<<1;
                } else { //RMW (Read-Write-Modify)
                    T=(memory[addr]<<1)+(ST&1);
                    ST&=124;
                    ST|=(T&128)|(T>255);
                    T&=0xFF;
                    ST|=(!T)<<1;
                    memory[addr]=T;
                    cycles+=2;
                }
                break;
            case 0x40:
                ST&=0xFE;
            case 0x60:
                if((IR&0xF)==0xA) { //LSR/ROR (Accu)
                    T=A;
                    A=(A>>1)+(ST&1)*128;
                    ST&=124;
                    ST|=(A&128)|(T&1);
                    A&=0xFF;
                    ST|=(!A)<<1;
                } else { //memory (RMW)
                    T=(memory[addr]>>1)+(ST&1)*128;
                    ST&=124;
                    ST|=(T&128)|(memory[addr]&1);
                    T&=0xFF;
                    ST|=(!T)<<1;
                    memory[addr]=T;
                    cycles+=2;
                }
                break;
            case 0xC0:
                if(IR&4) { //DEC
                    memory[addr]--;
                    ST&=125;
                    ST|=(!memory[addr])<<1|(memory[addr]&128);
                    cycles+=2;
                } else { //DEX
                    X--;
                    X&=0xFF;
                    ST&=125;
                    ST|=(!X)<<1|(X&128);
                }
                break;
            case 0xA0: //LDX/TSX/TAX
                if((IR&0xF)!=0xA) {
                    X=memory[addr];
                } else if(IR&0x10) {
                    X=SP;
                    break;
                } else {
                    X=A;
                }
                ST&=125;
                ST|=(!X)<<1|(X&128);
                break;
            case 0x80: //STX/TXS/TXA
                if(IR&4) {
                    memory[addr]=X;
                    storadd=addr;
                } else if(IR&0x10) {
                    SP=X;
                } else {
                    A=X;
                    ST&=125;
                    ST|=(!A)<<1|(A&128);
                } 
                break;
            case 0xE0: //INC/NOP
                if(IR&4) {
                    memory[addr]++;
                    ST&=125;
                    ST|=(!memory[addr])<<1|(memory[addr]&128);
                    cycles+=2;
                }
        }
    //nybble2:  8:register/status
    } else if((IR&0xC)==8) {
        switch (IR&0xF0) {
            case 0x60: //PLA
                SP++;
                SP&=0xFF;
                A=memory[0x100+SP];
                ST&=125;
                ST|=(!A)<<1|(A&128);
                cycles=4;
                break;
            case 0xC0: //INY
                Y++;
                Y&=0xFF;
                ST&=125;
                ST|=(!Y)<<1|(Y&128);
                break;
            case 0xE0: //INX
                X++;
                X&=0xFF;
                ST&=125;
                ST|=(!X)<<1|(X&128);
                break;
            case 0x80: //DEY
                Y--;
                Y&=0xFF;
                ST&=125;
                ST|=(!Y)<<1|(Y&128);
                break;
            case 0x00: //PHP
                memory[0x100+SP]=ST;
                SP--;
                SP&=0xFF;
                cycles=3;
                break;
            case 0x20: //PLP
                SP++;
                SP&=0xFF;
                ST=memory[0x100+SP];
                cycles=4;
                break;
            case 0x40:  //PHA
                memory[0x100+SP]=A;
                SP--;
                SP&=0xFF;
                cycles=3;
                break;
            case 0x90:  //TYA
                A=Y;
                ST&=125;
                ST|=(!A)<<1|(A&128);
                break;
            case 0xA0: //TAY
                Y=A;
                ST&=125;
                ST|=(!Y)<<1|(Y&128);
                break;
            default: //CLC/SEC/CLI/SEI/CLV/CLD/SED
                if(flagsw[IR>>5]&0x20) {
                    ST|=(flagsw[IR>>5]&0xDF);
                } else {
                    ST&=255-(flagsw[IR>>5]&0xDF);
                }
        }
    //nybble2:  0: control/branch/Y/compare  4: Y/compare  C:Y/compare/JMP
    } else {
        if ((IR&0x1F)==0x10) { //BPL/BMI/BVC/BVS/BCC/BCS/BNE/BEQ  relative branch 
            PC++; T=memory[PC];
            if(T&0x80) T-=0x100;
            if(IR&0x20) {
                if (ST&branchflag[IR>>6]) {
                    PC+=T;
                    cycles=3;
                }
            } else {
                if (!(ST&branchflag[IR>>6])) {
                    PC+=T;
                    cycles=3;
                }
            }
        } else {  //nybble2:  0:Y/control/Y/compare  4:Y/compare  C:Y/compare/JMP
            switch (IR&0x1F) { //addressing modes
                case 0x00: //imm. (or abs.low for JSR/BRK)
                    addr = ++PC;
                    cycles=2;
                    break;
                case 0x1C: //abs,x
                    addr  = memory[++PC];
                    addr += memory[++PC]*256 + X;
                    cycles=5;
                    break;
                case 0x0C: //abs
                    addr  = memory[++PC];
                    addr += memory[++PC]*256;
                    cycles=4;
                    break;
                case 0x14: //zp,x
                    addr = memory[++PC] + X;
                    cycles=4;
                    break;
                case 0x04: //zp
                    addr = memory[++PC];
                    cycles=3;
            }
            
            addr&=0xFFFF;  
            switch (IR&0xE0) {
            case 0x00: //BRK
                memory[0x100+SP]=PC%256;
                SP--;
                SP&=0xFF;
                memory[0x100+SP]=PC/256;
                SP--;
                SP&=0xFF;
                memory[0x100+SP]=ST;
                SP--;SP&=0xFF; 
                PC = memory[0xFFFE]+memory[0xFFFF]*256-1;
                cycles=7;
                break;
            case 0x20:
                if(IR&0xF) { //BIT
                    ST &= 0x3D;
                    ST |= (memory[addr]&0xC0) | ( !(A&memory[addr]) )<<1;
                } else { //JSR
                    memory[0x100+SP]=(PC+2)%256;
                    SP--;
                    SP&=0xFF;
                    memory[0x100+SP]=(PC+2)/256;
                    SP--;SP&=0xFF;
                    PC=memory[addr]+memory[addr+1]*256-1;
                    cycles=6;
                }
                break;
            case 0x40:
                if(IR&0xF) { //JMP
                    PC = addr-1;
                    cycles=3;
                } else { //RTI
                    if(SP>=0xFF) return 0xFE;
                    SP++;
                    SP&=0xFF;
                    ST=memory[0x100+SP];
                    SP++;
                    SP&=0xFF;
                    T=memory[0x100+SP];
                    SP++;
                    SP&=0xFF;
                    PC=memory[0x100+SP]+T*256-1;
                    cycles=6;
                }
                break;
            case 0x60:
                if(IR&0xF) { //JMP() (indirect)
                    PC = memory[addr]+memory[addr+1]*256-1;
                    cycles=5;
                } else { //RTS
                    if(SP>=0xFF) return 0xFF;
                    SP++;
                    SP&=0xFF;
                    T=memory[0x100+SP];
                    SP++;
                    SP&=0xFF;
                    PC=memory[0x100+SP]+T*256-1;
                    cycles=6;
                }
                break;
            case 0xC0: //CPY
                T=Y-memory[addr];
                ST&=124;
                ST|=(!(T&0xFF))<<1|(T&128)|(T>=0);
                break;
            case 0xE0: //CPX
                T=X-memory[addr];
                ST&=124;
                ST|=(!(T&0xFF))<<1|(T&128)|(T>=0);
                break;
            case 0xA0: //LDY
                Y=memory[addr];
                ST&=125;
                ST|=(!Y)<<1|(Y&128);
                break;
            case 0x80: //STY
                memory[addr]=Y;
                storadd=addr;
            }
        }
    }
    
    //if(IR==0xCB) //test SBX
    // printf("PC:%4.4X IR:%2.2X, addr: %4.4X,%2.2X,  storadd: %4.4X,%2.2X,  A:%2.2X, X:%2.2X, Y:%2.2X ST:%2.2X\n",PC,IR,addr,memory[addr],storadd,memory[storadd],A,X,Y,ST);  
    
    PC++; //PC&=0xFFFF; 
    return 0; 
}

//----------------------------- SID emulation -----------------------------------------

unsigned int TriSaw_8580[4096], PulseSaw_8580[4096], PulseTriSaw_8580[4096];

const byte ADSR_exptable[256] = {1, 30, 30, 30, 30, 30, 30, 16, 16, 16, 16, 16, 16, 16, 16, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 4, 4, 4, 4, 4, //pos0:1  pos6:30  pos14:16  pos26:8
    4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, //pos54:4 //pos93:2
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1
};

#ifdef LIBCSID_FULL
 int ADSRperiods[16] = {9, 32, 63, 95, 149, 220, 267, 313, 392, 977, 1954, 3126, 3907, 11720, 19532, 31251};
#else
 #define PERIOD0 CLOCK_RATIO_DEFAULT //max(round(clock_ratio),9)
 #define STEP0 3 //ceil(PERIOD0/9.0)
 float ADSRperiods[16] = {PERIOD0, 32, 63, 95, 149, 220, 267, 313, 392, 977, 1954, 3126, 3907, 11720, 19532, 31251};
 byte ADSRstep[16] =   {  STEP0, 1,  1,  1,  1,    1,   1,   1,   1,   1,    1,    1,    1,     1,     1,     1};
#endif



//registers: 0:freql1  1:freqh1  2:pwml1  3:pwmh1  4:ctrl1  5:ad1   6:sr1  7:freql2  8:freqh2  9:pwml2 10:pwmh2 11:ctrl2 12:ad2  13:sr 14:freql3 15:freqh3 16:pwml3 17:pwmh3 18:ctrl3 19:ad3  20:sr3 
//           21:cutoffl 22:cutoffh 23:flsw_reso 24:vol_ftype 25:potX 26:potY 27:OSC3 28:ENV3
void initSID() { 
    int i;
    for(i=0xD400;i<=0xD7FF;i++) memory[i]=0;
    for(i=0xDE00;i<=0xDFFF;i++) memory[i]=0;
    for(i=0;i<9;i++) {
        ADSRstate[i]=HOLDZERO_BITMASK;
        ratecnt[i]=envcnt[i]=expcnt[i]=0;
    } 
}

void cSID_init(int samplerate) {
    int i;
    
    cap_6581_reciprocal = -1000000/CAP_6581;
    cutoff_steepness_6581 = FILTER_DARKNESS_6581*(2048.0-VCR_FET_TRESHOLD);
    #ifdef LIBCSID_FULL
     clock_ratio = round(C64_PAL_CPUCLK/samplerate);
     cutoff_ratio_8580 = -2 * 3.14 * (12500.0 / 2048) / C64_PAL_CPUCLK;
     //cutoff_ratio_6581 = -2 * 3.14 * (20000.0 / 2048) / C64_PAL_CPUCLK;
     //cutoff_bias_6581 = 1 - exp( -2 * 3.14 * 220 / C64_PAL_CPUCLK ); //around 220Hz below treshold
     
     createCombinedWF(TriSaw_8580, 0.5, 2.2, 0.9);
     createCombinedWF(PulseSaw_8580, 0.23, 1.27, 0.55);
     createCombinedWF(PulseTriSaw_8580, 0.5, 1.6, 0.8);
    #else
     clock_ratio = C64_PAL_CPUCLK/samplerate;
     if (clock_ratio>9) {
         ADSRperiods[0]=clock_ratio;
         ADSRstep[0]=ceil(clock_ratio/9.0);
     } else {
         ADSRperiods[0]=9.0;
         ADSRstep[0]=1;
     }
     cutoff_ratio_8580 = -2 * 3.14 * (12500 / 2048) / samplerate;
     
     createCombinedWF(TriSaw_8580, 0.8, 2.4, 0.64);
     createCombinedWF(PulseSaw_8580, 1.4, 1.9, 0.68);
     createCombinedWF(PulseTriSaw_8580, 0.8, 2.5, 0.64);
    #endif
    
    for(i = 0; i < 9; i++) {
        ADSRstate[i] = HOLDZERO_BITMASK;
        envcnt[i] = 0;
        ratecnt[i] = 0; 
        phaseaccu[i] = 0;
        prevaccu[i] = 0;
        expcnt[i] = 0; 
        noise_LFSR[i] = 0x7FFFFF;
        prevwfout[i] = 0;
        prevSR[i]=0;
    }
    for(i = 0; i < 3; i++) {
        sourceMSBrise[i] = 0; sourceMSB[i] = 0;
        prevlowpass[i] = 0; prevbandpass[i] = 0;
    }
    initSID();
}

//The anatomy of combined waveforms: The resid source simply uses 4kbyte 8bit samples from wavetable arrays, says these waveforms are mystic due to the analog behaviour.
//It's true, the analog things inside SID play a significant role in how the combined waveforms look like, but process variations are not so huge that cause much differences in SIDs.
//After checking these waveforms by eyes, it turned out for me that these waveform are fractal-like, recursively approachable waveforms.
//My 1st thought and trial was to store only a portion of the waveforms in table, and magnify them depending on phase-accumulator's state.
//But I wanted to understand how these waveforms are produced. I felt from the waveform-diagrams that the bits of the waveforms affect each other,
//hence the recursive look. A short C code proved by assumption, I could generate something like a pulse+saw combined waveform.
//Recursive calculations were not feasible for MCU of SwinSID, but for jsSID I could utilize what I found out and code below generates the combined waveforms into wavetables. 
//To approach the combined waveforms as much as possible, I checked out the SID schematic that can be found at some reverse-engineering sites...
//The SID's R-2R ladder WAVE DAC is driven by operation-amplifier like complementary FET output drivers, so that's not the place where I first thought the magic happens.
//These 'opamps' (for all 12 wave-bits) have single FETs as inputs, and they switch on above a certain level of input-voltage, causing 0 or 1 bit as R-2R DAC input.
//So the first keyword for the workings is TRESHOLD. These FET inputs are driven through serial switch FETs (wave-selector) that normally enables one waveform at a time.
//The phase-accumulator's output is brought to 3 kinds of circuitries for the 3 basic waveforms. The pulse simply drives
//all wave-selector inputs with a 0/1 depending on pulsewidth, the sawtooth has a XOR for triangle/ringmod generation, but what
//is common for all waveforms, they have an open-drain driver before the wave-selector, which has FETs towards GND and 'FET resistor' towards the power-supply rail.
//These outputs are clearly not designed to drive high loads, and normally they only have to drive the FETs input mentioned above.
//But when more of these output drivers are switched together by the switch-FETs in the wave-selector, they affect each other by loading each other.
//The pulse waveform, when selected, connects all of them together through a fairly strong connection, and its signal also affects the analog level (pulls below the treshold)...
//The farther a specific DAC bit driver is from the other, the less it affects its output. It turned out it's not powers of 2 but something else,
//that creates similar combined waveforms to that of real SID's... Note that combined waveforms never have values bigger than their sourcing sawtooth wave.
//The analog levels that get generated by the various bit drivers, that pull each other up/DOWN, depend on the resistances the components/wires inside the SID.
//And finally, what is output on the DAC depends on whether these analog levels are below or above the FET gate's treshold-level,
//That's how the combined waveform is generated. Maybe I couldn't explain well enough, but the code below is simple enough to understand the mechanism algoritmically.
//This simplified schematic exapmle might make it easier to understand sawtooth+pulse combination (must be observed with monospace fonts):
//                               _____            |-    .--------------.   /\/\--.
// Vsupply                /  .----| |---------*---|-    /    Vsupply   !    R    !      As can be seen on this schematic,
//  ------.       other   !  !   _____        !  TRES   \       \      !         /      the pulse wave-selector FETs 
//        !       saw bit *--!----| |---------'  HOLD   /       !     |-     2R  \      connect the neighbouring sawtooth
//        /       output  !  !                          !      |------|-         /      outputs with a fairly strong 
//     Rd \              |-  !WAVEFORM-SELECTOR         *--*---|-      !    R    !      connection to each other through
//        /              |-  !SWITCHING FETs            !  !    !      *---/\/\--*      their own wave-selector FETs.
//        ! saw-bit          !    _____                |-  !   ---     !         !      So the adjacent sawtooth outputs
//        *------------------!-----| |-----------*-----|-  !          |-         /      pull each other lower (or maybe a bit upper but not exceeding sawtooth line)
//        ! (weak drive,so   !  saw switch       ! TRES-!  `----------|-     2R  \      depending on their low/high state and
//       |- can be shifted   !                   ! HOLD !              !         /      distance from each other, causing
//  -----|- down (& up?)     !    _____          !      !              !     R   !      the resulting analog level that
//        ! by neighbours)   *-----| |-----------'     ---            ---   /\/\-*      will either turn the output on or not.
//   GND ---                 !  pulse switch                                     !      (Depending on their relation to treshold.)
//
//(As triangle waveform connects adjacent bits by default, the above explained effect becomes even stronger, that's why combined waveforms with thriangle are at 0 level most of the time.)

//in case you don't like these calculated combined waveforms it's easy to substitute the generated tables by pre-sampled 'exact' versions

unsigned int combinedWF(char num, char channel, unsigned int* wfarray, int index, char differ6581, byte freqh) {
    #ifdef LIBCSID_FULL
     if(differ6581 && SID_model[num]==6581) index &= 0x7FF; 
     return wfarray[index];
    #else
     static float addf;
     addf = 0.6+0.4/freqh;
     
     if(differ6581 && SID_model[num]==6581) index&=0x7FF; 
     prevwavdata[channel] = wfarray[index]*addf + prevwavdata[channel]*(1.0-addf);
     return prevwavdata[channel];
    #endif
}

void createCombinedWF(unsigned int* wfarray, float bitmul, float bitstrength, float treshold) {
    int i,j,k;
    for (i=0; i<4096; i++) {
        wfarray[i]=0;
        for (j=0; j<12;j++) {
            #ifdef LIBCSID_FULL
             float bitlevel=((i>>j)&1);
             for (k=0; k<12; k++) if (!((i>>k)&1)) bitlevel -= bitmul / pow(bitstrength, fabs(k-j));
            #else
             float bitlevel=0; 
             for (k=0; k<12; k++) bitlevel += ( bitmul/pow(bitstrength,fabs(k-j)) ) * (((i>>k)&1)-0.5);
            #endif
            wfarray[i] += (bitlevel>=treshold)? pow(2,j) : 0;
        }
        wfarray[i]*=12;
    }
}









//
// libcsid exported api
//

const char *libcsid_getauthor() {
    return (char *)&SIDauthor;
}

const char *libcsid_getinfo() {
    return (char *)&SIDinfo;
}

const char *libcsid_gettitle() {
    return (char *)&SIDtitle;
}

const int libcsid_getsubtunenum() {
    return subtune_amount;
}

void libcsid_init(int _samplerate, int _sidmodel) {
    samplerate = _samplerate;
    sampleratio = round(C64_PAL_CPUCLK / samplerate);
    requested_SID_model = _sidmodel;
}

int libcsid_load(unsigned char *_buffer, int _bufferlen, int _subtune) {
    int readata, strend, preferred_SID_model[3] = {8580, 8580, 8580};
    unsigned int i, datalen, offs, loadaddr;
    
    subtune = _subtune;
    
    unsigned char *filedata = _buffer;
    datalen = _bufferlen;
    
    offs = filedata[7];
    loadaddr = filedata[8] + filedata[9] ? filedata[8] * 256 + filedata[9] : filedata[offs] + filedata[offs + 1] * 256;
    printf("\nOffset: $%4.4X, Loadaddress: $%4.4X \nTimermodes:", offs, loadaddr);
    
    for (i = 0; i < 32; i++) {
        timermode[31 - i] = (filedata[0x12 + (i >> 3)] & (byte)pow(2, 7 - i % 8)) ? 1 : 0;
        printf(" %1d",timermode[31 - i]);
    }
    
    for (i = 0; i < MAX_DATA_LEN; i++) {
        memory[i] = 0;
    }
    
    for (i = offs + 2; i < datalen; i++) {
        if (loadaddr + i - (offs + 2) < MAX_DATA_LEN) {
            memory[loadaddr + i - (offs + 2)] = filedata[i];
        }
    }
    
    strend = 1;
    for (i = 0; i < 32; i++) {
        if (strend != 0) {
            strend = SIDtitle[i] = filedata[0x16 + i];
        } else {
            strend = SIDtitle[i] = 0;
        }
    }
    
    strend = 1;
    for (i = 0; i < 32; i++) {
        if (strend != 0) {
            strend = SIDauthor[i] = filedata[0x36 + i];
        } else {
            strend = SIDauthor[i] = 0;
        }
    }
    
    strend = 1;
    for (i = 0; i < 32; i++) {
        if (strend != 0) {
            strend = SIDinfo[i] = filedata[0x56 + i];
        } else {
            strend = SIDinfo[i] = 0;
        }
    }
    
    printf("\nTitle: %s    ",SIDtitle);
    printf("Author: %s    ",SIDauthor);
    printf("Info: %s",SIDinfo);
    
    initaddr=filedata[0xA]+filedata[0xB]? filedata[0xA]*256+filedata[0xB] : loadaddr; playaddr=playaddf=filedata[0xC]*256+filedata[0xD]; printf("\nInit:$%4.4X,Play:$%4.4X, ",initaddr,playaddr);
    subtune_amount=filedata[0xF];
    preferred_SID_model[0] = (filedata[0x77]&0x30)>=0x20? 8580 : 6581;
    printf("Subtunes:%d , preferred SID-model:%d", subtune_amount, preferred_SID_model[0]);
    preferred_SID_model[1] = (filedata[0x77]&0xC0)>=0x80 ? 8580 : 6581;
    preferred_SID_model[2] = (filedata[0x76]&3)>=3 ? 8580 : 6581; 
    SID_address[1] = filedata[0x7A]>=0x42 && (filedata[0x7A]<0x80 || filedata[0x7A]>=0xE0) ? 0xD000+filedata[0x7A]*16 : 0;
    SID_address[2] = filedata[0x7B]>=0x42 && (filedata[0x7B]<0x80 || filedata[0x7B]>=0xE0) ? 0xD000+filedata[0x7B]*16 : 0;
    
    SIDamount=1+(SID_address[1]>0)+(SID_address[2]>0); if(SIDamount>=2) printf("(SID1), %d(SID2:%4.4X)",preferred_SID_model[1],SID_address[1]); 
    if(SIDamount==3) printf(", %d(SID3:%4.4X)",preferred_SID_model[2],SID_address[2]);
    if (requested_SID_model!=-1) printf(" (requested:%d)",requested_SID_model); printf("\n");
    
    for (i=0;i<SIDamount;i++) {
        if (requested_SID_model==8580 || requested_SID_model==6581) {
            SID_model[i] = requested_SID_model;
        } else {
            SID_model[i] = preferred_SID_model[i];
        }
    }
    
    OUTPUT_SCALEDOWN = SID_CHANNEL_AMOUNT * 16 + 26;
    if (SIDamount == 2) {
      OUTPUT_SCALEDOWN /= 0.6;
    } else if (SIDamount >= 3) {
      OUTPUT_SCALEDOWN /= 0.4;
    }
    
    cSID_init(samplerate);
    init(subtune);
    
    return 0;
}

void libcsid_render(signed short *_output, int _numsamples) {
    play(_output, _numsamples);
}