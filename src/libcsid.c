// Based on cSID - an attempt at a usable simple API
// cSID by Hermit (Mihaly Horvath), (Year 2016..2017) http://hermit.sidrip.com
// License: WTF - Do what the fuck you want with this code, but please mention me as its original author.

// Let's see if we can clean up this thing and make it more maintainable without breaking it...

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "libcsid.h"

float cutoff_steepness_6581, cap_6581_reciprocal;
unsigned int TriSaw_8580[4096], PulseSaw_8580[4096], PulseTriSaw_8580[4096];

const byte flagsw[]={0x01,0x21,0x04,0x24,0x00,0x40,0x08,0x28}, branchflag[]={0x80,0x40,0x01,0x02};
const byte FILTSW[9] = {1,2,4,1,2,4,1,2,4};

const byte ADSR_exptable[256] = {
    1, 30, 30, 30, 30, 30, 30, 16, 16, 16, 16, 16, 16, 16, 16, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 4, 4, 4, 4, 4, //pos0:1  pos6:30  pos14:16  pos26:8
    4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, //pos54:4 //pos93:2
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1
};

const int ADSRperiods[16] = {9, 32, 63, 95, 149, 220, 267, 313, 392, 977, 1954, 3126, 3907, 11720, 19532, 31251};

enum {
    GATE_BITMASK=0x01,     SYNC_BITMASK=0x02,         RING_BITMASK=0x04,     TEST_BITMASK=0x08,
    TRI_BITMASK=0x10,      SAW_BITMASK=0x20,          PULSE_BITMASK=0x40,    NOISE_BITMASK=0x80,
    HOLDZERO_BITMASK=0x10, DECAYSUSTAIN_BITMASK=0x40, ATTACK_BITMASK=0x80,
    LOWPASS_BITMASK=0x10,  BANDPASS_BITMASK=0x20,     HIGHPASS_BITMASK=0x40, OFF3_BITMASK=0x80
};

//function prototypes
void cSID_init(CsidPlayer* self, int samplerate);
int SID(CsidPlayer* self, char num, unsigned int baseaddr);
void initSID(CsidPlayer* self);
void initCPU(CsidPlayer* self, unsigned int mempos);
byte CPU(CsidPlayer* self); 
void init(CsidPlayer* self, byte subtune);
unsigned int combinedWF(CsidPlayer* self, char num, char channel, unsigned int* wfarray, int index, char differ6581, byte freqh);
void createCombinedWF(unsigned int* wfarray, float bitmul, float bitstrength, float treshold);


//----------------------------- MAIN thread ----------------------------

void init(CsidPlayer* self, byte subtune) {
    long int timeout;
    self->curSubtune = subtune;
    initCPU(self, self->initaddr);
    initSID(self);
    self->A=subtune;
    self->memory[1]=0x37;
    self->memory[0xDC05]=0;
    for(timeout=100000;timeout>=0;timeout--) if (CPU(self)) break; 
    
    if (self->timermode[subtune] || self->memory[0xDC05]) { //&& self->playaddf {   //CIA timing
        if (!self->memory[0xDC05]) { //C64 startup-default
            self->memory[0xDC04]=0x24;
            self->memory[0xDC05]=0x40;
        }
        self->frame_sampleperiod = (self->memory[0xDC04]+self->memory[0xDC05]*256)/self->clock_ratio;
    } else {
        self->frame_sampleperiod = self->samplerate/PAL_FRAMERATE;  //Vsync timing
    }
    //frame_sampleperiod = (memory[0xDC05]!=0 || (!timermode[subtune] && playaddf))? samplerate/PAL_FRAMERATE : (memory[0xDC04] + memory[0xDC05]*256) / clock_ratio; 
    if(self->playaddf==0) {
        self->playaddr = ((self->memory[1]&3)<2)? self->memory[0xFFFE]+self->memory[0xFFFF]*256 : self->memory[0x314]+self->memory[0x315]*256;
        printf("IRQ-playaddress:%4.4X\n",self->playaddr);
    } else { //player under KERNAL (Crystal Kingdom Dizzy)
        self->playaddr=self->playaddf;
        if (self->playaddr>=0xE000 && self->memory[1]==0x37) self->memory[1]=0x35;
    }
    initCPU(self, self->playaddr);
    self->framecnt=1;
    self->finished=0;
    self->CPUtime=0;
    self->dynCIA=0;
}

void libcsid_render(CsidPlayer* self, signed short *stream, int numSamples) { 
    int i,j,k, output;
    
    for(i = 0; i < numSamples; i++) {
        output = 0;
        self->framecnt--;
        if (self->framecnt<=0) {
            self->framecnt=self->frame_sampleperiod;
            self->finished=0;
            self->PC=self->playaddr;
            self->SP=0xFF;
        }
        // printf("%d  %f\n",framecnt,playtime); }
        
        #ifdef LIBCSID_FULL
         for (j=0; j<self->sampleratio; j++) {
             if (self->finished==0 && --self->cycles<=0) {
                 self->pPC=self->PC;
                 //RTS,RTI and IRQ player ROM return handling
                 if (CPU(self)>=0xFE || ( (self->memory[1]&3)>1 && self->pPC<0xE000 && (self->PC==0xEA31 || self->PC==0xEA81) ) ) {
                     self->finished=1;
                 }
                 if ( (self->addr==0xDC05 || self->addr==0xDC04) && (self->memory[1]&3) && self->timermode[self->curSubtune] ) {
                     self->frame_sampleperiod = (self->memory[0xDC04] + self->memory[0xDC05]*256) / self->clock_ratio;  //dynamic CIA-setting (Galway/Rubicon workaround)
                     self->dynCIA=1;
                 }
                 if(self->storadd>=0xD420 && self->storadd<0xD800 && (self->memory[1]&3)) {  //CJ in the USA workaround (writing above $d420, except SID2/SID3)
                     if ( !(self->SID_address[1]<=self->storadd && self->storadd<self->SID_address[1]+0x1F) && !(self->SID_address[2]<=self->storadd && self->storadd<self->SID_address[2]+0x1F) ) {
                         self->memory[self->storadd&0xD41F]=self->memory[self->storadd]; //write to $D400..D41F if not in SID2/SID3 address-space
                     }
                 }
             }
             for (k=0; k<self->SIDamount; k++) output += SID(self, k, self->SID_address[k]);
         } 
         output /= self->sampleratio;
        #else
         if (self->finished==0) { 
             while (self->CPUtime<=self->clock_ratio) {
                 self->pPC=self->PC;
                 //RTS,RTI and IRQ player ROM return handling
                 if (CPU(self)>=0xFE || ( (self->memory[1]&3)>1 && self->pPC<0xE000 && (self->PC==0xEA31 || self->PC==0xEA81) ) ) {
                     self->finished=1;
                     break;
                 } else {
                     self->CPUtime+=self->cycles;
                 }
                 if ( (self->addr==0xDC05 || self->addr==0xDC04) && (self->memory[1]&3) && self->timermode[self->curSubtune] ) {
                     self->frame_sampleperiod = (self->memory[0xDC04] + self->memory[0xDC05]*256) / self->clock_ratio;  //dynamic CIA-setting (Galway/Rubicon workaround)
                     self->dynCIA=1;
                 }
                 if(self->storadd>=0xD420 && self->storadd<0xD800 && (self->memory[1]&3)) {  //CJ in the USA workaround (writing above $d420, except SID2/SID3)
                     if ( !(self->SID_address[1]<=self->storadd && self->storadd<self->SID_address[1]+0x1F) && !(self->SID_address[2]<=self->storadd && self->storadd<self->SID_address[2]+0x1F) ) {
                         self->memory[self->storadd&0xD41F]=self->memory[self->storadd]; //write to $D400..D41F if not in SID2/SID3 address-space
                     }
                 }
                 //Whittaker player workarounds (if GATE-bit triggered too fast, 0 for some cycles then 1)
                 if(self->addr==0xD404 && !(self->memory[0xD404]&GATE_BITMASK)) self->ADSRstate[0]&=0x3E;
                 if(self->addr==0xD40B && !(self->memory[0xD40B]&GATE_BITMASK)) self->ADSRstate[1]&=0x3E;
                 if(self->addr==0xD412 && !(self->memory[0xD412]&GATE_BITMASK)) self->ADSRstate[2]&=0x3E;
             }
             self->CPUtime-=self->clock_ratio;
         }
         for (k=0; k<self->SIDamount; k++) output += SID(self, k, self->SID_address[k]);
        #endif
        stream[i]=output; 
    }
    
     //mix = SID(0,0xD400); if (SID_address[1]) mix += SID(1,SID_address[1]); if(SID_address[2]) mix += SID(2,SID_address[2]);
     //return mix * volume * SIDamount_vol[SIDamount] + (Math.random()*background_noise-background_noise/2); 
}

//--------------------------------- CPU emulation -------------------------------------------

void initCPU(CsidPlayer* self, unsigned int mempos) {
    self->PC=mempos; self->A=0; self->X=0; self->Y=0; self->ST=0; self->SP=0xFF;
} 

//My CPU implementation is based on the instruction table by Graham at codebase64.
//After some examination of the table it was clearly seen that columns of the table (instructions' 2nd nybbles)
// mainly correspond to addressing modes, and double-rows usually have the same instructions.
//The code below is laid out like this, with some exceptions present.
//Thanks to the hardware being in my mind when coding this, the illegal instructions could be added fairly easily...
byte CPU (CsidPlayer* self) { //the CPU emulation for SID/PRG playback (ToDo: CIA/VIC-IRQ/NMI/RESET vectors, BCD-mode)
    //'IR' is the instruction-register, naming after the hardware-equivalent
    //'cycle': ensure smallest 6510 runtime (for implied/register instructions)
    self->IR=self->memory[self->PC];
    self->cycles=2;
    self->storadd=0;
    
    //nybble2:  1/5/9/D:accu.instructions, 3/7/B/F:illegal opcodes
    if(self->IR&1) {
        switch (self->IR&0x1F) { //addressing modes (begin with more complex cases), PC wraparound not handled inside to save codespace
            case 0x01:
            case 0x03: //(zp,x)
                self->addr  = self->memory[self->memory[++self->PC]+self->X];
                self->addr += self->memory[self->memory[self->PC]+self->X+1]*256;
                self->cycles=6;
                break;
            case 0x11:
            case 0x13: //(zp),y (5..6 cycles, 8 for R-M-W)
                self->addr  = self->memory[self->memory[++self->PC]];
                self->addr += self->memory[self->memory[self->PC]+1]*256 + self->Y;
                self->cycles=6;
                break;
            case 0x19:
            case 0x1B: //abs,y //(4..5 cycles, 7 cycles for R-M-W)
                self->addr  = self->memory[++self->PC];
                self->addr += self->memory[++self->PC]*256 + self->Y;
                self->cycles=5;
                break;
            case 0x1D: //abs,x //(4..5 cycles, 7 cycles for R-M-W)
                self->addr  = self->memory[++self->PC];
                self->addr += self->memory[++self->PC]*256 + self->X;
                self->cycles=5;
                break;
            case 0x0D:
            case 0x0F: //abs
                self->addr  = self->memory[++self->PC];
                self->addr += self->memory[++self->PC]*256;
                self->cycles=4;
                break;
            case 0x15: //zp,x
                self->addr = self->memory[++self->PC] + self->X;
                self->cycles=4;
                break; 
            case 0x05:
            case 0x07: //zp
                self->addr = self->memory[++self->PC];
                self->cycles=3;
                break; 
            case 0x17: 
                if ((self->IR&0xC0)!=0x80) { //zp,x for illegal opcodes
                    self->addr = self->memory[++self->PC] + self->X;
                    self->cycles=4;
                } else { //zp,y for LAX/SAX illegal opcodes
                    self->addr = self->memory[++self->PC] + self->Y;
                    self->cycles=4;
                }
                break;
            case 0x1F:
                if ((self->IR&0xC0)!=0x80) { //abs,x for illegal opcodes
                    self->addr  = self->memory[++self->PC];
                    self->addr += self->memory[++self->PC]*256 + self->X;
                    self->cycles=5;
                } else { //abs,y for LAX/SAX illegal opcodes
                    self->addr  = self->memory[++self->PC];
                    self->addr += self->memory[++self->PC]*256 + self->Y;
                    self->cycles=5;
                }
                break;
            case 0x09:
            case 0x0B: //immediate
                self->addr = ++self->PC;
                self->cycles=2;
        }
        
        self->addr&=0xFFFF;
        switch (self->IR&0xE0) {
            case 0x60: 
               if ((self->IR&0x1F)!=0xB) { //ADC / RRA (ROR+ADC)
                    if((self->IR&3)==3) {
                        self->T = (self->memory[self->addr]>>1)+(self->ST&1)*128;
                        self->ST&=124;
                        self->ST|=(self->T&1);
                        self->memory[self->addr]=self->T;
                        self->cycles+=2;
                    }
                    self->T=self->A;
                    self->A+=self->memory[self->addr]+(self->ST&1);
                    self->ST&=60;
                    self->ST|=(self->A&128)|(self->A>255);
                    self->A&=0xFF;
                    self->ST |= (!self->A)<<1 | ( !((self->T^self->memory[self->addr])&0x80) & ((self->T^self->A)&0x80) ) >> 1;
                } else { //V-flag set by intermediate ADC mechanism: (self->A&mem)+mem
                    self->A&=self->memory[self->addr];
                    self->T+=self->memory[self->addr]+(self->ST&1);
                    self->ST&=60;
                    self->ST |= (self->T>255) | ( !((self->A^self->memory[self->addr])&0x80) & ((self->T^self->A)&0x80) ) >> 1;
                    self->T=self->A;
                    self->A=(self->A>>1)+(self->ST&1)*128;
                    self->ST|=(self->A&128)|(self->T>127);
                    self->ST|=(!self->A)<<1;
                }
                // ARR (AND+ROR, bit0 not going to C, but C and bit7 get exchanged.)
                break;
            case 0xE0: //SBC / ISC(ISB)=INC+SBC
                if((self->IR&3)==3 && (self->IR&0x1F)!=0xB) {
                    self->memory[self->addr]++;
                    self->cycles+=2;
                }
                self->T=self->A;
                self->A-=self->memory[self->addr]+!(self->ST&1);
                self->ST&=60;
                self->ST|=(self->A&128)|(self->A>=0);
                self->A&=0xFF;
                self->ST |= (!self->A)<<1 | ( ((self->T^self->memory[self->addr])&0x80) & ((self->T^self->A)&0x80) ) >> 1;
                break; 
            case 0xC0:
                if((self->IR&0x1F)!=0xB) { // CMP / DCP(DEC+CMP)
                    if ((self->IR&3)==3) {
                        self->memory[self->addr]--;
                        self->cycles+=2;
                    } 
                    self->T=self->A-self->memory[self->addr];
                } else { //SBX (AXS) (CMP+DEX at the same time)
                    self->X=self->T=(self->A&self->X)-self->memory[self->addr];
                }
                self->ST&=124;
                self->ST|=(!(self->T&0xFF))<<1|(self->T&128)|(self->T>=0);
                break;
            case 0x00:
                if ((self->IR&0x1F)!=0xB) { //ORA / SLO(ASO)=ASL+ORA
                    if ((self->IR&3)==3) {
                        self->ST&=124;
                        self->ST|=(self->memory[self->addr]>127);
                        self->memory[self->addr]<<=1;
                        self->cycles+=2;
                    }  
                    self->A|=self->memory[self->addr];
                    self->ST&=125;
                    self->ST|=(!self->A)<<1|(self->A&128);
                } else { //ANC (AND+Carry=bit7)
                    self->A&=self->memory[self->addr];
                    self->ST&=124;
                    self->ST|=(!self->A)<<1|(self->A&128)|(self->A>127);
                }
                break;
            case 0x20:
                if ((self->IR&0x1F)!=0xB) { //AND / RLA (ROL+AND)
                    if ((self->IR&3)==3) {
                        self->T=(self->memory[self->addr]<<1)+(self->ST&1);
                        self->ST&=124;
                        self->ST|=(self->T>255);
                        self->T&=0xFF;
                        self->memory[self->addr]=self->T;
                        self->cycles+=2;
                    }  
                    self->A&=self->memory[self->addr];
                    self->ST&=125;
                    self->ST|=(!self->A)<<1|(self->A&128); 
                } else { //ANC (AND+Carry=bit7)
                    self->A&=self->memory[self->addr];
                    self->ST&=124;
                    self->ST|=(!self->A)<<1|(self->A&128)|(self->A>127);
                }
                break;
            case 0x40:
                if ((self->IR&0x1F)!=0xB) { //EOR / SRE(LSE)=LSR+EOR
                    if ((self->IR&3)==3) {
                        self->ST&=124;
                        self->ST|=(self->memory[self->addr]&1);
                        self->memory[self->addr]>>=1;
                        self->cycles+=2;
                    }
                    self->A^=self->memory[self->addr];
                    self->ST&=125;
                    self->ST|=(!self->A)<<1|(self->A&128);
                } else { //ALR(ASR)=(AND+LSR)
                    self->A&=self->memory[self->addr];
                    self->ST&=124;
                    self->ST|=(self->A&1);
                    self->A>>=1;
                    self->A&=0xFF;
                    self->ST|=(self->A&128)|((!self->A)<<1);
                }
                break;
            case 0xA0:
                if ((self->IR&0x1F)!=0x1B) { //LDA / LAX (illegal, used by my 1 rasterline player)
                        self->A=self->memory[self->addr];
                        if ((self->IR&3)==3) self->X=self->A;
                    } else { // LAS (LAR)
                        self->A=self->X=self->SP=self->memory[self->addr]&self->SP;
                    }
                    self->ST&=125;
                    self->ST|=((!self->A)<<1) | (self->A&128);
                    break;
            case 0x80:
                if ((self->IR&0x1F)==0xB) { //XAA (TXA+AND), highly unstable on real 6502!
                    self->A = self->X & self->memory[self->addr];
                    self->ST&=125;
                    self->ST|=(self->A&128) | ((!self->A)<<1);
                } else if ((self->IR&0x1F)==0x1B) { //TAS(SHS) (SP=A&self->X, mem=S&H} - unstable on real 6502
                    self->SP=self->A&self->X;
                    self->memory[self->addr]=self->SP&((self->addr>>8)+1);
                } else { //STA / SAX (at times same as AHX/SHX/SHY) (illegal) 
                    self->memory[self->addr]=self->A & (((self->IR&3)==3)?self->X:0xFF);
                    self->storadd=self->addr;
                }
                break;
        }
    //nybble2:  2:illegal/LDX, 6:A/self->X/INC/DEC, A:Accu-shift/reg.transfer/NOP, E:shift/self->X/INC/DEC
    } else if(self->IR&2) {
        switch (self->IR&0x1F) { //addressing modes
            case 0x1E: //abs,x / abs,y
                self->addr  = self->memory[++self->PC];
                self->addr += self->memory[++self->PC]*256 + ( ((self->IR&0xC0)!=0x80) ? self->X:self->Y );
                self->cycles=5;
                break;
            case 0x0E: //abs
                self->addr  = self->memory[++self->PC];
                self->addr += self->memory[++self->PC]*256;
                self->cycles=4;
                break;
            case 0x16: //zp,x / zp,y
                self->addr = self->memory[++self->PC] + ( ((self->IR&0xC0)!=0x80) ? self->X:self->Y );
                self->cycles=4;
                break;
            case 0x06: //zp
                self->addr = self->memory[++self->PC];
                self->cycles=3;
                break;
            case 0x02: //imm.
                self->addr = ++self->PC;
                self->cycles=2;
        }
        
        self->addr&=0xFFFF; 
        switch (self->IR&0xE0) {
            case 0x00:
                self->ST&=0xFE;
            case 0x20:
                if((self->IR&0xF)==0xA) { //ASL/ROL (Accu)
                    self->A=(self->A<<1)+(self->ST&1);
                    self->ST&=124;
                    self->ST|=(self->A&128)|(self->A>255);
                    self->A&=0xFF;
                    self->ST|=(!self->A)<<1;
                } else { //RMW (Read-Write-Modify)
                    self->T=(self->memory[self->addr]<<1)+(self->ST&1);
                    self->ST&=124;
                    self->ST|=(self->T&128)|(self->T>255);
                    self->T&=0xFF;
                    self->ST|=(!self->T)<<1;
                    self->memory[self->addr]=self->T;
                    self->cycles+=2;
                }
                break;
            case 0x40:
                self->ST&=0xFE;
            case 0x60:
                if((self->IR&0xF)==0xA) { //LSR/ROR (Accu)
                    self->T=self->A;
                    self->A=(self->A>>1)+(self->ST&1)*128;
                    self->ST&=124;
                    self->ST|=(self->A&128)|(self->T&1);
                    self->A&=0xFF;
                    self->ST|=(!self->A)<<1;
                } else { //self->memory (RMW)
                    self->T=(self->memory[self->addr]>>1)+(self->ST&1)*128;
                    self->ST&=124;
                    self->ST|=(self->T&128)|(self->memory[self->addr]&1);
                    self->T&=0xFF;
                    self->ST|=(!self->T)<<1;
                    self->memory[self->addr]=self->T;
                    self->cycles+=2;
                }
                break;
            case 0xC0:
                if(self->IR&4) { //DEC
                    self->memory[self->addr]--;
                    self->ST&=125;
                    self->ST|=(!self->memory[self->addr])<<1|(self->memory[self->addr]&128);
                    self->cycles+=2;
                } else { //DEX
                    self->X--;
                    self->X&=0xFF;
                    self->ST&=125;
                    self->ST|=(!self->X)<<1|(self->X&128);
                }
                break;
            case 0xA0: //LDX/TSX/TAX
                if((self->IR&0xF)!=0xA) {
                    self->X=self->memory[self->addr];
                } else if(self->IR&0x10) {
                    self->X=self->SP;
                    break;
                } else {
                    self->X=self->A;
                }
                self->ST&=125;
                self->ST|=(!self->X)<<1|(self->X&128);
                break;
            case 0x80: //STX/TXS/TXA
                if(self->IR&4) {
                    self->memory[self->addr]=self->X;
                    self->storadd=self->addr;
                } else if(self->IR&0x10) {
                    self->SP=self->X;
                } else {
                    self->A=self->X;
                    self->ST&=125;
                    self->ST|=(!self->A)<<1|(self->A&128);
                } 
                break;
            case 0xE0: //INC/NOP
                if(self->IR&4) {
                    self->memory[self->addr]++;
                    self->ST&=125;
                    self->ST|=(!self->memory[self->addr])<<1|(self->memory[self->addr]&128);
                    self->cycles+=2;
                }
        }
    //nybble2:  8:register/status
    } else if((self->IR&0xC)==8) {
        switch (self->IR&0xF0) {
            case 0x60: //PLA
                self->SP++;
                self->SP&=0xFF;
                self->A=self->memory[0x100+self->SP];
                self->ST&=125;
                self->ST|=(!self->A)<<1|(self->A&128);
                self->cycles=4;
                break;
            case 0xC0: //INY
                self->Y++;
                self->Y&=0xFF;
                self->ST&=125;
                self->ST|=(!self->Y)<<1|(self->Y&128);
                break;
            case 0xE0: //INX
                self->X++;
                self->X&=0xFF;
                self->ST&=125;
                self->ST|=(!self->X)<<1|(self->X&128);
                break;
            case 0x80: //DEY
                self->Y--;
                self->Y&=0xFF;
                self->ST&=125;
                self->ST|=(!self->Y)<<1|(self->Y&128);
                break;
            case 0x00: //PHP
                self->memory[0x100+self->SP]=self->ST;
                self->SP--;
                self->SP&=0xFF;
                self->cycles=3;
                break;
            case 0x20: //PLP
                self->SP++;
                self->SP&=0xFF;
                self->ST=self->memory[0x100+self->SP];
                self->cycles=4;
                break;
            case 0x40:  //PHA
                self->memory[0x100+self->SP]=self->A;
                self->SP--;
                self->SP&=0xFF;
                self->cycles=3;
                break;
            case 0x90:  //TYA
                self->A=self->Y;
                self->ST&=125;
                self->ST|=(!self->A)<<1|(self->A&128);
                break;
            case 0xA0: //TAY
                self->Y=self->A;
                self->ST&=125;
                self->ST|=(!self->Y)<<1|(self->Y&128);
                break;
            default: //CLC/SEC/CLI/SEI/CLV/CLD/SED
                if(flagsw[self->IR>>5]&0x20) {
                    self->ST|=(flagsw[self->IR>>5]&0xDF);
                } else {
                    self->ST&=255-(flagsw[self->IR>>5]&0xDF);
                }
        }
    //nybble2:  0: control/branch/Y/compare  4: Y/compare  C:Y/compare/JMP
    } else {
        if ((self->IR&0x1F)==0x10) { //BPL/BMI/BVC/BVS/BCC/BCS/BNE/BEQ  relative branch 
            self->PC++; self->T=self->memory[self->PC];
            if(self->T&0x80) self->T-=0x100;
            if(self->IR&0x20) {
                if (self->ST&branchflag[self->IR>>6]) {
                    self->PC+=self->T;
                    self->cycles=3;
                }
            } else {
                if (!(self->ST&branchflag[self->IR>>6])) {
                    self->PC+=self->T;
                    self->cycles=3;
                }
            }
        } else {  //nybble2:  0:Y/control/Y/compare  4:Y/compare  C:Y/compare/JMP
            switch (self->IR&0x1F) { //addressing modes
                case 0x00: //imm. (or abs.low for JSR/BRK)
                    self->addr = ++self->PC;
                    self->cycles=2;
                    break;
                case 0x1C: //abs,x
                    self->addr  = self->memory[++self->PC];
                    self->addr += self->memory[++self->PC]*256 + self->X;
                    self->cycles=5;
                    break;
                case 0x0C: //abs
                    self->addr  = self->memory[++self->PC];
                    self->addr += self->memory[++self->PC]*256;
                    self->cycles=4;
                    break;
                case 0x14: //zp,x
                    self->addr = self->memory[++self->PC] + self->X;
                    self->cycles=4;
                    break;
                case 0x04: //zp
                    self->addr = self->memory[++self->PC];
                    self->cycles=3;
            }
            
            self->addr&=0xFFFF;  
            switch (self->IR&0xE0) {
            case 0x00: //BRK
                self->memory[0x100+self->SP]=self->PC%256;
                self->SP--;
                self->SP&=0xFF;
                self->memory[0x100+self->SP]=self->PC/256;
                self->SP--;
                self->SP&=0xFF;
                self->memory[0x100+self->SP]=self->ST;
                self->SP--;self->SP&=0xFF; 
                self->PC = self->memory[0xFFFE]+self->memory[0xFFFF]*256-1;
                self->cycles=7;
                break;
            case 0x20:
                if(self->IR&0xF) { //BIT
                    self->ST &= 0x3D;
                    self->ST |= (self->memory[self->addr]&0xC0) | ( !(self->A&self->memory[self->addr]) )<<1;
                } else { //JSR
                    self->memory[0x100+self->SP]=(self->PC+2)%256;
                    self->SP--;
                    self->SP&=0xFF;
                    self->memory[0x100+self->SP]=(self->PC+2)/256;
                    self->SP--;self->SP&=0xFF;
                    self->PC=self->memory[self->addr]+self->memory[self->addr+1]*256-1;
                    self->cycles=6;
                }
                break;
            case 0x40:
                if(self->IR&0xF) { //JMP
                    self->PC = self->addr-1;
                    self->cycles=3;
                } else { //RTI
                    if(self->SP>=0xFF) return 0xFE;
                    self->SP++;
                    self->SP&=0xFF;
                    self->ST=self->memory[0x100+self->SP];
                    self->SP++;
                    self->SP&=0xFF;
                    self->T=self->memory[0x100+self->SP];
                    self->SP++;
                    self->SP&=0xFF;
                    self->PC=self->memory[0x100+self->SP]+self->T*256-1;
                    self->cycles=6;
                }
                break;
            case 0x60:
                if(self->IR&0xF) { //JMP() (indirect)
                    self->PC = self->memory[self->addr]+self->memory[self->addr+1]*256-1;
                    self->cycles=5;
                } else { //RTS
                    if(self->SP>=0xFF) return 0xFF;
                    self->SP++;
                    self->SP&=0xFF;
                    self->T=self->memory[0x100+self->SP];
                    self->SP++;
                    self->SP&=0xFF;
                    self->PC=self->memory[0x100+self->SP]+self->T*256-1;
                    self->cycles=6;
                }
                break;
            case 0xC0: //CPY
                self->T=self->Y-self->memory[self->addr];
                self->ST&=124;
                self->ST|=(!(self->T&0xFF))<<1|(self->T&128)|(self->T>=0);
                break;
            case 0xE0: //CPX
                self->T=self->X-self->memory[self->addr];
                self->ST&=124;
                self->ST|=(!(self->T&0xFF))<<1|(self->T&128)|(self->T>=0);
                break;
            case 0xA0: //LDY
                self->Y=self->memory[self->addr];
                self->ST&=125;
                self->ST|=(!self->Y)<<1|(self->Y&128);
                break;
            case 0x80: //STY
                self->memory[self->addr]=self->Y;
                self->storadd=self->addr;
            }
        }
    }
    
    //if(IR==0xCB) //test SBX
    // printf("PC:%4.4X IR:%2.2X, addr: %4.4X,%2.2X,  storadd: %4.4X,%2.2X,  A:%2.2X, X:%2.2X, Y:%2.2X ST:%2.2X\n",PC,IR,addr,memory[addr],storadd,memory[storadd],A,X,Y,ST);  
    
    self->PC++; //PC&=0xFFFF; 
    return 0; 
}

//----------------------------- SID emulation -----------------------------------------

//registers: 0:freql1  1:freqh1  2:pwml1  3:pwmh1  4:ctrl1  5:ad1   6:sr1  7:freql2  8:freqh2  9:pwml2 10:pwmh2 11:ctrl2 12:ad2  13:sr 14:freql3 15:freqh3 16:pwml3 17:pwmh3 18:ctrl3 19:ad3  20:sr3 
//           21:cutoffl 22:cutoffh 23:flsw_reso 24:vol_ftype 25:potX 26:potY 27:OSC3 28:ENV3
void initSID(CsidPlayer* self) { 
    int i;
    for(i=0xD400;i<=0xD7FF;i++) self->memory[i]=0;
    for(i=0xDE00;i<=0xDFFF;i++) self->memory[i]=0;
    for(i=0;i<9;i++) {
        self->ADSRstate[i]=HOLDZERO_BITMASK;
        self->ratecnt[i] = 0;
        self->envcnt[i] = 0;
        self->expcnt[i] = 0;
    } 
}

void cSID_init(CsidPlayer* self, int samplerate) {
    int i;
    
    cap_6581_reciprocal = -1000000/CAP_6581;
    cutoff_steepness_6581 = FILTER_DARKNESS_6581*(2048.0-VCR_FET_TRESHOLD);
    #ifdef LIBCSID_FULL
     self->clock_ratio = round(CLOCK_CPU_PAL/samplerate);
     self->cutoff_ratio_8580 = -2 * 3.14 * (12500.0 / 2048) / CLOCK_CPU_PAL;
     self->cutoff_ratio_6581 = -2 * 3.14 * (20000.0 / 2048) / CLOCK_CPU_PAL;
     self->cutoff_bias_6581 = 1 - exp( -2 * 3.14 * 220 / CLOCK_CPU_PAL ); //around 220Hz below treshold
     
     createCombinedWF(TriSaw_8580, 0.5, 2.2, 0.9);
     createCombinedWF(PulseSaw_8580, 0.23, 1.27, 0.55);
     createCombinedWF(PulseTriSaw_8580, 0.5, 1.6, 0.8);
    #else
     self->clock_ratio = CLOCK_CPU_PAL/samplerate;
     if (self->clock_ratio>9) {
         self->period0 = self->clock_ratio;
         self->step0 = ceil(self->clock_ratio/9.0);
     } else {
         self->period0 = 9.0;
         self->step0 = 1;
     }
     self->cutoff_ratio_8580 = -2 * 3.14 * (12500 / 2048) / samplerate;
     self->cutoff_ratio_6581 = -2 * 3.14 * (20000.0 / 2048) / samplerate;
     self->cutoff_bias_6581  = 1 - exp( -2 * 3.14 * 220 / samplerate ); //around 220Hz below treshold
     
     createCombinedWF(TriSaw_8580, 0.8, 2.4, 0.64);
     createCombinedWF(PulseSaw_8580, 1.4, 1.9, 0.68);
     createCombinedWF(PulseTriSaw_8580, 0.8, 2.5, 0.64);
    #endif
    
    for(i = 0; i < 9; i++) {
        self->ADSRstate[i] = HOLDZERO_BITMASK;
        self->envcnt[i] = 0;
        self->ratecnt[i] = 0; 
        self->phaseaccu[i] = 0;
        self->prevaccu[i] = 0;
        self->expcnt[i] = 0; 
        self->noise_LFSR[i] = 0x7FFFFF;
        self->prevwfout[i] = 0;
        self->prevSR[i]=0;
    }
    for(i = 0; i < 3; i++) {
        self->sourceMSBrise[i] = 0;
        self->sourceMSB[i] = 0;
        self->prevlowpass[i] = 0;
        self->prevbandpass[i] = 0;
        self->cutoff[i] = 0;
        self->resonance[i] = 0;
    }
    initSID(self);
}

//My SID implementation is similar to what I worked out in a SwinSID variant during 3..4 months of development. (So jsSID only took 2 weeks armed with this experience.)
//I learned the workings of ADSR/WAVE/filter operations mainly from the quite well documented resid and resid-fp codes.
//(The SID reverse-engineering sites were also good sources.)
//Note that I avoided many internal/automatic variables from the SID function, assuming better speed this way. (Not using stack as much, but I'm not sure and it may depend on platform...)
//(The same is true for CPU emulation and player-code.)
//
//Any mention of anti-aliasing applies only to csid-light variant.
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


int SID(CsidPlayer* self, char sidNum, unsigned int baseaddr) {
    //better keep these variables static so they won't slow down the routine like if they were internal automatic variables always recreated
    //or move them into "self"
    byte channel, ctrl, SR, prevgate, wf, test; 
    byte *sReg, *vReg;
    unsigned int accuadd, pw, wfout;
    unsigned long int MSB;
    int step;
    long int output, nonfilt, filtin, filtout;
    #ifdef LIBCSID_FULL
     long int ftmp;
     #define INTERNAL_RATE CLOCK_CPU_PAL
     #define INTERNAL_RATIO 1
     unsigned int period;
    #else
     #define INTERNAL_RATE self->samplerate
     #define INTERNAL_RATIO self->clock_ratio
     float period;
     float ftmp;
    #endif

    filtin=nonfilt=0;
    sReg = &self->memory[baseaddr];
    vReg = sReg;

    //treating 2SID and 3SID channels uniformly (0..5 / 0..8), this probably avoids some extra code
    for (channel = sidNum * SID_CHANNEL_AMOUNT ; channel < (sidNum + 1) * SID_CHANNEL_AMOUNT ; channel++, vReg += 7) {
        int trig = 0;

        //ADSR envelope generator:
        ctrl = vReg[4];
        SR = vReg[6];
        prevgate = (self->ADSRstate[channel] & GATE_BITMASK);
        if (prevgate != (ctrl & GATE_BITMASK)) { //gatebit-change?
            if (prevgate) {
                self->ADSRstate[channel] &= 0xFF - (GATE_BITMASK | ATTACK_BITMASK | DECAYSUSTAIN_BITMASK);
            } else { //falling edge
                self->ADSRstate[channel] = (GATE_BITMASK | ATTACK_BITMASK | DECAYSUSTAIN_BITMASK); //rising edge, also sets hold_zero_bit=0
                #ifndef LIBCSID_FULL
                 //assume SR->GATE write order: workaround to have crisp soundstarts by triggering delay-bug
                 //(this is for the possible missed CTRL(GATE) vs SR register write order situations (1MHz CPU is cca 20 times faster than samplerate)
                 if ((SR & 0xF) > (self->prevSR[channel] & 0xF)) trig = 1;
                #endif
            }
        }

        //set ADSR period that should be checked against rate-counter (depending on ADSR state Attack/DecaySustain/Release)
        if (self->ADSRstate[channel] & ATTACK_BITMASK) {
            step = vReg[5] >> 4;
        } else if (self->ADSRstate[channel] & DECAYSUSTAIN_BITMASK) {
            step = vReg[5] & 0xF;
        } else {
            step = SR & 0xF;
        }

        #ifdef LIBCSID_FULL
         period = ADSRperiods[step];
         step = 1;
        #else
         if (step == 0) {
             period = self->period0;
             step = self->step0;
         } else {
             period = ADSRperiods[step];
             step = 1;
         }
         self->prevSR[channel] = SR; //if(SR&0xF) ratecnt[channel]+=5;  //assume SR->GATE write order: workaround to have crisp soundstarts by triggering delay-bug
        #endif

        self->ratecnt[channel] += INTERNAL_RATIO;
        //self->ratecnt[channel] &= 0x7FFF;
        if (self->ratecnt[channel] >= 0x8000) self->ratecnt[channel] -= 0x8000; //can wrap around (ADSR delay-bug: short 1st frame)
        if (self->ratecnt[channel] >= period && self->ratecnt[channel] < period + INTERNAL_RATIO && trig == 0) { //ratecounter shot (matches rateperiod) (in genuine SID ratecounter is LFSR)
            self->ratecnt[channel] -= period; //compensation for timing instead of simply setting 0 on rate-counter overload
            if ((self->ADSRstate[channel] & ATTACK_BITMASK) || ++self->expcnt[channel] == ADSR_exptable[self->envcnt[channel]]) {
                self->expcnt[channel] = 0; 
                if (!(self->ADSRstate[channel] & HOLDZERO_BITMASK)) {
                    if (self->ADSRstate[channel] & ATTACK_BITMASK) {
                        self->envcnt[channel] += step;
                        if (self->envcnt[channel] >= 0xFF) {
                            self->envcnt[channel] = 0xFF;
                            self->ADSRstate[channel] &= 0xFF - ATTACK_BITMASK;
                        }
                    } else if ( !(self->ADSRstate[channel] & DECAYSUSTAIN_BITMASK) || self->envcnt[channel] > (SR & 0xF0) + (SR >> 4)) {
                        self->envcnt[channel] -= step; //resid adds 1 cycle delay, we omit that pipelining mechanism here
                        if (self->envcnt[channel] <= 0 && (self->envcnt[channel] + step) != 0) {
                            self->envcnt[channel] = 0;
                            self->ADSRstate[channel] |= HOLDZERO_BITMASK;
                        }
                    }
                //TODO: find out why envelopes fail if not wrapped around byte size
                self->envcnt[channel] &=0xFF;
                }
            }
        }


        //WAVE generation code (phase accumulator and waveform-selector):
        test = ctrl & TEST_BITMASK;
        wf = ctrl & 0xF0;
        accuadd = (vReg[0] + vReg[1] * 256) * INTERNAL_RATIO;
        if (test || ((ctrl & SYNC_BITMASK) && self->sourceMSBrise[sidNum])) {
            self->phaseaccu[channel] = 0;
        } else {
            self->phaseaccu[channel] += accuadd;
            if (self->phaseaccu[channel] > 0xFFFFFF) self->phaseaccu[channel] -= 0x1000000;
            self->phaseaccu[channel] &= 0xFFFFFF;
        }
        MSB = self->phaseaccu[channel] & 0x800000;
        self->sourceMSBrise[sidNum] = (MSB > (self->prevaccu[channel] & 0x800000)) ? 1 : 0;
        if (wf & NOISE_BITMASK) { //noise waveform
            int tmp = self->noise_LFSR[channel];

            //WARN: csid-full does not check for "|| accuadd >= 0x100000"
            if (((self->phaseaccu[channel] & 0x100000) != (self->prevaccu[channel] & 0x100000)) || accuadd >= 0x100000) {
                //clock LFSR all time if clockrate exceeds observable at given samplerate
                tmp = ((tmp << 1) + (((tmp & 0x400000) ^ ((tmp & 0x20000) << 5)) ? 1 : test)) & 0x7FFFFF;
                self->noise_LFSR[channel] = tmp;
            }
            //we simply zero output when other waveform is mixed with noise. On real SID LFSR continuously gets filled by zero and locks up. ($C1 waveform with pw<8 can keep it for a while...)
            wfout = (wf & 0x70) ? 0 : ((tmp & 0x100000) >> 5) + ((tmp & 0x40000) >> 4) + ((tmp & 0x4000) >> 1) + ((tmp & 0x800) << 1) + ((tmp & 0x200) << 2) + ((tmp & 0x20) << 5) + ((tmp & 0x04) << 7) + ((tmp & 0x01) << 8);
        } else if (wf & PULSE_BITMASK) { //simple pulse
            pw = (vReg[2] + (vReg[3] & 0xF) * 256) * 16;
            
            #ifdef LIBCSID_FULL
             int tmp;
            #else
             int tmp = (int) accuadd >> 9;  
             if (0 < pw && pw < tmp) pw = tmp;
             tmp ^= 0xFFFF;
             if (pw > tmp) pw = tmp;
            #endif

            tmp = self->phaseaccu[channel] >> 8;
            if (wf == PULSE_BITMASK) { //simple pulse
                #ifdef LIBCSID_FULL
                 if (test || tmp>=pw) { //rising edge
                     wfout = 0xFFFF;
                 } else { //falling edge
                     wfout=0;
                 }
                #else //bandlimited
                 step = (accuadd>=255)? 65535/(accuadd/256.0) : 0xFFFF;
                 if (test) {
                     wfout=0xFFFF;
                 } else if (tmp<pw) { //rising edge
                     int lim = (0xFFFF - pw) * step;
                     if (lim > 0xFFFF) lim = 0xFFFF;
                     tmp = lim - (pw - tmp) * step;
                     wfout = (tmp < 0) ? 0:tmp;
                 } else { //falling edge
                     int lim = pw*step;
                     if (lim > 0xFFFF) lim = 0xFFFF;
                     tmp = (0xFFFF - tmp) * step - lim;
                     wfout = (tmp >= 0) ? 0xFFFF : tmp;
                 }
                #endif
            } else { //combined pulse
                wfout = (tmp >= pw || test) ? 0xFFFF : 0; //this aliases at high pitch when not oversampled
                if (wf & TRI_BITMASK) {
                    if (wf & SAW_BITMASK) { //pulse+saw+triangle (waveform nearly identical to tri+saw)
                        wfout = wfout ? combinedWF(self, sidNum, channel, PulseTriSaw_8580, tmp >> 4, 1, vReg[1]) : 0;
                    } else { //pulse+triangle
                        tmp = self->phaseaccu[channel] ^ (ctrl & RING_BITMASK ? self->sourceMSB[sidNum] : 0);
                        wfout = wfout ? combinedWF(self, sidNum, channel, PulseSaw_8580, (tmp ^ (tmp & 0x800000 ? 0xFFFFFF : 0)) >> 11, 0, vReg[1]) : 0;
                    }
                } else if (wf & SAW_BITMASK) { //pulse+saw
                    wfout = wfout ? combinedWF(self, sidNum, channel, PulseSaw_8580, tmp >> 4, 1, vReg[1]) : 0;
                }
            }
        } else if (wf & SAW_BITMASK) { //saw
            wfout = self->phaseaccu[channel] >> 8; //this aliases at high pitch when not oversampled
            if (wf & TRI_BITMASK) { //saw+triangle
                wfout = combinedWF(self, sidNum, channel, TriSaw_8580, wfout >> 4, 1, vReg[1]);
            } else { //bandlimited saw
                #ifndef LIBCSID_FULL
                 float steep = (accuadd/65536.0)/288.0;
                 wfout += wfout*steep;
                 if(wfout>0xFFFF) wfout=0xFFFF-(wfout-0x10000)/steep; 
                #endif
            }
        } else if (wf & TRI_BITMASK) { //triangle, doesn't need antialias so much
            int tmp = self->phaseaccu[channel] ^ (ctrl & RING_BITMASK ? self->sourceMSB[sidNum] : 0);
            wfout = (tmp ^ (tmp & 0x800000 ? 0xFFFFFF : 0)) >> 7;
        }
        wfout&=0xFFFF;

        if (wf) { //emulate waveform 00 floating wave-DAC (on real SID waveform00 decays after 15s..50s depending on temperature?)
            self->prevwfout[channel] = wfout;
        } else {
            wfout = self->prevwfout[channel];
        }
        self->prevaccu[channel] = self->phaseaccu[channel];
        self->sourceMSB[sidNum] = MSB; //(So the decay is not an exact value. Anyway, we just simply keep the value to avoid clicks and support SounDemon digi later...)

        //routing the channel signal to either the filter or the unfiltered master output depending on filter-switch SID-registers
        if (sReg[0x17] & FILTSW[channel]) {
            filtin += ((long int)wfout - 0x8000) * self->envcnt[channel] / 256;
        } else if ((FILTSW[channel] != 4) || !(sReg[0x18] & OFF3_BITMASK)) {
            nonfilt += ((long int)wfout - 0x8000) * self->envcnt[channel] / 256;
        }
    }
    //update readable SID1-registers (some SID tunes might use 3rd channel ENV3/OSC3 value as control)
    if(sidNum==0 && self->memory[1]&3) { //OSC3, ENV3 (some players rely on it) 
        sReg[0x1B]=wfout>>8;
        sReg[0x1C]=self->envcnt[3];
    }
    
    //FILTER:
    #define SCALE_CUTOFF 0x10000
    #define SCALE_RESO   0x100
    #ifdef LIBCSID_FULL
     //calculate cutoff and resonance curves only at samplerate is still adequate and reduces CPU stress of frequent float calculations
     self->filterctrl_prescaler[sidNum]--;
     if (self->filterctrl_prescaler[sidNum]<=0) {
         self->filterctrl_prescaler[sidNum]=self->clock_ratio;
    #endif
        self->cutoff[sidNum] = 2 + sReg[0x16] * 8 + (sReg[0x15] & 7); //WARN: csid-light does not "2 +", why?
        if (self->SID_model[sidNum] == SIDMODEL_8580) {
            self->resonance[sidNum] = ( pow(2, ((4 - (sReg[0x17] >> 4)) / 8.0)) ) * SCALE_RESO;
            self->cutoff[sidNum] = ( 1 - exp((self->cutoff[sidNum]+2) * self->cutoff_ratio_8580) ) * SCALE_CUTOFF; //linear curve by resistor-ladder VCR
        } else { //6581
            self->resonance[sidNum] = ( (sReg[0x17] > 0x5F) ? 8.0 / (sReg[0x17] >> 4) : 1.41 ) * SCALE_RESO;
            #if 1
             float rDS_VCR_FET;
             self->cutoff[sidNum] += round(filtin*FILTER_DISTORTION_6581); //MOSFET-VCR control-voltage-modulation (resistance-modulation aka 6581 filter distortion) emulation
             //below Vth treshold Vgs control-voltage FET presents an open circuit
             // rDS ~ (-Vth*rDSon) / (Vgs-Vth)  //above Vth FET drain-source resistance is proportional to reciprocal of cutoff-control voltage
             rDS_VCR_FET = self->cutoff[sidNum]<=VCR_FET_TRESHOLD ? 100000000.0 : cutoff_steepness_6581/(self->cutoff[sidNum]-VCR_FET_TRESHOLD);
             self->cutoff[sidNum] = ( 1 - exp( cap_6581_reciprocal / (VCR_SHUNT_6581*rDS_VCR_FET/(VCR_SHUNT_6581+rDS_VCR_FET)) / INTERNAL_RATE ) ) * SCALE_CUTOFF; //curve with 1.5MOhm VCR parallel Rshunt emulation
            #else
             self->cutoff[sidNum] = ( self->cutoff_bias_6581 + ( (self->cutoff[sidNum] < 192) ? 0 : 1 - exp((self->cutoff[sidNum]-192) * self->cutoff_ratio_6581) )  ) * SCALE_CUTOFF;
            #endif
        }
    #ifdef LIBCSID_FULL
     }
     //the filter-calculation itself can't be prescaled because sound-quality would suffer of no 'oversampling'
    #endif
    filtout=0;
    ftmp = filtin + self->prevbandpass[sidNum] * self->resonance[sidNum] / SCALE_RESO + self->prevlowpass[sidNum];
    if (sReg[0x18] & HIGHPASS_BITMASK) filtout -= ftmp;
    ftmp = self->prevbandpass[sidNum] - ftmp * self->cutoff[sidNum] / SCALE_CUTOFF;
    self->prevbandpass[sidNum] = ftmp;
    if (sReg[0x18] & BANDPASS_BITMASK) filtout -= ftmp;
    ftmp = self->prevlowpass[sidNum] + ftmp * self->cutoff[sidNum] / SCALE_CUTOFF;
    self->prevlowpass[sidNum] = ftmp;
    if (sReg[0x18] & LOWPASS_BITMASK) filtout += ftmp;
    
    //output stage for one SID
    output = (nonfilt+filtout) * (sReg[0x18]&0xF) / self->volScale;
    //saturation logic on overload (not needed if the callback handles it)
    if (output>=32767) {
        output=32767;
    } else if (output<=-32768) {
        output=-32768;
    }
    return (int)output; // master output
    #undef SCALE_CUTOFF
    #undef SCALE_RESO
    #undef INTERNAL_RATE
    #undef INTERNAL_RATIO
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

unsigned int combinedWF(CsidPlayer* self, char num, char channel, unsigned int* wfarray, int index, char differ6581, byte freqh) {
    if(differ6581 && self->SID_model[num]==SIDMODEL_6581) index &= 0x7FF;
    #ifdef LIBCSID_FULL
     return wfarray[index];
    #else
     float addf = 0.6+0.4/freqh;
     
     self->prevwavdata[channel] = wfarray[index]*addf + self->prevwavdata[channel]*(1.0-addf);
     return self->prevwavdata[channel];
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

const int libcsid_getsubtunenum(CsidPlayer* self) {
    return self->subtuneAmount;
}

void libcsid_init(CsidPlayer* self, int samplerate) {
    self->samplerate  = samplerate;
    self->sampleratio = round(CLOCK_CPU_PAL / samplerate);
}

int libcsid_load(CsidPlayer* self, unsigned char *_buffer, int _bufferlen, int subtune) {
    int readata, preferred_SID_model[3] = {SIDMODEL_8580, SIDMODEL_8580, SIDMODEL_8580};
    unsigned int i, datalen, offs, loadaddr;
    
    self->curSubtune = subtune;
    
    unsigned char *filedata = _buffer;
    datalen = _bufferlen;
    
    offs = filedata[7];
    loadaddr = filedata[8] + filedata[9] ? filedata[8] * 256 + filedata[9] : filedata[offs] + filedata[offs + 1] * 256;
    printf("\nOffset: $%4.4X, Loadaddress: $%4.4X \nTimermodes:", offs, loadaddr);
    
    for (i = 0; i < 32; i++) {
        self->timermode[31 - i] = (filedata[0x12 + (i >> 3)] & (byte)pow(2, 7 - i % 8)) ? 1 : 0;
        printf(" %1d",self->timermode[31 - i]);
    }
    
    for (i = 0; i < MAX_DATA_LEN; i++) {
        self->memory[i] = 0;
    }
    
    for (i = offs + 2; i < datalen; i++) {
        if (loadaddr + i - (offs + 2) < MAX_DATA_LEN) {
            self->memory[loadaddr + i - (offs + 2)] = filedata[i];
        }
    }
    
    self->initaddr=filedata[0xA]+filedata[0xB]? filedata[0xA]*256+filedata[0xB] : loadaddr;
    self->playaddr=self->playaddf=filedata[0xC]*256+filedata[0xD];
    printf("\nInit:$%4.4X,Play:$%4.4X, ", self->initaddr, self->playaddr);
    self->subtuneAmount=filedata[0xF];
    preferred_SID_model[0] = (filedata[0x77]&0x30)>=0x20? SIDMODEL_8580 : SIDMODEL_6581;
    printf("Subtunes:%d , preferred SID-model:%d\n", self->subtuneAmount, preferred_SID_model[0]);
    preferred_SID_model[1] = (filedata[0x77]&0xC0)>=0x80 ? SIDMODEL_8580 : SIDMODEL_6581;
    preferred_SID_model[2] = (filedata[0x76]&3)>=3 ? SIDMODEL_8580 : SIDMODEL_6581; 
    self->SID_address[0] = 0xD400;
    self->SID_address[1] = filedata[0x7A]>=0x42 && (filedata[0x7A]<0x80 || filedata[0x7A]>=0xE0) ? 0xD000+filedata[0x7A]*16 : 0;
    self->SID_address[2] = filedata[0x7B]>=0x42 && (filedata[0x7B]<0x80 || filedata[0x7B]>=0xE0) ? 0xD000+filedata[0x7B]*16 : 0;
    
    self->SIDamount=1+(self->SID_address[1]>0)+(self->SID_address[2]>0);
    if(self->SIDamount>=2) printf("(SID1), %d(SID2:%4.4X)",preferred_SID_model[1],self->SID_address[1]); 
    if(self->SIDamount==3) printf(", %d(SID3:%4.4X)",preferred_SID_model[2],self->SID_address[2]);
    
    for (i=0;i<self->SIDamount;i++) {
        self->SID_model[i] = preferred_SID_model[i];
    }
    
    self->volScale = (SID_CHANNEL_AMOUNT * 16 + 26);
    if (self->SIDamount == 2) {
      self->volScale /= 0.6;
    } else if (self->SIDamount >= 3) {
      self->volScale /= 0.4;
    }
    
    cSID_init(self, self->samplerate);
    init(self, subtune);
    
    return 0;
}