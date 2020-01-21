#ifndef _CSID_H_
#define _CSID_H_

#define MAX_DATA_LEN 65536

#define SIDMODEL_8580 8580
#define SIDMODEL_6581 6581

#define DEFAULT_SAMPLERATE 44100
#define DEFAULT_SIDMODEL SIDMODEL_6581

typedef struct CsidPlayer CsidPlayer;
typedef unsigned char byte;

extern void libcsid_init(CsidPlayer* self, int samplerate);
extern int libcsid_load(CsidPlayer* self, unsigned char *buffer, int bufferlen, int subtune);
extern const int   libcsid_getsubtunenum(CsidPlayer* self);

extern void libcsid_render(CsidPlayer* self, signed short *stream, int numSamples);

//-----------------------------------------------------------------------------------------------------------
//Various gubbins

#define CLOCK_MASTER_PAL  17734475
#define CLOCK_MASTER_NTSC 14318180
//WARN: required as float for csid-light
#define CLOCK_CPU_PAL     985248.f    // CLOCK_MASTER_PAL  / 18
#define CLOCK_CPU_NTSC    1022727.f   // CLOCK_MASTER_NTSC / 14
#define CLOCK_VICII_PAL   7881984
#define CLOCK_VICII_NTSC  8181816

#define SID_CHANNEL_AMOUNT 3
//selected carefully otherwise some ADSR-sensitive tunes may suffer more.
#define PAL_FRAMERATE 50.06f //49.4 //50.06 //50.0443427 //50.1245419 //(CLOCK_CPU_PAL/63/312.5)

#define VCR_SHUNT_6581 1500 //1500 //kOhm //cca 1.5 MOhm Rshunt across VCR FET drain and source (causing 220Hz bottom cutoff with 470pF integrator capacitors in old C64)
#define VCR_FET_TRESHOLD 350 //192 //Vth (on cutoff numeric range 0..2048) for the VCR cutoff-frequency control FET below which it doesn't conduct
#define CAP_6581 0.470 //0.470 //nF //filter capacitor value for 6581
#define FILTER_DARKNESS_6581 33.0 //22.0 //the bigger the value, the darker the filter control is (that is, cutoff frequency increases less with the same cutoff-value)
#define FILTER_DISTORTION_6581 0.0032 //0.0016 //the bigger the value the more of resistance-modulation (filter distortion) is applied for 6581 cutoff-control
#define CLOCK_RATIO_DEFAULT CLOCK_CPU_PAL/DEFAULT_SAMPLERATE  //(50.0567520: lowest framerate where Sanxion is fine, and highest where Myth is almost fine)

struct CsidPlayer {
    //SID-emulation variables:
    int cutoff[3], resonance[3];
    byte filterctrl_prescaler[3];
    long int prevlowpass[3], prevbandpass[3];
    short int envcnt[9];
    byte ADSRstate[9], expcnt[9], sourceMSBrise[9], prevSR[9];
    unsigned long int prevwfout[9], sourceMSB[3], noise_LFSR[9];
    //the cutoff must be signed otherwise compiler may make errors in multiplications
    //so if samplerate is smaller, cutoff needs to be 'long int' as its value can exceed 32768
    //maybe Hermit wanted this code to work in 16bit DOS environment
    float clock_ratio;
    float cutoff_ratio_8580, cutoff_ratio_6581, cutoff_bias_6581;
    
    //player-related variables:
    int volScale; //compensation for main volume and also filter reso emphasis
    int SIDamount, SID_model[3], sampleratio;
    unsigned int initaddr, playaddr, playaddf, SID_address[3];
    byte memory[MAX_DATA_LEN], timermode[0x20];
    int curSubtune, subtuneAmount;
    long int samplerate;
    float CPUtime;
    
    //CPU (and CIA/VIC-IRQ) emulation constants and variables
    unsigned int PC, pPC, addr, storadd;
    short int A, T, SP; 
    byte X, Y, IR, ST;  //STATUS-flags: N V - B D I Z C
    char cycles, finished, dynCIA;
    
    #ifdef LIBCSID_FULL
     //SID-emulation variables:
     unsigned int ratecnt[9];
     unsigned long int phaseaccu[9], prevaccu[9];
     //player-related variables:
     int framecnt, frame_sampleperiod;
    #else
     //SID-emulation variables:
     unsigned long int prevwavdata[9];
     long int phaseaccu[9], prevaccu[9];
     float ratecnt[9];
     float period0;
     byte  step0;
     //player-related variables:
     float framecnt, frame_sampleperiod;
    #endif
};

#endif