#ifndef _CSID_H_
#define _CSID_H_

#define MAX_DATA_LEN 65536

#define SIDMODEL_8580 8580
#define SIDMODEL_6581 6581

#define DEFAULT_SAMPLERATE 44100
#define DEFAULT_SIDMODEL SIDMODEL_6581

typedef struct CsidPlayer CsidPlayer;
typedef unsigned char byte;

extern void libcsid_init(int samplerate, int sidmodel);
extern int libcsid_load(unsigned char *buffer, int bufferlen, int subtune);
extern const char *libcsid_getauthor();
extern const char *libcsid_getinfo();
extern const char *libcsid_gettitle();
extern const int   libcsid_getsubtunenum();

extern void libcsid_render(signed short *stream, int numSamples);

#endif