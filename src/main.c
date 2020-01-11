#define SAMPRATE 44100

#define WIN32_LEAN_AND_MEAN // for stripping windows.h include

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <conio.h>
#include <windows.h>

#include "libcsid.h"
#include "tinywinmm.h"

int* loadfile(char* name, int* buflen) {
    char * buf;
	FILE *fin = fopen(name, "rb");
	if (NULL == fin) {
        printf( "Could not open the file.\n" );
		return NULL;
	}

    fseek(fin, 0, SEEK_END);
    *buflen = ftell( fin );
    fseek(fin, 0, SEEK_SET);

    buf = malloc(*buflen);
    if(!buf) {
        fclose( fin );
        printf( "Out of memory!\n" );
        return NULL;
    }

    fread(buf, 1, *buflen, fin);
	fclose(fin);

    return buf;
}

int main(int argc, char *argv[]) {
  char* buf; int buflen;
  FILE *in;
  int subtune = 0; int subtune_total;
  int prefmodel = -1;
  
  if (argc < 2) goto ERR;
  
  buf = loadfile(argv[1], &buflen);
  if (!buf) goto ERR;
  if (argc >= 3) subtune = atoi(argv[2]);
  if (argc >= 4) prefmodel = atoi(argv[3]);
  if (prefmodel != SIDMODEL_8580 && prefmodel != SIDMODEL_6581) prefmodel = -1;
  libcsid_init(SAMPRATE, prefmodel);
  libcsid_load(buf, buflen, subtune);
  if (!openMixer(SAMPRATE, 16, 1, &libcsid_render)) goto ERR;
  subtune_total = libcsid_getsubtunenum();
  printf("Press ESC to stop.\n");
  
  while(1) {
    if(kbhit()) {
        unsigned char c = _getch();
        if (c == 0x1B) break;
        if (c == 0xE0) {
            c = _getch();
            int oldsub = subtune;
            if (c == 0x4D) subtune = ++subtune % subtune_total;
            if (c == 0x4B) subtune = --subtune<0 ? subtune_total-1 : subtune;
            if (oldsub != subtune) {
                printf("Playing subtune %i / %i\n", subtune+1, subtune_total);
                enterCritical();
                libcsid_load(buf, buflen, subtune);
                leaveCritical();
            }
        }
    }
    SleepEx(1, 1);
  }
  closeMixer();
  return 0;
  
  ERR:
    closeMixer();
    return 1;
}