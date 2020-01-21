#define SAMPRATE 44100
#define MIX_BUF_SAMPLES 4096
#define MIX_BUF_NUM 2

#define WIN32_LEAN_AND_MEAN // for stripping windows.h include

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <conio.h>

#include <windows.h> // for mixer stream
#include <mmsystem.h> // for mixer stream

static volatile char isAudioRunning;
static HANDLE hThread;
static HWAVEOUT hWaveOut; // Device handle
static WAVEFORMATEX wfx;
static int16_t *mixBuffer[MIX_BUF_NUM];
static WAVEHDR header[MIX_BUF_NUM];
static int currBuf;
static CRITICAL_SECTION critAudio;
static void (*funCallback)(void* buffer, int numSamples);

static DWORD WINAPI mixThread(LPVOID lpParam) {
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);
    InitializeCriticalSection(&critAudio);
    while(isAudioRunning) {
        while (header[currBuf].dwFlags & WHDR_DONE) {
            EnterCriticalSection(&critAudio);
            funCallback(mixBuffer[currBuf], MIX_BUF_SAMPLES);
            LeaveCriticalSection(&critAudio);
            waveOutPrepareHeader(hWaveOut, &header[currBuf], sizeof(WAVEHDR));
            waveOutWrite(hWaveOut, &header[currBuf], sizeof(WAVEHDR));
            currBuf = (currBuf + 1) % MIX_BUF_NUM;
        }
        SleepEx(1, 1);
    }
    DeleteCriticalSection(&critAudio);
    return (0);
}

void closeMixer(void) {
    int i;
    
    isAudioRunning = FALSE;
    if (hThread) {
        WaitForSingleObject(hThread, INFINITE);
        CloseHandle(hThread);
        hThread = NULL;
    }
    if (hWaveOut) {
        waveOutReset(hWaveOut);
        for (i = 0; i < MIX_BUF_NUM; ++i) {
            if (header[i].dwUser != 0xFFFF) waveOutUnprepareHeader(hWaveOut, &header[i], sizeof (WAVEHDR));
        }
        waveOutClose(hWaveOut);
        for (i = 0; i < MIX_BUF_NUM; ++i) {
            if (mixBuffer[i] != NULL) {
                free(mixBuffer[i]);
                mixBuffer[i] = NULL;
            }
        }
        hWaveOut = NULL;
    }
    currBuf = 0;
}

BOOL openMixer(DWORD outputFreq, int bits, int channels, void* callback) {
    int i;
    DWORD threadID;
    
    funCallback = callback;
    for (i = 0; i < MIX_BUF_NUM; ++i) header[i].dwUser = 0xFFFF;
    closeMixer();

    wfx.nSamplesPerSec = outputFreq;
    wfx.wBitsPerSample = bits;
    wfx.nChannels = channels;

    wfx.cbSize = 0;
    wfx.wFormatTag = WAVE_FORMAT_PCM;
    wfx.nBlockAlign = (wfx.wBitsPerSample * wfx.nChannels) / 8;
    wfx.nAvgBytesPerSec = wfx.nBlockAlign * wfx.nSamplesPerSec;

    if(waveOutOpen(&hWaveOut, WAVE_MAPPER, &wfx, 0, 0, CALLBACK_NULL) != MMSYSERR_NOERROR) {
        return FALSE;
    }

    for (i = 0; i < MIX_BUF_NUM; i++) {
        mixBuffer[i] = (int16_t *)(calloc(MIX_BUF_SAMPLES, wfx.nBlockAlign));
        if (mixBuffer[i] == NULL) goto omError;
        
        memset(&header[i], 0, sizeof(WAVEHDR));
        header[i].dwBufferLength = wfx.nBlockAlign * MIX_BUF_SAMPLES;
        header[i].lpData = (LPSTR) mixBuffer[i];
        waveOutPrepareHeader(hWaveOut, &header[i], sizeof(WAVEHDR));
        waveOutWrite(hWaveOut, &header[i], sizeof(WAVEHDR));
    }
    isAudioRunning = TRUE;
    hThread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)(mixThread), callback, 0, &threadID);
    if (!hThread) return FALSE;
    
    return TRUE;
    
    omError:
        closeMixer();
        return FALSE;
}

void enterCritical() {if (hThread) EnterCriticalSection(&critAudio);}
void leaveCritical() {if (hThread) LeaveCriticalSection(&critAudio);}