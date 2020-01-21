void closeMixer(void);
BOOL openMixer(DWORD outputFreq, int bits, int channels, void *callback);
void enterCritical();
void leaveCritical();