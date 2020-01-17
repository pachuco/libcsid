@echo off

@echo off
set gccbase=G:\p_files\rtdk\mingw32-gcc5\bin
set gccbase=G:\p_files\rtdk\i686-8.1.0-win32-dwarf-rt_v6-rev0\mingw32\bin

set PATH=%PATH%;%gccbase%;%fbcbase%
::-Wpedantic
set opts=-std=c99 -mconsole -O3 -s -Wall -Wextra
set link=-lwinmm

del csidlight.exe
del csidfull.exe
pushd src
gcc -o ..\csidlight.exe main.c tinywinmm.c libcsid-light.c %opts% %link% 2> ..\errlight.log
gcc -o ..\csidfull.exe main.c tinywinmm.c libcsid-full.c %opts% %link% 2>   ..\errfull.log
gcc -o ..\csidfullorig.exe main.c tinywinmm.c libcsid-full-orig.c %opts% %link% 2>   ..\errfullorig.log
gcc -o ..\csidlightorig.exe main.c tinywinmm.c libcsid-light-orig.c %opts% %link% 2> ..\errlightorig.log
popd