"""Converts Mark Oosting BAC Mono WAV sound bank to small 16kHz 16-bit loop data.
Source: https://github.com/markeasting/engine-audio (MIT license)."""
import os, subprocess, struct, sys
from pathlib import Path

SRC = Path(sys.argv[1]) if len(sys.argv)>1 else Path('sounds')
DEST = Path(sys.argv[2]) if len(sys.argv)>2 else Path('EngineSamples.h')
BANK=[('on_low','BAC_Mono_onlow.wav'),('on_high','BAC_Mono_onhigh.wav'),
      ('off_low','BAC_Mono_offlow.wav'),('off_high','BAC_Mono_offveryhigh.wav')]
lines=['#pragma once','#include <stdint.h>','#include <pgmspace.h>',
       '// Original BAC Mono WAV sources: markeasting/engine-audio, MIT. 16kHz mono PCM, 1.8 sec loops.']
for key, file in BANK:
    target=SRC/file
    if not target.exists():raise FileNotFoundError(str(target))
    # Original files are typically large. Trim and apply crossover at wrap boundary.
    raw=subprocess.check_output(['ffmpeg','-hide_banner','-loglevel','error','-y','-ss','0.4',
      '-i',str(target),'-ac','1','-ar','16000','-t','1.8','-f','s16le','-acodec','pcm_s16le','-'])
    data=list(struct.unpack('<%dh'%(len(raw)//2),raw))
    if not data:raise RuntimeError('empty WAV: '+file)
    n=min(1400,len(data)//5)
    # Last 87ms of loop blends with opening 87ms; preserve length for seamless playback.
    for i in range(n):
        weight=(i+1)/n
        idx=len(data)-n+i
        data[idx]=int((1-weight)*data[idx]+weight*data[i])
    name='engine_'+key
    lines.append(f'static const uint32_t ENGINE_{key.upper()}_LEN={len(data)}UL;')
    lines.append(f'static const int16_t {name}[] PROGMEM = {{')
    for i in range(0,len(data),16):
        lines.append('  '+','.join(str(z) for z in data[i:i+16])+',')
    lines.append('};')
    print(file,len(data),'frames',file=sys.stderr)
DEST.write_text('\n'.join(lines)+'\n')
print('Header bytes:',DEST.stat().st_size)