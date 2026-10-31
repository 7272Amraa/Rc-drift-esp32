"""Add live GPIO13-held flag to TX V6 accessory packets without changing protocol size.
The high bit of hornToggle means Button 1 currently held; the lower 7 bits
are the short-press event counter, keeping legacy RX compatible.
Run on the COMPACT TX source copied to the Arduino build directory.
"""
from pathlib import Path
import sys
p=Path(sys.argv[1])
s=p.read_text()
needle='a.hornToggle=hornCounter;'
replacement='a.hornToggle=(hornCounter & 0x7F) | (button.stable==LOW ? 0x80 : 0);'
if replacement in s:
 print('TX horn-held patch already applied')
elif needle in s:
 s=s.replace(needle,replacement)
 # The 7-bit counter wraps; receiver uses modulo 128 comparison.
 p.write_text(s)
 print('TX horn held-state patch applied')
else:
 raise SystemExit('TX protocol changed: cannot apply safe patch')