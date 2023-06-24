@echo off

REM Run lzss.py to encode firmware.bin to firmware.lzss
python lzss.py --encode "..\.pio\build\portenta_h7_m7\firmware.bin" "..\.pio\build\portenta_h7_m7\firmware.lzss"

REM Run bin2ota.py to convert firmware.lzss to firmware.ota
python3 bin2ota.py PORTENTA_H7_M7 "..\.pio\build\portenta_h7_m7\firmware.lzss" "..\.pio\build\portenta_h7_m7\firmware.ota"
