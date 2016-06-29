rem @ECHO OFF

CALL venv.bat

if exist MAKEFILE_FLASHBIOS MAKE -fMAKEFILE_FLASHBIOS CLEAN %2 %3 %4 %5
if exist MAKEFILE_FLASHBIOS MAKE -fMAKEFILE_FLASHBIOS %1 %2 %3 %4 %5
@ECHO *
@ECHO ************** Flash Bios proceeded **************

if exist MAKEFILE_FLASHBIOS_0x58100 MAKE -fMAKEFILE_FLASHBIOS_0x58100 CLEAN %2 %3 %4 %5
if exist MAKEFILE_FLASHBIOS_0x58100 MAKE -fMAKEFILE_FLASHBIOS_0x58100 %1 %2 %3 %4 %5
@ECHO *
@ECHO ************** Flash Bios for address 0x58100 proceeded **************










