C:\Windows\System32\taskkill.exe /F /IM JLinkGDBServer.exe
start "" "C:\Program Files (x86)\SEGGER\JLinkARM_V480a\JLinkGDBServer.exe" -select USB -device MK60DN512xxx10 -if JTAG -speed 1000
F:\gcc-arm-none-eabi-4_8-2013q4-20131204-win32\bin\arm-none-eabi-gdb.exe %*