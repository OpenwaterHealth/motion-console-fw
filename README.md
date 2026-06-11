# Openwater Open-MOTION Console Firmware

This repository contains the firmware for the Motion Console.


CPPCheck
```bash

  cppcheck --enable=warning,style,performance,portability \
                   --std=c11 \
                   --error-exitcode=1 \
                   --force \
                   --inline-suppr \
                   --std=c11
                   --output-file=cppcheck_report.txt
                   --template="[{severity}] {file}:{line} {id} - {message}" \
                   --suppress=missingIncludeSystem \
                   --suppress=*:Core/Src/lwrb.c \
                   --suppress=*:Core/Src/jsmn.c \
                   --suppress=*:Core/Src/system_stm32h7xx.c \
                   --suppress=*:Core/Src/stm32h7xx_hal_msp.c \
                   --suppress=*:Core/Src/syscalls.c \
                   ./Core/Src ./USB_DEVICE/App


```