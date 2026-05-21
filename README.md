# Openwater Open-MOTION Console Firmware

This repository contains the firmware for the Motion Console.


CPPCheck
```bash

cppcheck --enable=warning,style,performance,portability \
  --inconclusive
  --force
  --std=c11
  --output-file=cppcheck_report.txt
  --template="[{severity}] {file}:{line} {id} - {message}"
  core/src/          
  USB_DEVICE/ 
  


```