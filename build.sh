#!/usr/bin/bash

# Make sure obj is existing directory
if [[ ! -d "./obj" ]]; then
  mkdir ./obj
fi

rm -f ./obj/*

# 8MHz clock
readonly SWITCHES="-Wall -save-temps=obj -std=c99 -O0 -DF_CPU=8000000 -mmcu=atmega328p"
# avr-gcc ${SWITCHES} -c ./src/main.c --output ./obj/main.o
avr-gcc ${SWITCHES} -c ./src/timer0.c --output ./obj/timer0.o
avr-gcc ${SWITCHES} -c ./src/rtos.c --output ./obj/rtos.o
avr-gcc ${SWITCHES} -x assembler-with-cpp -c ./src/rtos_asm.s --output ./obj/rtos_asm.o
# avr-gcc ${SWITCHES} -x assembler-with-cpp -c ./src/rtos_test_asm.s --output ./obj/rtos_test_asm.o

# avr-gcc ${SWITCHES} -Wl,--print-memory-usage ./obj/*.o --output main.elf
# avr-objcopy -O ihex -R .eeprom main.elf main.hex

