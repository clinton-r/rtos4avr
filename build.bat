@setlocal
@if exist .\obj goto haveobj
mkdir .\obj
:haveobj
del /q .\obj\*.*

@rem  8MHz clock
@set switches=-Wall -save-temps=obj -std=c99 -O3 -DF_CPU=8000000 -mmcu=atmega328p

@rem  avr-gcc %switches% -c .\src\main.c --output .\obj\main.o
avr-gcc %switches% -c .\src\timer0.c --output .\obj\timer0.o
avr-gcc %switches% -c .\src\rtos.c --output .\obj\rtos.o
avr-gcc %switches% -x assembler-with-cpp -c .\src\rtos_asm.s --output .\obj\rtos_asm.o
@rem  avr-gcc %switches% -x assembler-with-cpp -c .\src\rtos_test_asm.s --output .\obj\rtos_test_asm.o

@rem  avr-gcc %switches% -Wl,--print-memory-usage .\obj\*.o --output main.elf
@rem  avr-objcopy -O ihex -R .eeprom main.elf main.hex
