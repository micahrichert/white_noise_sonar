set -e
cd libopencm3
make lib/stm32/f4
cd ../stlink
make
cd ..

arm-none-eabi-g++ -save-temps -O3 -fpermissive -std=c++11 -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32F4 -Ilibopencm3/include -Tstm32f4-discovery.ld stm32f4.cpp libopencm3/lib/libopencm3_stm32f4.a -o white_noise.elf  -nostartfiles -Wl,--gc-sections -lgcc -lm -lc -lc -lrdimon -lnosys

arm-none-eabi-objcopy -Obinary white_noise.elf white_noise.bin
./flash_and_reboot.sh white_noise.bin
