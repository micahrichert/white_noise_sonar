cd libopencm3
make
cd ../stlink
make
cd ..
arm-none-eabi-g++ -std=c++11 -DUSE_LIBOPENCM3 -mfloat-abi=hard -DSTM32F4 -Ilibopencm3/include stm32f4.cpp libopencm3/lib/libopencm3_stm32f4.a -nostartfiles -Wl,--gc-sections -lgcc -lm -lc -lc -lrdimon -lnosys -Tstm32f4-discovery.ld -o white_noise.elf

arm-none-eabi-objcopy -Obinary white_noise.elf white_noise.bin
./flash_and_reboot.sh white_noise.bin
