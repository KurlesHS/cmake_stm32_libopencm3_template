# cmake_stm32_libopencm3_template
Шаблон для проекта на базе cmake для stm32 с использованием библиотеки libopencm3

пример команды на компиляцию:

*cmake .. -DCMAKE_TOOLCHAIN_FILE:STRING=/usr/share/cmake-3.11/toolchain.cmake -DCMAKE_CXX_COMPILER=arm-none-eabi-g++ -DCMAKE_C_COMPILER:STRING=arm-none-eabi-gcc -DCMAKE_BUILD_TYPE=Release*

