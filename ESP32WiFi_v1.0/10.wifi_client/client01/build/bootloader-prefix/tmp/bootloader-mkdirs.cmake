# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/8.esp32/Espressif/frameworks/esp-idf-v5.1.2/components/bootloader/subproject"
  "D:/1.Program/6.esp32project/10.wifi_client/client01/build/bootloader"
  "D:/1.Program/6.esp32project/10.wifi_client/client01/build/bootloader-prefix"
  "D:/1.Program/6.esp32project/10.wifi_client/client01/build/bootloader-prefix/tmp"
  "D:/1.Program/6.esp32project/10.wifi_client/client01/build/bootloader-prefix/src/bootloader-stamp"
  "D:/1.Program/6.esp32project/10.wifi_client/client01/build/bootloader-prefix/src"
  "D:/1.Program/6.esp32project/10.wifi_client/client01/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/1.Program/6.esp32project/10.wifi_client/client01/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/1.Program/6.esp32project/10.wifi_client/client01/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
