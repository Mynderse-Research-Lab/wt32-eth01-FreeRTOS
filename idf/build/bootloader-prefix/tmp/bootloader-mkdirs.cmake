# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/ziaah/esp-idf/components/bootloader/subproject"
  "D:/Projects/wt32-eth01-base/idf/build/bootloader"
  "D:/Projects/wt32-eth01-base/idf/build/bootloader-prefix"
  "D:/Projects/wt32-eth01-base/idf/build/bootloader-prefix/tmp"
  "D:/Projects/wt32-eth01-base/idf/build/bootloader-prefix/src/bootloader-stamp"
  "D:/Projects/wt32-eth01-base/idf/build/bootloader-prefix/src"
  "D:/Projects/wt32-eth01-base/idf/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/Projects/wt32-eth01-base/idf/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/Projects/wt32-eth01-base/idf/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
