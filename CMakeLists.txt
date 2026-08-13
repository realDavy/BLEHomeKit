cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32c3-lcdkit-hap-ble-light)

# HAP requires C++20 (std::span, concepts).
idf_build_set_property(CXX_COMPILE_OPTIONS "-std=gnu++20" APPEND)
