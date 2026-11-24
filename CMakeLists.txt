cmake_minimum_required(VERSION 3.16)

# Passwords live in sdkconfig.defaults.local, which git ignores. It is read
# after the shared defaults, so its values win.
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/sdkconfig.defaults.local")
    set(SDKCONFIG_DEFAULTS "sdkconfig.defaults;sdkconfig.defaults.local")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(jk_bms_can)
