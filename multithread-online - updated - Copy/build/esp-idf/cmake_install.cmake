# Install script for directory: D:/mugdho/Espressif/frameworks/esp-idf-v5.5

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files (x86)/multithread-online")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "D:/mugdho/Espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-objdump.exe")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/xtensa/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_driver_gpio/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_pm/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/mbedtls/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/bootloader/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esptool_py/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/partition_table/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_app_format/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_bootloader_format/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/app_update/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_partition/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/efuse/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/bootloader_support/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_mm/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/spi_flash/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_system/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_common/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_rom/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/hal/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/log/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/heap/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/soc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_security/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_hw_support/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/freertos/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/newlib/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/pthread/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/cxx/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_timer/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_driver_gptimer/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_ringbuf/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_driver_uart/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/app_trace/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_event/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/nvs_flash/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_phy/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_driver_usb_serial_jtag/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_vfs_console/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/vfs/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/lwip/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_netif_stack/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_netif/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/wpa_supplicant/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_coex/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_wifi/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_driver_spi/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_gdbstub/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/bt/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/unity/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/cmock/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/console/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_driver_pcnt/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_driver_mcpwm/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_driver_ana_cmpr/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_driver_i2s/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/sdmmc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_driver_sdmmc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_driver_sdspi/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_driver_sdio/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_driver_dac/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_driver_bitscrambler/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_driver_rmt/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_driver_tsens/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_driver_sdm/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_driver_i2c/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_driver_ledc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_driver_parlio/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_driver_twai/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/driver/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/http_parser/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp-tls/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_adc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_driver_isp/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_driver_cam/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_psram/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_driver_jpeg/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_driver_ppa/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_driver_touch_sens/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_eth/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_hid/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/tcp_transport/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_http_client/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_http_server/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_https_ota/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_https_server/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_lcd/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/protobuf-c/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/protocomm/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_local_ctrl/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/espcoredump/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/wear_levelling/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/fatfs/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/idf_test/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/ieee802154/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/json/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/mqtt/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/nvs_sec_provider/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/openthread/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/perfmon/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/rt/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/spiffs/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/ulp/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/usb/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/wifi_provisioning/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/esp_idf_lib_helpers/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/dht/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/onewire/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/ds18x20/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/main/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/i2cdev/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/ads111x/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/ads130e08/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/aht/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/am2320/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/bh1750/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/bh1900nux/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/bme680/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/bmp180/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/bmp280/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/button/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/calibration/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/ccs811/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/lib8tion/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/color/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/dps310/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/ds1302/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/ds1307/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/ds3231/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/ds3502/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/encoder/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/example/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/framebuffer/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/hd44780/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/hdc1000/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/hmc5883l/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/ht16k33/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/hts221/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/hx711/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/icm42670/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/impulse_sensor/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/ina219/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/ina260/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/ina3221/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/l3gx/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/lc709203f/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/led_strip/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/led_strip_spi/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/lm75/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/ls7366r/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/lsm303/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/max1704x/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/max31725/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/max31855/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/max31865/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/max7219/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/mcp23008/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/mcp23x17/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/mcp342x/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/mcp4725/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/mcp960x/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/mcp9808/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/mhz19b/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/mp2660/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/mpu6050/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/ms5611/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/noise/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/pca9557/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/pca9632/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/pca9685/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/pcf8563/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/pcf8574/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/pcf8575/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/pcf8591/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/qmc5883l/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/qmi8658c/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/qmp6988/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/rda5807m/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/scd30/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/scd4x/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/sfa3x/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/sgm58031/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/sgp40/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/sht3x/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/sht4x/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/si7021/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/sts21/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/sts3x/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/tca6424a/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/tca9548/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/tca95x5/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/tda74xx/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/tps63101x/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/tsl2561/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/tsl2591/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/tsl4531/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/tsys01/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/ultrasonic/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/veml7700/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/mugdho/japan office/multithread-online - updated/build/esp-idf/wiegand/cmake_install.cmake")
endif()

