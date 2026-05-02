# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.22)

option(VGA "Enable VGA" OFF)
option(TFT "Enable TFT display" OFF)
option(ILI9341 "Enable TFT ILI9341 display" OFF)
option(HDMI "Enable HDMI display" OFF)
option(TV "Enable TV composite output" OFF)
option(SOFTTV "Enable TV soft composite output" OFF)
option(MURM20 "Murmulator 2.0" OFF)
option(ZERO2 "RP2350-PiZero" OFF)

set(ZERO2 ON)
#set(MURM20 ON)
#set(VGA ON)
#set(HDMI ON)
#set(TFT ON)
#set(ILI9341 ON)
#set(TV ON)
#set(SOFTTV ON)

set(VERSION "1.2.3")

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(OUTPUT_DIR "${CMAKE_SOURCE_DIR}/bin/${CMAKE_BUILD_TYPE}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${OUTPUT_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${OUTPUT_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${OUTPUT_DIR}")

IF(ZERO2)
    # For Board RP2350-PiZero
    set(PICO_BOARD_HEADER_DIRS ${CMAKE_SOURCE_DIR}/src/boards)
    set(PICO_BOARD waveshare_rp2350_pizero CACHE STRING "Board type" FORCE)
    set(PICO_PLATFORM rp2350-arm-s)
    set(CPU_MHZ 252)
ENDIF()
include(pico_sdk_import.cmake)

project(murmulator-tester C CXX ASM)
pico_sdk_init()

add_subdirectory(drivers/ps2kbd)
add_subdirectory(drivers/fatfs)
add_subdirectory(drivers/sdcard)
add_subdirectory(drivers/nespad)
add_subdirectory(drivers/psram)
add_subdirectory(drivers/audio)

add_subdirectory(drivers/vga-nextgen)
add_subdirectory(drivers/st7789)
add_subdirectory(drivers/hdmi)
add_subdirectory(drivers/tv)
add_subdirectory(drivers/tv-software)
add_subdirectory(drivers/graphics)

# INCLUDE FILES THAT SHOULD BE COMPILED:
file(GLOB_RECURSE SRC
 "src/*.cpp"
 "src/*.c"
# "drivers/usb/*.c"
)

message(STATUS "Add source files:")
foreach (SRC_FILE IN LISTS SRC)
    message(STATUS "${SRC_FILE}")
endforeach ()
message(STATUS "")

add_executable(${PROJECT_NAME} ${SRC})
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

#configure_file(memmap.ld.in memmap.ld @ONLY)
#pico_set_linker_script(${PROJECT_NAME} ${CMAKE_CURRENT_BINARY_DIR}/memmap.ld)
if (m1p2)
    pico_set_linker_script(${PROJECT_NAME} "${CMAKE_SOURCE_DIR}/memmap.ld")
endif ()

pico_set_program_name(${PROJECT_NAME} "Murmulator Tester")
pico_set_program_version(${PROJECT_NAME} ${VERSION})

target_link_libraries(${PROJECT_NAME} PRIVATE
        ps2kbd
        sdcard
        fatfs
        nespad
        psram
        audio

        graphics

        pico_runtime
        pico_stdlib
        hardware_pio

        pico_multicore
        hardware_flash
        hardware_exception
        hardware_pwm

#        tinyusb_board
#        tinyusb_device
#        pico_stdio_usb

        tinyusb_host tinyusb_board
#        stdc++

)

#family_configure_device_example(${PROJECT_NAME} noos)
target_link_options(${PROJECT_NAME} PRIVATE -Xlinker --print-memory-usage)
target_include_directories(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
target_compile_options(${PROJECT_NAME} PUBLIC
        -Wno-error=suggest-attribute=format
        -Wno-error=cast-qual
        -Wno-error=unused-parameter
        -Wno-error=conversion
        -Wno-error=format=
        -Wno-error=sign-compare
        -Wno-error=missing-field-initializers
        -Wno-error=switch
        -Wno-error=implicit-fallthrough=
        -Wno-error=stringop-truncation
        -Wno-error=restrict
        -w
)
endif ()

if (MURM20)
    target_compile_definitions(${PROJECT_NAME} PRIVATE
        PROJECT_VERSION="${VERSION}"
        MURM20=1
        # VGA 8 pins starts from pin:
        VGA_BASE_PIN=12

        # HDMI 8 pins starts from pin:
        HDMI_BASE_PIN=12

        # TFT
        TFT_CS_PIN=12
        TFT_RST_PIN=14
        TFT_LED_PIN=15
        TFT_DC_PIN=16
        TFT_DATA_PIN=18
        TFT_CLK_PIN=19

        # SDCARD
        SDCARD_PIN_SPI0_CS=5
        SDCARD_PIN_SPI0_SCK=6
        SDCARD_PIN_SPI0_MOSI=7
        SDCARD_PIN_SPI0_MISO=4

        # PS2 keyboard
        KBD_CLOCK_PIN=2
        KBD_DATA_PIN=3

        # NES Gamepad
        NES_GPIO_CLK=20
        NES_GPIO_LAT=21
        NES_GPIO_DATA=26

        PSRAM
        # PSRAM_MUTEX=1
        PSRAM_SPINLOCK=1
        PSRAM_ASYNC=1

        PSRAM_PIN_CS=8
        PSRAM_PIN_SCK=6
        PSRAM_PIN_MOSI=7
        PSRAM_PIN_MISO=4

#        BUTTER_PSRAM_GPIO=8
        BUTTER_PSRAM_GPIO=47

        BEEPER_PIN=9
        PWM_PIN0=10
        PWM_PIN1=11

        AUDIO_DATA_PIN=9
        AUDIO_CLOCK_PIN=10
    )
ELSEIF(ZERO2)
    target_compile_definitions(${PROJECT_NAME} PRIVATE
        PROJECT_VERSION="${VERSION}"
        ZERO2=1
        # VGA 8 pins starts from pin:
        VGA_BASE_PIN=32

        # HDMI 8 pins starts from pin:
        HDMI_BASE_PIN=32

        # PS2 keyboard (moved from GPIO 2/3 for PCM5122 I2C)
        KBD_CLOCK_PIN=14
        KBD_DATA_PIN=15

        AUDIO_DATA_PIN=10 # DIN
        AUDIO_CLOCK_PIN=11 # BCK, 12 # LRCK

        PWM_PIN0=10
        PWM_PIN1=11
        BEEPER_PIN=12
        
        USE_NESPAD
        NES_GPIO_CLK=4
        NES_GPIO_LAT=5
        NES_GPIO_DATA=7
        NES_GPIO_DATA2=8

        SDCARD_SPI_BUS=spi1
        SDCARD_PIN_SPI0_SCK=30
        SDCARD_PIN_SPI0_MOSI=31
        SDCARD_PIN_SPI0_MISO=40
        SDCARD_PIN_SPI0_CS=43
    )
else ()
    target_compile_definitions(${PROJECT_NAME} PRIVATE
        PROJECT_VERSION="${VERSION}"
        # VGA 8 pins starts from pin:
        VGA_BASE_PIN=6

        # HDMI 8 pins starts from pin:
        HDMI_BASE_PIN=6

        # TFT
        TFT_CS_PIN=6
        TFT_RST_PIN=8
        TFT_LED_PIN=9
        TFT_DC_PIN=10
        TFT_DATA_PIN=12
        TFT_CLK_PIN=13

        # SDCARD
        SDCARD_PIN_SPI0_SCK=2
        SDCARD_PIN_SPI0_MOSI=3
        SDCARD_PIN_SPI0_MISO=4
        SDCARD_PIN_SPI0_CS=5

        # PS2 keyboard
        KBD_CLOCK_PIN=0
        KBD_DATA_PIN=1

        # NES Gamepad
        NES_GPIO_CLK=14
        NES_GPIO_DATA=16
        NES_GPIO_LAT=15

        PSRAM
        # PSRAM_MUTEX=1
        PSRAM_SPINLOCK=1
        PSRAM_ASYNC=1

        PSRAM_PIN_CS=18
        PSRAM_PIN_SCK=19
        PSRAM_PIN_MOSI=20
        PSRAM_PIN_MISO=21

#        BUTTER_PSRAM_GPIO=19
#        BUTTER_PSRAM_GPIO=47
DEBUG=1

        PWM_PIN0=26
        PWM_PIN1=27
        BEEPER_PIN=28

        AUDIO_DATA_PIN=26
        AUDIO_CLOCK_PIN=27
    )
endif ()

if (PICO_BOARD MATCHES "waveshare_rp2350_pizero")
    SET(BUILD_NAME "z0p2-${PROJECT_NAME}")
    target_compile_definitions(${PROJECT_NAME} PRIVATE PLAT="z0p2")
elseif (PICO_BOARD MATCHES "pico2")
    if (MURM20)
        SET(BUILD_NAME "m2p2-${PROJECT_NAME}")
        target_compile_definitions(${PROJECT_NAME} PRIVATE PLAT="m2p2")
    else ()
        SET(BUILD_NAME "m1p2-${PROJECT_NAME}")
        target_compile_definitions(${PROJECT_NAME} PRIVATE PLAT="m1p2")
    endif ()
else ()
    if (MURM20)
        SET(BUILD_NAME "m2p1-${PROJECT_NAME}")
        target_compile_definitions(${PROJECT_NAME} PRIVATE PLAT="m2p1")
    elseif (ZERO)
        SET(BUILD_NAME "z0p1-${PROJECT_NAME}")
        target_compile_definitions(${PROJECT_NAME} PRIVATE PLAT="z0p1")
    else ()
        SET(BUILD_NAME "m1p1-${PROJECT_NAME}")
        target_compile_definitions(${PROJECT_NAME} PRIVATE PLAT="m1p1")
    endif ()
    pico_define_boot_stage2(slower_boot2 ${PICO_DEFAULT_BOOT_STAGE2_FILE})
    target_compile_definitions(slower_boot2 PRIVATE PICO_FLASH_SPI_CLKDIV=4)
    pico_set_boot_stage2(${PROJECT_NAME} slower_boot2)
endif ()

IF (TFT)
    target_link_libraries(${PROJECT_NAME} PRIVATE st7789)
    target_compile_definitions(${PROJECT_NAME} PRIVATE TFT)
    SET(BUILD_NAME "${BUILD_NAME}-TFT")
    IF (ILI9341)
        SET(BUILD_NAME "${BUILD_NAME}-ILI9341")
        target_compile_definitions(${PROJECT_NAME} PRIVATE ILI9341)
    ELSE ()
        SET(BUILD_NAME "${BUILD_NAME}-ST7789")
    ENDIF ()
ELSEIF(TV)
    target_compile_definitions(${PROJECT_NAME} PRIVATE TV)
    target_link_libraries(${PROJECT_NAME} PRIVATE tv)
    SET(BUILD_NAME "${BUILD_NAME}-TV")
ELSEIF(SOFTTV)
	target_compile_definitions(${PROJECT_NAME} PRIVATE SOFTTV)
	target_link_libraries(${PROJECT_NAME} PRIVATE tv-software)
	SET(BUILD_NAME "${BUILD_NAME}-TV-SOFT")
ELSE()
    target_link_libraries(${PROJECT_NAME} PRIVATE hdmi vga-nextgen)
    target_compile_definitions(${PROJECT_NAME} PRIVATE HDMI VGA)
    SET(BUILD_NAME "${BUILD_NAME}-VGA-HDMI")
ENDIF()

if (m1p2)
    SET(BUILD_NAME "${BUILD_NAME}-${VERSION}.m1p2")
endif()

pico_enable_stdio_uart(${PROJECT_NAME} 0)
pico_enable_stdio_usb(${PROJECT_NAME} 0)

#pico_set_binary_type(${PROJECT_NAME} no_flash)
#pico_set_binary_type(${PROJECT_NAME} copy_to_ram)

pico_add_extra_outputs(${PROJECT_NAME})

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "${BUILD_NAME}")
