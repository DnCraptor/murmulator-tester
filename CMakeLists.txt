# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.22)
include(pico_sdk_import.cmake)

project(murmulator-tester C CXX ASM)
pico_sdk_init()

option(VGA "Enable VGA" OFF)
option(TFT "Enable TFT display" OFF)
option(ILI9341 "Enable TFT ILI9341 display" OFF)
option(HDMI "Enable HDMI display" OFF)
option(TV "Enable TV composite output" OFF)
option(SOFTTV "Enable TV soft composite output" OFF)
option(MURM20 "Murmulator 2.0" OFF)

#set(MURM20 ON)
#set(VGA ON)
#set(HDMI ON)
#set(TFT ON)
#set(ILI9341 ON)
#set(TV ON)
#set(SOFTTV ON)

set(VERSION "1.0.5")

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(OUTPUT_DIR "${CMAKE_SOURCE_DIR}/bin/${CMAKE_BUILD_TYPE}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${OUTPUT_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${OUTPUT_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${OUTPUT_DIR}")

add_subdirectory(drivers/ps2)
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

pico_set_program_name(${PROJECT_NAME} "Murmulator Tester")
pico_set_program_version(${PROJECT_NAME} ${VERSION})

target_link_libraries(${PROJECT_NAME} PRIVATE
        ps2
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
        hardware_pwm

        tinyusb_board
        tinyusb_device
        pico_stdio_usb
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

        # PSRAM
        # PSRAM_MUTEX=1
        PSRAM_SPINLOCK=1
        PSRAM_ASYNC=1

        PSRAM_PIN_CS=18
        PSRAM_PIN_SCK=19
        PSRAM_PIN_MOSI=20
        PSRAM_PIN_MISO=21

        BEEPER_PIN=9
        PWM_PIN0=10
        PWM_PIN1=11

        AUDIO_DATA_PIN=9
        AUDIO_CLOCK_PIN=10
    )
else ()
    target_compile_definitions(${PROJECT_NAME} PRIVATE
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

        PWM_PIN0=26
        PWM_PIN1=27
        BEEPER_PIN=28

        AUDIO_DATA_PIN=26
        AUDIO_CLOCK_PIN=27
    )
endif ()

if (PICO_BOARD MATCHES "pico2")
    if (MURM20)
        SET(BUILD_NAME "m2p2-${PROJECT_NAME}")
    else ()
        SET(BUILD_NAME "m1p2-${PROJECT_NAME}")
    endif ()
else ()
    if (MURM20)
        SET(BUILD_NAME "m2p1-${PROJECT_NAME}")
    else ()
        SET(BUILD_NAME "m1p1-${PROJECT_NAME}")
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
ELSEIF (HDMI)
    target_link_libraries(${PROJECT_NAME} PRIVATE hdmi)
    target_compile_definitions(${PROJECT_NAME} PRIVATE HDMI)
    SET(BUILD_NAME "${BUILD_NAME}-HDMI")
ELSEIF(TV)
    target_compile_definitions(${PROJECT_NAME} PRIVATE TV)
    target_link_libraries(${PROJECT_NAME} PRIVATE tv)
    SET(BUILD_NAME "${BUILD_NAME}-TV")
ELSEIF(SOFTTV)
	target_compile_definitions(${PROJECT_NAME} PRIVATE SOFTTV)
	target_link_libraries(${PROJECT_NAME} PRIVATE tv-software)
	SET(BUILD_NAME "${BUILD_NAME}-TV-SOFT")
ELSE()
    target_compile_definitions(${PROJECT_NAME} PRIVATE VGA)
    target_link_libraries(${PROJECT_NAME} PRIVATE vga-nextgen)
    SET(BUILD_NAME "${BUILD_NAME}-VGA")
ENDIF()

SET(BUILD_NAME "${BUILD_NAME}-${VERSION}")

pico_enable_stdio_uart(${PROJECT_NAME} 0)
pico_enable_stdio_usb(${PROJECT_NAME} 1)

#pico_set_binary_type(${PROJECT_NAME} no_flash)
#pico_set_binary_type(${PROJECT_NAME} copy_to_ram)

pico_add_extra_outputs(${PROJECT_NAME})

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "${BUILD_NAME}")
