FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.c)

idf_component_register(
    SRCS ${app_sources}
    INCLUDE_DIRS "."
    REQUIRES
        driver
        esp_driver_mcpwm
        esp_driver_uart
        esp_driver_gpio
        led_strip
)
