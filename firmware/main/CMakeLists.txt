file(GLOB VL53L1_SRCS "VL53L1/core/src/*.c" "VL53L1/platform/src/*.c")

idf_component_register(SRCS "main.c" ${VL53L1_SRCS}
                    INCLUDE_DIRS "VL53L1/core/inc" "VL53L1/platform/inc")
