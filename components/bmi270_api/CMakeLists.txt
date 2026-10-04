# Bosch Sensortec BMI270 SensorAPI (BSD-3-Clause), unmodified. Used for chip init / config upload.
idf_component_register(
    SRCS "bmi2.c" "bmi270.c"
    INCLUDE_DIRS "."
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-function -Wno-maybe-uninitialized)
