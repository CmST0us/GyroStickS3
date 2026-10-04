# Portable (no ESP-IDF dependency) page format + sample codec.
idf_component_register(
    SRCS "gyl_format.c" "gyl_codec.c"
    INCLUDE_DIRS "include"
)
