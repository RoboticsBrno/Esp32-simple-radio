cmake_minimum_required(VERSION 3.0)

set(SRCS
    "src/simple_radio.cpp"
)

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "./src"
    REQUIRES bt nvs_flash
)
