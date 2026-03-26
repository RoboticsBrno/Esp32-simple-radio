cmake_minimum_required(VERSION 3.16)

set(SRCS
    "src/simple_radio.cpp"
)

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "./src"
    REQUIRES esp_event esp_netif esp_wifi nvs_flash
)

target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_20)
