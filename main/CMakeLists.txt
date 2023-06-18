idf_component_register(SRCS "main.c" "dht11.c" "ssd1306.c" "mqtt.c"
                    INCLUDE_DIRS ".")
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")