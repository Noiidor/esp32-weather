set(srcs "src/hc8.c")
set(public_requires)
list(APPEND public_requires "esp_driver_uart")

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "include"
                       REQUIRES ${public_requires}) 
