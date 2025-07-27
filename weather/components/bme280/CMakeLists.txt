set(srcs "src/bme280.c")
set(public_requires)
list(APPEND public_requires "esp_driver_i2c")

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "include"
                       REQUIRES ${public_requires}) 
