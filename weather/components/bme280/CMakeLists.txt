set(srcs "src/bme280.c")
set(public_requires)

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "include"
                       REQUIRES ${public_requires}) 
