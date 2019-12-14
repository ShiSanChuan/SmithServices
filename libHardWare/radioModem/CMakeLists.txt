include_directories(./include)
include_directories(./inc)
AUX_SOURCE_DIRECTORY(./src SERIAL_DIR_SRCS)
ADD_LIBRARY(RadioInterface ${SERIAL_DIR_SRCS})

