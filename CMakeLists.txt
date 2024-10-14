cmake_minimum_required(VERSION 3.8)
project(csmbus)

find_package(tutrc_ament_node REQUIRED)

tutrc_ament_init()

set(SMBUS_SRCS
    src/smbus/can_smbus/cs_io.cpp
    src/smbus/can_smbus/cs_type.c
    src/smbus/eth_smbus/es_ctrl.cpp
    src/smbus/eth_smbus/es_socket.cpp
    src/smbus/eth_smbus/es_type.c
    src/smbus/logger/logger.cpp
    src/smbus/robomas.cpp
    src/smbus/odrive.cpp
)

tutrc_ament_add_executable(smbus src/main.cpp ${SMBUS_SRCS})

# Install launch files.
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
)

tutrc_ament_final()