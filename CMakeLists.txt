cmake_minimum_required(VERSION 3.8)
project(csmbus)

find_package(tutrc_ament_node REQUIRED)
find_package(diagnostic_updater REQUIRED)

tutrc_ament_init()

set(CSMBUS_SRCS
    src/csmbus/can/cc_io.cpp
    src/csmbus/can/cc_type.c
    src/csmbus/eth/ec_ctrl.cpp
    src/csmbus/eth/ec_socket.cpp
    src/csmbus/eth/ec_type.c
    src/csmbus/logger/logger.cpp
    src/csmbus/robomas.cpp
    src/csmbus/odrive.cpp
)

tutrc_ament_add_executable(smbus src/main.cpp ${CSMBUS_SRCS})

# Install launch files.
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
)

tutrc_ament_final()