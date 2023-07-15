CMAKE_MINIMUM_REQUIRED(VERSION 3.10)
PROJECT(CHA-private-project)

set(RAISIM_PATH "$ENV{HOME}/Library/raisimLib/raisim/linux")
include_directories(${RAISIM_PATH}/include)
link_directories(${RAISIM_PATH}/lib)

ADD_COMPILE_DEFINITIONS (URDF_RSC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/urdf/")

add_subdirectory(examples)