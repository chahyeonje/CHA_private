cmake_minimum_required(VERSION 3.10)
project(CHA-private-project)

find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})
find_package(RBDL COMPONENTS URDFREADER REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS} ${RBDL_INCULDE_DIRS})
link_libraries(${EIGEN3_LIBRARIES}
        ${RBDL_LIBRARY}
        ${RBDL_URDFREADER_LIBRARY}
        raisim
        pthread)

add_executable(A1 src/a1.cpp)

