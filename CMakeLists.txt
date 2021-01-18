cmake_minimum_required(VERSION 3.0)
project(py_osrm)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/")
set(CMAKE_CXX_STANDARD 14)

## PYBIND SETUP
add_subdirectory(pybind11)
pybind11_add_module(PyOSRM main.cpp)

find_package(LibOSRM REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

target_link_libraries(PyOSRM PRIVATE ${LibOSRM_LIBRARIES} ${LibOSRM_DEPENDENT_LIBRARIES})
target_link_libraries (PyOSRM PRIVATE Eigen3::Eigen)

include_directories(SYSTEM ${LibOSRM_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${LibOSRM_CXXFLAGS}")

install(TARGETS PyOSRM DESTINATION /usr/local/lib/python3.6/dist-packages)
