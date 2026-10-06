cmake_minimum_required(VERSION 3.18)
project(overtake_nmpc CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
# no fused multiply-add, so results do not depend on the CPU the code was built for
add_compile_options(-Wall -Wextra -ffp-contract=off)

# ---- dependencies

# CasADi (with IPOPT) is the pip package in the Python environment. Its library
# is built with the old libstdc++ string ABI, so everything that passes strings
# to it has to be as well, Chrono included.
add_compile_definitions(_GLIBCXX_USE_CXX11_ABI=0)

if(NOT Python3_EXECUTABLE AND EXISTS ${CMAKE_SOURCE_DIR}/.venv/bin/python)
    set(Python3_EXECUTABLE ${CMAKE_SOURCE_DIR}/.venv/bin/python)
endif()
find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)
execute_process(
    COMMAND ${Python3_EXECUTABLE} -c "import casadi, os; print(os.path.dirname(casadi.__file__))"
    OUTPUT_VARIABLE CASADI_DIR OUTPUT_STRIP_TRAILING_WHITESPACE RESULT_VARIABLE casadi_missing)
if(casadi_missing)
    message(FATAL_ERROR "CasADi not found in ${Python3_EXECUTABLE}: pip install casadi")
endif()
execute_process(
    COMMAND ${Python3_EXECUTABLE} -m pybind11 --cmakedir
    OUTPUT_VARIABLE pybind11_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
find_package(pybind11 CONFIG REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include(FetchContent)
FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz)
FetchContent_MakeAvailable(json)

# Project Chrono is optional: without it only the bicycle plant is available.
find_package(Chrono COMPONENTS Vehicle CONFIG QUIET PATHS $ENV{HOME}/opt/chrono/install/lib/cmake/Chrono)

# ---- the library: model, plants, controllers, scenarios

set(SOURCES
    parameters.cpp
    plant/bicycle.cpp
    plant/bicycle_plant.cpp
    controller/pure_pursuit/path.cpp
    controller/pure_pursuit/overtake.cpp
    controller/pure_pursuit/speed_profile.cpp
    controller/pure_pursuit/track.cpp
    controller/nmpc/overtake.cpp
    controller/nmpc/track.cpp
    sim/geometry.cpp
    sim/perception.cpp
    sim/scenarios/overtake/scenario.cpp
    sim/scenarios/overtake/run.cpp
    sim/scenarios/circuit/track.cpp
    sim/scenarios/circuit/scenario.cpp
    sim/scenarios/circuit/run.cpp)
if(Chrono_FOUND)
    list(APPEND SOURCES plant/chrono_plant.cpp)
else()
    list(APPEND SOURCES plant/chrono_plant_missing.cpp)
endif()

add_library(overtake STATIC ${SOURCES})
target_include_directories(overtake PUBLIC ${CMAKE_SOURCE_DIR})
target_include_directories(overtake SYSTEM PUBLIC ${CASADI_DIR}/include)
target_compile_definitions(overtake PRIVATE
    OV_PARAMETERS_JSON="${CMAKE_SOURCE_DIR}/parameters.json"
    OV_CHRONO_PARAMETERS_JSON="${CMAKE_SOURCE_DIR}/parameters_chrono_sedan.json")
target_link_libraries(overtake PUBLIC Eigen3::Eigen nlohmann_json::nlohmann_json ${CASADI_DIR}/libcasadi.so)
set(RPATH ${CASADI_DIR})

if(Chrono_FOUND)
    message(STATUS "Project Chrono found: ${CHRONO_LIBRARIES}")
    target_include_directories(overtake SYSTEM PRIVATE ${CHRONO_INCLUDE_DIRS})
    # Chrono is built with -march=native; its headers must be compiled the same way
    set_source_files_properties(plant/chrono_plant.cpp PROPERTIES COMPILE_OPTIONS "-march=native;-fopenmp")
    target_compile_definitions(overtake PRIVATE OV_CHRONO_DATA="${CHRONO_DATA_DIR}" OV_CHRONO_VEHICLE_DATA="${CHRONO_VEHICLE_DATA_DIR}")
    target_link_libraries(overtake PUBLIC ${CHRONO_LIBRARIES})
    list(APPEND RPATH ${CHRONO_LIB_DIR} $ENV{HOME}/opt/chrono/install/lib)
endif()

# ---- the simulation generator

add_executable(simulate sim/generator.cpp)
target_link_libraries(simulate PRIVATE overtake)
set_target_properties(simulate PROPERTIES BUILD_RPATH "${RPATH}")

# ---- identification of the bicycle-model parameters of the Chrono sedan

if(Chrono_FOUND)
    add_executable(identify_chrono plant/identify_chrono.cpp)
    target_link_libraries(identify_chrono PRIVATE overtake)
    set_target_properties(identify_chrono PROPERTIES BUILD_RPATH "${RPATH}")
endif()

# ---- Python module for the tests and the animations, written next to the sources

pybind11_add_module(overtake_core bindings/overtake_core.cpp)
target_link_libraries(overtake_core PRIVATE overtake)
set_target_properties(overtake_core PROPERTIES BUILD_RPATH "${RPATH}" LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})
