cmake_minimum_required(VERSION 3.30)

project(step-driver)

include_directories(Inc)

file(GLOB_RECURSE STEPPER_MOTORS_SOURCES "Src/*.*")

add_library(${PROJECT_NAME} STATIC ${STEPPER_MOTORS_SOURCES})

target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Inc>
)