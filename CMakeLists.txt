# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)


include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# O nome do projeto será o nome da pasta raiz
get_filename_component(PROJECT_DIR_NAME ${CMAKE_SOURCE_DIR} NAME)
project(${PROJECT_DIR_NAME})
#project(teste)
