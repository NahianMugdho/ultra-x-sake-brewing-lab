# # The following lines of boilerplate have to be in your project's
# # CMakeLists in this exact order for cmake to work correctly
# cmake_minimum_required(VERSION 3.16)

# include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# # "Trim" the build. Include the minimal set of components, main, and anything it depends on.
# idf_build_set_property(MINIMAL_BUILD OFF)
# project(practise-test2)

# CMakeLists.txt for project root (MULTITHREAD-ONLINE directory)
# This file should be at: MULTITHREAD-ONLINE/CMakeLists.txt

cmake_minimum_required(VERSION 3.16)

# Add esp-idf-lib components
# Windows path - use forward slashes or escaped backslashes
set(EXTRA_COMPONENT_DIRS "D:/mugdho/japan office/multithread-online - updated/esp-idf-lib/components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(multithread-online)