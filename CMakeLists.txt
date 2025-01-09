project(nosCppUtilities LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
add_library(nosCppUtilities INTERFACE nosCppUtilities.hpp)

target_include_directories(nosCppUtilities INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})