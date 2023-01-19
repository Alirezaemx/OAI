find_library(ori NAMES "libori.so" REQUIRED)
if(NOT ori)
  message(FATAL_ERROR "cannot locate libori.so")
endif()

add_library(aw2sori_transpro MODULE oaiori.c)
target_compile_options(aw2sori_transpro PRIVATE -shared -fPIC -msse4 -g -ggdb -DLITE_COMPILATION)
target_link_libraries(aw2sori_transpro PRIVATE ori)
set_target_properties(aw2sori_transpro PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
