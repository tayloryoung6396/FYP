# Find source files
file(GLOB_RECURSE src "*.c" "*.cpp" "*.hpp" "*.ipp")
add_library(nuclear OBJECT ${src})

set_target_properties(nuclear PROPERTIES LINKER_LANGUAGE CXX)