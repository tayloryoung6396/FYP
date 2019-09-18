# Find source files
file(GLOB_RECURSE src "*.c" "*.cpp" "*.hpp" "*.ipp")
add_library(module OBJECT ${src})
target_link_libraries(module utility)

# Include the directories as private unless they are needed elsewhere
target_include_directories(module PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
