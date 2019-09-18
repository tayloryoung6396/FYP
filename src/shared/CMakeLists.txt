# Find source files
file(GLOB_RECURSE src "*.c" "*.cpp" "*.hpp" "*.ipp")
add_library(utility OBJECT ${src})
target_link_libraries(utility stm32 nuclear)

# Include the directories as private unless they are needed elsewhere
target_include_directories(utility PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
