# SPDX-License-Identifier: LicenseRef-CSSL-1.0

add_compile_options(-Wunused-parameter)
find_library(yunsdr_ss NAMES yunsdr_ss)

add_library(oai_yunsdrdevif MODULE yunsdr_lib.c)
target_link_libraries(oai_yunsdrdevif PRIVATE yunsdr_ss)
target_link_libraries(oai_yunsdrdevif PRIVATE UTIL)
set_target_properties(oai_yunsdrdevif PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
add_custom_command(TARGET oai_yunsdrdevif POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E create_symlink liboai_yunsdrdevif.so liboai_device.so
                   WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
