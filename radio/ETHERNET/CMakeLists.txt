add_library(oai_eth_transpro MODULE
            ethernet_lib.c
            eth_udp.c
            eth_raw.c
)
set_target_properties(oai_eth_transpro PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

add_subdirectory(benetel)
