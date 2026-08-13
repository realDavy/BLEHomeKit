idf_component_register()
add_subdirectory(hap)
target_link_libraries(${COMPONENT_LIB} INTERFACE hap)
