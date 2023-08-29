function(force_regenerate_at_next_build)
    set( FLAG_FILE_REBUILD ${CMAKE_CURRENT_SOURCE_DIR}/.flag_regenerate_cmake )
    execute_process ( COMMAND touch ${FLAG_FILE_REBUILD} )
    add_custom_target(regenerate_cmake ALL)
    add_custom_command(
        TARGET  regenerate_cmake
        DEPENDS ${FLAG_FILE_REBUILD}
        COMMAND rm -f ${FLAG_FILE_REBUILD}
        COMMENT "The cmake configuration will be regenerated at next build."
    )
    set_property(
        DIRECTORY
        APPEND
        PROPERTY CMAKE_CONFIGURE_DEPENDS
        ${FLAG_FILE_REBUILD}
    )
endfunction()
