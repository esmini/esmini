
include_directories (
  ${ESMINI_LIB_PATH}
  ${COMMON_MINI_PATH}
  ${EXTERNALS_PUGIXML_PATH}
)

set (TARGET hello_world)
set (TARGET_TMP hello_world_tmp)

add_custom_target(${TARGET_TMP})

file(GLOB modules RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.cpp")

foreach(file_path ${modules})
  get_filename_component( filename ${file_path} NAME )
  string( REPLACE ".cpp" "" module_name ${filename} )

  add_executable( ${module_name} ${filename} )

  add_dependencies( ${TARGET_TMP} ${module_name} )

  target_link_libraries( ${module_name} esminiLib project_options)

  set_target_properties ( ${module_name} PROPERTIES FOLDER ${CodeExamplesFolder}/${TARGET} )

  install ( TARGETS ${module_name} DESTINATION ${CODE_EXAMPLES_BIN_PATH})

endforeach()
