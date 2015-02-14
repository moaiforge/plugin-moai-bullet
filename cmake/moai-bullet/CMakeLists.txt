cmake_minimum_required ( VERSION 2.8.5 )
project ( moai-bullet )    


  set ( MOAI_BULLET_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../moai-bullet" )  


  # Use all cpp files in directory as source code
  file ( GLOB MOAI_BULLET_SRC "${MOAI_BULLET_DIR}/*.cpp" "${MOAI_BULLET_DIR}/*.h" )

  add_library ( moai-bullet STATIC ${MOAI_BULLET_SRC} )

  target_include_directories (moai-bullet PUBLIC ${MOAI_BULLET_DIR} "${MOAI_BULLET_DIR}/../")

  target_link_libraries ( moai-bullet moai-sim moai-core moai-util bullet zlcore )
  
  set_target_properties(moai-bullet PROPERTIES FOLDER Modules)


