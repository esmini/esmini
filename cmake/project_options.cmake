add_library(project_options INTERFACE)

target_compile_features(project_options INTERFACE cxx_std_${CMAKE_CXX_STANDARD})

include(${CMAKE_CURRENT_LIST_DIR}/cache.cmake)
enable_cache()

include(${CMAKE_CURRENT_LIST_DIR}/sanitizers.cmake)
if(ENABLE_SANITIZERS)
  enable_sanitizers(project_options)
endif()
