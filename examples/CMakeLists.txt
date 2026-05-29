###########################################################
# DLIO examples                                           #
###########################################################

include(GNUInstallDirs)

# File-replay driver (no extra deps beyond the core lib)
add_executable(dlio_live_mapping live_mapping.cpp)
target_link_libraries(dlio_live_mapping dlio)
install(TARGETS dlio_live_mapping RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# Live viewer (optional) — pulls PCL visualization (VTK) into THIS target only.
option(DLIO_BUILD_VIZ "Build the live PCL-visualizer example" ON)
if(DLIO_BUILD_VIZ)
  find_package(PCL QUIET COMPONENTS common io visualization)
  if(PCL_visualization_FOUND OR PCL_VISUALIZATION_FOUND)
    add_executable(dlio_live_viz live_viz.cpp)
    target_link_libraries(dlio_live_viz dlio ${PCL_LIBRARIES})
    target_include_directories(dlio_live_viz PRIVATE ${PCL_INCLUDE_DIRS})
    install(TARGETS dlio_live_viz RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
    message(STATUS "dlio: building dlio_live_viz (PCL visualization found)")
  else()
    message(STATUS "dlio: PCL visualization not found, skipping dlio_live_viz")
  endif()
endif()
