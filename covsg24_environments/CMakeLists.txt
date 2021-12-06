cmake_minimum_required(VERSION 3.5)
project(covsg24_environments)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  environments/
  DESTINATION share/${PROJECT_NAME}/environments/
)

ament_package()

file(GLOB_RECURSE traffic_editor_paths "environments/*.building.yaml")

foreach(path ${traffic_editor_paths})

  # Get the output world name 
  string(REGEX REPLACE "\\.[^.]*\.[^.]*$" "" no_extension_path ${path})
  string(REGEX MATCH "[^\/]+$" world_name  ${no_extension_path})

  set(map_path ${path})
  set(output_world_name ${world_name})
  set(output_dir ${CMAKE_CURRENT_BINARY_DIR}/${output_world_name})
  set(output_world_path ${output_dir}/${output_world_name}.world)
  set(output_model_dir ${output_dir}/models)

  # first, generate the world 
  message("BUILDING WORLDFILE WITH COMMAND: ros2 run rmf_building_map_tools building_map_generator gazebo ${map_path} ${output_world_path} ${output_model_dir}")
  if (NO_DOWNLOAD_MODELS)
    add_custom_command(
      DEPENDS ${map_path}
      COMMAND ros2 run rmf_building_map_tools building_map_generator gazebo ${map_path} ${output_world_path} ${output_model_dir}
      COMMAND cp -a ${output_model_dir}/. "~/.gazebo/models/" && mkdir -p "~/.gazebo/worlds/" && cp ${output_world_path} "~/.gazebo/worlds/"
      OUTPUT ${output_world_path}
    )
  else()
    message("DOWNLOADING MODELS WITH COMMAND: ros2 run rmf_building_map_tools building_map_model_downloader ${map_path}")
    add_custom_command(
      DEPENDS ${map_path}
      COMMAND ros2 run rmf_building_map_tools building_map_generator gazebo ${map_path} ${output_world_path} ${output_model_dir}
      COMMAND cp -a ${output_model_dir}/. "~/.gazebo/models/" && mkdir -p "~/.gazebo/worlds/" && cp ${output_world_path} "~/.gazebo/worlds/"
      COMMAND ros2 run rmf_building_map_tools building_map_model_downloader ${map_path}
      OUTPUT ${output_world_path}
    )
  endif()

  add_custom_target(generate_${output_world_name} ALL
    DEPENDS ${output_world_path}
  )

  # now, generate the nav graphs
  set(output_nav_graphs_dir ${output_dir}/)
  set(output_nav_graphs_phony ${output_nav_graphs_dir}/phony)
  message("GENERATING NAV GRAPHS WITH COMMAND: ros2 run rmf_building_map_tools building_map_generator nav ${map_path} ${output_nav_graphs_dir}")
  add_custom_command(
    OUTPUT ${output_nav_graphs_phony}
    COMMAND ros2 run rmf_building_map_tools building_map_generator nav ${map_path} ${output_nav_graphs_dir}
    DEPENDS ${map_path}
  )

  add_custom_target(generate_${output_world_name}_nav_graphs ALL
    DEPENDS ${output_nav_graphs_phony}
  )

  install(
    DIRECTORY ${output_dir}
    DESTINATION share/${PROJECT_NAME}/environments/
  )

endforeach()
