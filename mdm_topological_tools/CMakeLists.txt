cmake_minimum_required(VERSION 2.8.3)

project(mdm_topological_tools)

find_package(catkin REQUIRED COMPONENTS
                             roscpp
                             message_generation
                             mdm_library
                             predicate_manager
                             actionlib
                             actionlib_msgs)

find_package( SDL )
find_package( SDL_image )

add_message_files(FILES
                  PoseLabel.msg)

generate_messages(DEPENDENCIES std_msgs)

if ( SDLIMAGE_FOUND )

add_library(mdm_topological_tools
            src/topological_node.cpp
            src/topological_map.cpp
            src/topological_action_manager.cpp
            src/pose_labeler.cpp
            src/topological_predicate.cpp
            src/topological_move_base_action_layer.cpp)

else ( SDLIMAGE_FOUND )

add_library(mdm_topological_tools
            src/topological_node.cpp
            src/topological_map.cpp
            src/topological_action_manager.cpp
            src/topological_move_base_action_layer.cpp)
            
message(STATUS "WARNING: The SDL_image libary was not found on this system.")
message(STATUS "WARNING: You won't be able to use the pose_labeler tool or its dependencies.")

endif ( SDLIMAGE_FOUND )
            
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mdm_topological_tools
  CATKIN_DEPENDS roscpp mdm_library predicate_manager actionlib actionlib_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_dependencies(mdm_topological_tools ${PROJECT_NAME}_generate_messages_cpp)

if ( SDLIMAGE_FOUND )

target_link_libraries(mdm_topological_tools ${catkin_LIBRARIES} ${SDLIMAGE_LIBRARY})

add_executable(pose_labeler_node 
               src/pose_labeler_node.cpp)

add_dependencies(pose_labeler_node ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(pose_labeler_node ${catkin_LIBRARIES} ${SDLIMAGE_LIBRARY} mdm_topological_tools)

else ( SDLIMAGE_FOUND )

target_link_libraries(mdm_topological_tools ${catkin_LIBRARIES})

endif ( SDLIMAGE_FOUND )
