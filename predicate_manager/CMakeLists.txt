cmake_minimum_required(VERSION 2.8.3)

project(predicate_manager)

find_package(catkin REQUIRED COMPONENTS 
                             message_generation
                             std_msgs)
find_package(Boost REQUIRED)

add_message_files(FILES 
                  PredicateInfo.msg 
                  PredicateInfoMap.msg 
                  PredicateUpdate.msg
                  EventInfo.msg
                  EventInfoMap.msg
                  EventUpdate.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp message_generation
  )

add_library(${PROJECT_NAME} 
            src/prop_operator.cpp    
            src/prop_pv.cpp
            src/prop_and.cpp
            src/prop_or.cpp
            src/prop_not.cpp
            src/prop_xor.cpp
            src/prop_implies.cpp
            src/dependencies.cpp
            src/predicate.cpp
            src/prop_logic_predicate.cpp
            src/predicate_manager.cpp)
            
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
            
include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

set(CMAKE_BUILD_TYPE Debug)