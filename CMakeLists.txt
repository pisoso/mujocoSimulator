# 需要cmake的最低版本
cmake_minimum_required(VERSION 3.5)

# 设定项目名称 并指定语言
project(MyRobotSimulator LANGUAGES CXX)

set( USE_SOLUTION_FOLDERS ON )

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


#######################################################
#                     Find Eigen                      #
#######################################################
if (Eigen3_FOUND)
        message( STATUS "Eigen3 is found!" )
        get_target_property(Eigen_DIR Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
        message( STATUS "Eigen_DIR is ${Eigen_DIR}. ")

endif()


#######################################################
#                     Find Boost                      #
#######################################################
set( Boost_Components locale date_time filesystem timer regex thread serialization system program_options )
set( BOOST_ROOT /usr/local/lib/cmake/Boost-1.74.0 )
find_package(Boost CONFIG REQUIRED COMPONENTS ${Boost_Components} PATHS /usr/local/lib/cmake/Boost-1.74.0 NO_DEFAULT_PATH NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_ENVIRONMENT_PATH )
if(Boost_NOTFOUND)
    message(FATAL_ERROR "Boost is not found! Timing is not disabled! ")
endif()

#######################################################
#                     Find pinocchio                  #
#######################################################
set(hpp-fcl_DIR "/opt/openrobots/lib/cmake/hpp-fcl")
find_package(hpp-fcl REQUIRED)
if(hpp-fcl_FOUND)
message(STATUS "hpp-fcl is found!")
else()
message(SEND_ERROR "hpp-fcl is not found!")
endif()

set(eigenpy_DIR "/opt/openrobots/lib/cmake/eigenpy")
find_package(eigenpy REQUIRED)
if(eigenpy_FOUND)
message(STATUS "eigenpy is found!")
else()
message(SEND_ERROR "eigenpy is not found!")
endif()

set(pinocchio_DIR "/opt/openrobots/lib/cmake/pinocchio")
find_package(pinocchio REQUIRED)
if(pinocchio_FOUND)
message(STATUS "pinocchio is found!")
else()
message(STATUS "pinocchio is not found!")
endif()

############################################################
#   Find mujoco
############################################################
find_package(mujoco)

############################################################
#   Find glfw3
############################################################
find_package(glfw3 REQUIRED)

if (glfw3_FOUND)
  message(STATUS "Glfw found")

# 判断 glfw3 下的glfw是否被找到
  if( TARGET glfw )
      message( STATUS "TARGET glfw is found!" )
  else()
      message( STATUS "TARGET glfw is NOT FOUND!" )
  endif()
endif (glfw3_FOUND)


############################################################
#   Include sub project directories
############################################################

add_subdirectory( TestDemo )
add_subdirectory( TestBall )
add_subdirectory( TestPendulum )
add_subdirectory( TestPanda )
add_subdirectory( TestDoublePendulums )
add_subdirectory( Force_Control )
add_subdirectory( pdf )
add_subdirectory( gmr )

