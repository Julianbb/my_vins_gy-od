cmake_minimum_required(VERSION 2.8.3)
project(vins_estimator)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")


set(Boost_USE_STATIC_LIBS ON)
find_package(Boost REQUIRED COMPONENTS system filesystem program_options)
set(OpenCV_DIR /usr/local/share/OpenCV)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

message("OpenCV_VERSION "${OpenCV_VERSION})
message("Ceres_VERSION "${Ceres_VERSION})
message("Boost_VERSION "${Boost_VERSION})

include_directories(
    "/usr/local/include/eigen3"
    "include"
    "camera_model/include"
    ${OpenCV_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}

)



add_executable(vins_estimator
        src/estimator_node.cpp
		src/parameter.cpp
        
        src/feature_tracker/feature_tracker.cpp

        camera_model/src/chessboard/Chessboard.cc
		camera_model/src/calib/CameraCalibration.cc
		camera_model/src/camera_models/Camera.cc
		camera_model/src/camera_models/CameraFactory.cc
		camera_model/src/camera_models/CostFunctionFactory.cc
		camera_model/src/camera_models/PinholeCamera.cc
		camera_model/src/camera_models/CataCamera.cc
		camera_model/src/camera_models/EquidistantCamera.cc
		camera_model/src/camera_models/ScaramuzzaCamera.cc
		camera_model/src/sparse_graph/Transform.cc
		camera_model/src/gpl/gpl.cc
		camera_model/src/gpl/EigenQuaternionParameterization.cc

)

    target_link_libraries(vins_estimator 
        ${Boost_LIBRARIES} 
        ${OpenCV_LIBRARIES} 
        ${CERES_LIBRARIES}
        )