cmake_minimum_required(VERSION 2.8.3)
project(lima)

find_package(catkin REQUIRED COMPONENTS cv_bridge roscpp tf image_transport sensor_msgs)
find_package(VISP REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS cv_bridge roscpp tf image_transport sensor_msgs 
#  DEPENDS VISP
)

add_definitions(${VISP_DEFINITIONS})
link_directories(${VISP_LIBRARY_DIRS})
include_directories(include ${VISP_INCLUDE_DIRS})

add_library(${PROJECT_NAME} 
src/Arm.cpp
src/BBoxDynamicModel.cpp
src/BBoxEdgesModel.cpp
src/BBoxModel.cpp
src/BodyDynamicModel.cpp
src/ButtonAction.cpp
src/Button.cpp
src/ButtonPush.cpp
src/C1394Image.cpp
src/Camera.cpp
src/CameraObjectPose.cpp
src/CylinderModel.cpp
src/EdgesModel.cpp
src/CAOEdgesModel.cpp
src/FixedHandleAction.cpp
src/FixedHandle.cpp
src/FixedHandlePush.cpp
src/FJoint.cpp
src/FJointDynamicModel.cpp
src/ForceSensor.cpp
src/GeomModel.cpp
src/Hand.cpp
src/HandFrame.cpp
src/HandPreshape.cpp
src/Joint.cpp
src/JointDynamicModel.cpp
src/ObjectAction.cpp
src/ObjectClass.cpp
src/Object.cpp
src/ObjectFrame.cpp
src/ObjectModel.cpp
src/ODEGLScene.cpp
src/ODEGLVirtualCamera.cpp
src/PerfectHand.cpp
src/PerfectHandCylindrical.cpp
src/PerfectHandFingertip.cpp
src/PerfectHandFingertipToThumb.cpp
src/PerfectHandFrame.cpp
src/PerfectHandHook.cpp
src/PerfectHandLateral.cpp
src/PerfectHandMPhalanx.cpp
src/PerfectHandOneFinger.cpp
src/PerfectHandOPhalanx.cpp
src/PerfectHandPalm.cpp
src/PerfectHandPreshape.cpp
src/PerfectHandTOG.cpp
src/PoseEstimator.cpp
src/RawModel.cpp
src/RJoint2.cpp
src/RJoint2DynamicModel.cpp
src/RJoint.cpp
src/RJointDynamicModel.cpp
src/RollerAction.cpp
src/Roller.cpp
src/RollerTurn.cpp
src/Scene.cpp
src/Simulated8Armar.cpp
src/SimulatedArm.cpp
src/SimulatedBarrettHand.cpp
src/SimulatedForceSensor.cpp
src/SimulatedHand.cpp
src/SimulatedPA10.cpp
src/Task.cpp
src/TaskFrame.cpp
src/TJoint.cpp
src/TJointDynamicModel.cpp
src/TOG.cpp
src/TOGPrimitive.cpp
src/VirtualCamera.cpp
src/vpFFMpegGrabber.cpp
src/ROSGrabber.cpp
src/VVSPoseEstimator.cpp
src/tfHomogeneousMatrix.cpp
)

install(TARGETS ${PROJECT_NAME} 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

