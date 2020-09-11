cmake_minimum_required(VERSION 3.8)
project(omni_ros_gui)

#find_package(OpenCV 4.3.0 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	roslib
	std_msgs
	cv_bridge
	image_transport
#	message_generation
#	omni_ros
)

#add_message_files(
#        FILES
#        m_encoder.msg
#        m_motion_n.msg
#        m_motion.msg
#        m_flags.msg
#)

#generate_messages(
#        DEPENDENCIES
#        std_msgs
#)

catkin_package(
	CATKIN_DEPENDS roscpp rospy std_msgs 
)



include_directories(${catkin_INCLUDE_DIRS})
#include_directories(${OpenCV_INCLUDE_DIRS})

#add_executable(view_img_raw src/view_img_raw.cpp)
#add_dependencies(view_img_raw ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(view_img_raw
#	${catkin_LIBRARIES}
#	${OpenCV_LIBRARIES}
#)

#add_executable(image_converter src/image_converter.cpp)
#add_dependencies(image_converter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(image_converter
#	${catkin_LIBRARIES}
#	${OpenCV_LIBRARIES}
#)

#Python scripts

catkin_install_python(PROGRAMS scripts/manual_ctrl.py 
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#catkin_install_python(PROGRAMS scripts/modelo_python_escuta.py
#	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#catkin_install_python(PROGRAMS scripts/img_view_py.py
#	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
