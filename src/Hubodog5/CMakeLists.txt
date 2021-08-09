cmake_minimum_required(VERSION 2.8.3)
project(Hubodog5)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#-DEIGEN_USE_MKL_ALL")
#set(CMAKE_CXX_FLAGS_RELEASE "-O2 -Wall -g")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -flto -march=native")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Eigen3)
#find_package(raisim CONFIG REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
)

# qmake for PODO
set(TARGET ${CMAKE_PROJECT_NAME})

find_package(OpenGL)
find_package(Qt5 COMPONENTS Core Widgets Gui OpenGL Sql PrintSupport REQUIRED)


set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(QT5_LIBRARIES Qt5::Widgets Qt5::OpenGL Qt5::Sql Qt5::Gui Qt5::PrintSupport)



set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_GNU_SOURCE -D_REENTRANT -D__COBALT__")
set(XENO_DIR /usr/xenomai)
set(XENO_CONFIG ${XENO_DIR}/bin/xeno-config)
execute_process(COMMAND ${XENO_CONFIG} --skin=alchemy --cflags OUTPUT_VARIABLE XENO_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND ${XENO_CONFIG} --skin=alchemy --ldflags OUTPUT_VARIABLE XENO_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${XENO_CFLAGS}")
string(STRIP ${CMAKE_CXX_FLAGS} CMAKE_CXX_FLAGS)

SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${XENO_LDFLAGS}")


link_directories(
        /usr/xenomai/lib
        /usr/lib
        /usr/local/lib
)

#

set(PODO_LIBS
        alchemy
        copperplate
        cobalt
        pthread
        rt
        rbdl
#        rbdl_urdfreader #rbdl urdf test
        )








include_directories(
        ${Qt5Core_INCLUDE_DIRS}
        ${Qt5Widgets_INCLUDE_DIRS}
        ${Qt5Gui_INCLUDE_DIRS}
        ${Qt5OpenGL_INCLUDE_DIRS}
        ${Qt5Sql_INCLUDE_DIRS}
        ${Qt5PrintSupport_INCLUDE_DIRS}
)




LINK_LIBRARIES(
        ${Qt5Core_LIBRARIES}
        ${Qt5Widgets_LIBRARIES}
        ${Qt5Gui_LIBRARIES}
        ${Qt5OpenGL_LIBRARIES}
        ${Qt5Sql_LIBRARIES}
        ${Qt5PrintSupport_LIBRARIES}
)

include_directories(
        /usr/xenomai/include/cobalt
        /usr/xenomai/include
        /usr/xenomai/include/alchemy
        /usr/local/include/eigen3
        src/RBModel
        ${CMAKE_CURRENT_SOURCE_DIR}
        src
        src/gui
        src/Qtplot
        src/Controller
        src/qpSWIFT/include
        src/qpSWIFT/amd/include
        src/qpSWIFT/ldl/include
)

set(SOURCES
        src/main.cpp
        src/RBCAN.cpp
        src/RBMotorController.cpp
        src/RBDaemon.cpp
        src/RBLANComm.cpp
        src/NETWORK_UI.cpp
        src/RBJoystick.cpp
        src/gui/mainwindow.cpp
        src/gui/ModelDialog.cpp
        src/RBModel/isnl_opengl_globject.cpp
        src/RBModel/isnl_opengl_globject_STL.cpp
        src/RBModel/isnl_opengl_glskeleton.cpp
        src/RBModel/isnl_util_reqserver.cpp
        src/RBModel/isnl_util_skeleton.cpp
        src/RBModel/tinythread.cpp
        src/Qtplot/qcustomplot.cpp
        src/RBIMUSensor.cpp
        src/RBMotion.cpp
        src/Controller/main_controller.cpp
        src/Controller/hubodog5_general.cpp
        src/Controller/state_estimator.cpp
        src/Controller/func_sensor.cpp
        src/Controller/func_general.cpp
        src/Controller/func_uiuk.cpp
        src/Controller/func_kinematics.cpp
        #src/Controller/MPC_calc.cpp
        #src/Controller/func_mpc.cpp

        # qpSWIFT part
        #src/qpSWIFT/src/Auxilary.c
        #src/qpSWIFT/src/Prime.c
        #src/qpSWIFT/src/RUNQP.c
        #src/qpSWIFT/src/timer.c
        #src/qpSWIFT/src/ldl.c
        #src/qpSWIFT/amd/src/amd_1.c
        #src/qpSWIFT/amd/src/amd_2.c
        #src/qpSWIFT/amd/src/amd_aat.c
        #src/qpSWIFT/amd/src/amd_control.c
        #src/qpSWIFT/amd/src/amd_defaults.c
        #src/qpSWIFT/amd/src/amd_dump.c
        #src/qpSWIFT/amd/src/amd_global.c
        #src/qpSWIFT/amd/src/amd_info.c
        #src/qpSWIFT/amd/src/amd_order.c
        #src/qpSWIFT/amd/src/amd_postorder.c
        #src/qpSWIFT/amd/src/amd_post_tree.c
        #src/qpSWIFT/amd/src/amd_preprocess.c
        #src/qpSWIFT/amd/src/amd_valid.c


        # task function part
        #src/Controller/task_demo.cpp
        #src/Controller/task_dance01.cpp
        #src/Controller/task_onelegtest.cpp
        #src/Controller/task_MPC.cpp
        src/Controller/task_trot_test.cpp
        src/Controller/task_hopping_test.cpp
        src/Controller/task_flyingtrot.cpp
        #src/Controller/task_comcontrol_test.cpp
        src/ESTIMATOR/estimator.cpp
        src/ESTIMATOR/kinematics.cpp
        src/ESTIMATOR/operation.cpp
        src/ESTIMATOR/robot_parameters.cpp
        src/Controller/QP_Walking.cpp
        src/ESTIMATOR_MIT/PositionVelocityEstimator.cpp
        )



set(UI_SOURCES
        src/gui/mainwindow.ui
        src/gui/ModelDialog.ui
        )



    qt5_wrap_ui(UI_GENERATED_HEADERS ${UI_SOURCES})





    add_executable(${TARGET} ${SOURCES} ${UI_GENERATED_HEADERS})


    #target_link_libraries(${TARGET} ${PODO_LIBS} ${QT5_LIBRARIES} ${OPENGL_gl_LIBRARY} junny_estimation_podo_lib)
    #target_link_libraries(${TARGET} ${PODO_LIBS} ${QT5_LIBRARIES} ${OPENGL_gl_LIBRARY} ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} ${RAISIM_LIBRARIES} ${PINOCCHIO_LIBRARIES})
    #target_link_libraries(${TARGET} ${PODO_LIBS} ${QT5_LIBRARIES} ${OPENGL_gl_LIBRARY} ${OpenCV_LIBS} ${CERES_LIBRARIES})
    target_link_libraries(${TARGET} ${PODO_LIBS} ${QT5_LIBRARIES} ${OPENGL_gl_LIBRARY} ${OpenCV_LIBS} #raisim::raisim
    #    ${CERES_LIBRARIES}
        )

    #target_link_libraries(${TARGET} ${QT5_LIBRARIES})


