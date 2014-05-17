cmake_minimum_required(VERSION 2.8.3)
project(libartoolkit)

find_package(catkin REQUIRED)
find_package(GLUT REQUIRED)
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES AR ARgsub ARgsub_lite ARgsubUtil ARMulti ARvideo
)

#add_definitions(-Wall -msse2 -msse3)

include_directories(include
  ${catkin_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS})

add_library(AR  
    src/AR/arDetectMarker2.c  
    src/AR/arGetMarkerInfo.c  
    src/AR/arGetTransMat.c      
    src/AR/arUtil.c  
    src/AR/mAllocDup.c  
    src/AR/mAllocTrans.c  
    src/AR/mDisp.c  
    src/AR/mInv.c  
    src/AR/mSelfInv.c  
    src/AR/paramChangeSize.c  
    src/AR/paramDistortion.c  
    src/AR/vAlloc.c  
    src/AR/vHouse.c
    src/AR/arDetectMarker.c   
    src/AR/arGetTransMat2.c   
    src/AR/arGetTransMatCont.c  
    src/AR/mAllocInv.c  
    src/AR/mAllocUnit.c   
    src/AR/mDup.c   
    src/AR/mMul.c  
    src/AR/mTrans.c    
    src/AR/paramDecomp.c      
    src/AR/paramFile.c        
    src/AR/vDisp.c   
    src/AR/vInnerP.c
    src/AR/arGetCode.c        
    src/AR/arGetTransMat3.c   
    src/AR/arLabeling.c         
    src/AR/mAlloc.c  
    src/AR/mAllocMul.c  
    src/AR/mDet.c         
    src/AR/mFree.c  
    src/AR/mPCA.c  
    src/AR/mUnit.c     
    src/AR/paramDisp.c        
    src/AR/paramGet.c         
    src/AR/vFree.c   
    src/AR/vTridiag.c)
add_library(ARMulti 
    src/ARMulti/arMultiActivate.c  
    src/ARMulti/arMultiGetTransMat.c  
    src/ARMulti/arMultiReadConfigFile.c)
add_library(ARgsub 
    src/Gl/gsub.c)
target_link_libraries(ARgsub ${GLUT_LIBRARIES})
add_library(ARgsub_lite
    src/Gl/gsub_lite.c)
add_library(ARgsubUtil 
    src/Gl/gsubUtil.c)
add_library(ARvideo 
    src/VideoLinuxV4L/ccvt_c.c
    src/VideoLinuxV4L/video.c)

