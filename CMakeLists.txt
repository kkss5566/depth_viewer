cmake_minimum_required(VERSION 2.8.3)
project(depth_viewer)

## 添加編譯選項，並加入解決 Boost 和 PCL 兼容性的標誌
add_compile_options(-std=c++14 -DBOOST_MPL_CFG_NO_PREPROCESSED_HEADERS -DBOOST_MPL_LIMIT_LIST_SIZE=30)

## 尋找需要的包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)  # 添加 PCL

## 聲明catkin包
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge
)

## 指定頭文件目錄
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}  # 添加 PCL 頭文件目錄
)

## 聲明可執行文件
add_executable(depth_viewer_node src/depth_viewer.cpp)  # 修改名稱，避免衝突
add_executable(viewer_3D_node src/3D_viewer.cpp)

## 指定鏈接庫
target_link_libraries(depth_viewer_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}  # 链接 PCL 库
)

target_link_libraries(viewer_3D_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}  # 链接 PCL 库
)

