cmake_minimum_required(VERSION 3.10)

# 设置目标名称
set( TARGET_NAME "pdf")
# 把.cpp和.c文件都塞到SOURCE_FILES里
file(GLOB SOURCE_FILES "*.cpp" "*.c" )
# 把.hpp和.h文件都塞到HEADER_FILES里
file(GLOB HEADER_FILES "*.h" "*.hpp" )

# 添加库,说明这是一个库
add_library( ${TARGET_NAME} STATIC ${SOURCE_FILES} ${HEADER_FILES} )

# 链接其他的库
target_link_libraries( ${TARGET_NAME} PUBLIC Eigen3::Eigen)

# 把此目录添加进项目
target_include_directories( ${TARGET_NAME} PUBLIC "." )

