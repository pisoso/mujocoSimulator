# 需要cmake的最低版本
cmake_minimum_required(VERSION 3.5)

# 设置目标名称
set( TARGET_NAME "TestBall" )

# 把.cpp和.c文件都塞到SOURCE_FILES里
file(GLOB SOURCE_FILES "*.cpp" "*.c" )

# 添加可执行文件
add_executable(${TARGET_NAME} main.cpp)

# 链接其他的库
target_link_libraries(${TARGET_NAME} 
        glfw 
        mujoco
)
