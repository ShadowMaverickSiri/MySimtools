# ============================================================
# SimTools v2.0 - CMake 构建配置
# ============================================================

cmake_minimum_required(VERSION 3.10)
project(SimTools VERSION 2.0.0 LANGUAGES CXX)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 编译选项
if(MSVC)
    add_compile_options(/W4 /WX)
else()
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# ============================================================
# 查找依赖库 - Eigen3（可选）
# ============================================================

# 检查环境变量 EIGEN_ROOT
if(NOT DEFINED EIGEN_ROOT)
    set(EIGEN_ROOT "$ENV{EIGEN_ROOT}")
    if(EIGEN_ROOT)
        message(STATUS "Found EIGEN_ROOT environment variable: ${EIGEN_ROOT}")
    endif()
endif()

# 尝试查找 Eigen3（非必需）
find_package(Eigen3 3.3 QUIET NO_MODULE)

if(EIGEN3_FOUND)
    message(STATUS "Eigen3 found: ${EIGEN3_INCLUDE_DIR}")
    set(USE_EIGEN ON)
else()
    message(STATUS "Eigen3 not found, using C++ standard library implementation")
    set(USE_EIGEN OFF)

    # 如果设置了 EIGEN_ROOT，手动添加包含路径
    if(EIGEN_ROOT AND EXISTS "${EIGEN_ROOT}")
        if(EXISTS "${EIGEN_ROOT}/Eigen/Dense")
            set(EIGEN3_INCLUDE_DIR "${EIGEN_ROOT}")
            set(USE_EIGEN ON)
            message(STATUS "Using Eigen3 from EIGEN_ROOT: ${EIGEN_ROOT}")
        else()
            message(WARNING "EIGEN_ROOT is set but Eigen/Dense not found at ${EIGEN_ROOT}")
        endif()
    endif()
endif()

# ============================================================
# 库目标
# ============================================================

# 收集所有源文件
set(SIMTOOLS_SOURCES
    SimTools_Interpolation.cpp
    SimTools_Coordinate.cpp
    SimTools_Geodesy.cpp
    SimTools_Atmosphere.cpp
    SimTools_Random.cpp
    SimTools_FileIO.cpp
    SimTools_Numerical.cpp
    SimTools_Geometry.cpp
    SimTools_MatrixUtils.cpp
    SimTools_Time.cpp
    SimTools_Simulation.cpp
)

# 创建静态库（默认）
add_library(SimTools_static STATIC ${SIMTOOLS_SOURCES})
target_include_directories(SimTools_static PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_compile_features(SimTools_static PUBLIC cxx_std_14)

# 如果使用 Eigen，添加包含目录和定义
if(USE_EIGEN)
    target_include_directories(SimTools_static PUBLIC ${EIGEN3_INCLUDE_DIR})
    target_compile_definitions(SimTools_static PRIVATE USE_EIGEN)
    message(STATUS "SimTools will use Eigen3 library")
else()
    message(STATUS "SimTools will use C++ standard library")
endif()

# 创建动态库（可选）
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)
if(BUILD_SHARED_LIBS)
    add_library(SimTools_shared SHARED ${SIMTOOLS_SOURCES})
    target_include_directories(SimTools_shared PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
    target_compile_features(SimTools_shared PUBLIC cxx_std_14)
    target_compile_definitions(SimTools_shared PRIVATE SIMTOOLS_EXPORTS)

    if(USE_EIGEN)
        target_include_directories(SimTools_shared PUBLIC ${EIGEN3_INCLUDE_DIR})
        target_compile_definitions(SimTools_shared PRIVATE USE_EIGEN)
    endif()
endif()

# 别名：默认使用静态库
add_library(SimTools ALIAS SimTools_static)

# ============================================================
# 示例程序
# ============================================================

option(BUILD_EXAMPLES "Build example programs" ON)

if(BUILD_EXAMPLES)
    # 基础示例
    add_executable(SimTools_example SimTools_v2_examples.cpp)
    target_link_libraries(SimTools_example PRIVATE SimTools_static)

    # 单元测试示例
    add_executable(SimTools_test SimTools_test.cpp)
    target_link_libraries(SimTools_test PRIVATE SimTools_static)
endif()

# ============================================================
# 安装配置
# ============================================================

include(GNUInstallDirs)

install(TARGETS SimTools_static
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(FILES SimTools_v2.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# ============================================================
# CPack 配置（用于打包发布）
# ============================================================

set(CPACK_PACKAGE_NAME "SimTools")
set(CPACK_PACKAGE_VERSION "2.0.0")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Simulation Tools Library")
set(CPACK_PACKAGE_VENDOR "SimTools Development Team")
set(CPACK_PACKAGE_CONTACT "your-email@example.com")

include(CPack)
