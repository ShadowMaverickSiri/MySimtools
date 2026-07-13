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
    # MSVC 编译器选项
    add_compile_options(/W4)           # 警告级别 4
    add_compile_options(/utf-8)        # 支持 UTF-8 编码（中文注释）
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================
# 查找依赖库 - Eigen3（必需）
# ============================================================

# 本项目对外统一使用 EIGEN_ROOT，值应为直接包含 Eigen/ 目录的路径。
# 同时兼容 CMake 常用的 Eigen3_ROOT，以及历史环境变量 EIGEN3_ROOT。
set(EIGEN_ROOT "" CACHE PATH "Path containing the Eigen directory")

if(NOT EIGEN_ROOT)
    if(DEFINED Eigen3_ROOT AND Eigen3_ROOT)
        set(EIGEN_ROOT "${Eigen3_ROOT}")
    elseif(DEFINED ENV{EIGEN_ROOT} AND NOT "$ENV{EIGEN_ROOT}" STREQUAL "")
        set(EIGEN_ROOT "$ENV{EIGEN_ROOT}")
    elseif(DEFINED ENV{Eigen3_ROOT} AND NOT "$ENV{Eigen3_ROOT}" STREQUAL "")
        set(EIGEN_ROOT "$ENV{Eigen3_ROOT}")
    elseif(DEFINED ENV{EIGEN3_ROOT} AND NOT "$ENV{EIGEN3_ROOT}" STREQUAL "")
        set(EIGEN_ROOT "$ENV{EIGEN3_ROOT}")
        message(DEPRECATION "EIGEN3_ROOT is deprecated; use EIGEN_ROOT instead")
    endif()
endif()

# 先使用 Eigen 提供的 CMake 包；找不到时再把 EIGEN_ROOT 当作头文件目录。
find_package(Eigen3 3.3 QUIET NO_MODULE)

if(TARGET Eigen3::Eigen)
    message(STATUS "Eigen3 found through its CMake package")
elseif(EIGEN_ROOT AND EXISTS "${EIGEN_ROOT}/Eigen/Dense")
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    set_target_properties(Eigen3::Eigen PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${EIGEN_ROOT}"
    )
    message(STATUS "Using Eigen3 headers from EIGEN_ROOT: ${EIGEN_ROOT}")
else()
    message(FATAL_ERROR
        "Eigen3 3.3 or newer is required. Install Eigen3 or set EIGEN_ROOT "
        "to the directory that directly contains Eigen/Dense."
    )
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

# 收集所有头文件（用于在 IDE 中显示）
set(SIMTOOLS_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/SimTools_v2.h
)

# 创建静态库
add_library(SimTools_static STATIC ${SIMTOOLS_SOURCES})

# 将头文件添加到项目（用于在 IDE 中显示）
target_sources(SimTools_static PUBLIC
    ${SIMTOOLS_HEADERS}
)

target_include_directories(SimTools_static PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_compile_features(SimTools_static PUBLIC cxx_std_14)
target_compile_definitions(SimTools_static PUBLIC SIMTOOLS_STATIC)
target_link_libraries(SimTools_static PUBLIC Eigen3::Eigen)
message(STATUS "SimTools will use Eigen3 library")

# 创建动态库（可选）
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)
if(BUILD_SHARED_LIBS)
    add_library(SimTools_shared SHARED ${SIMTOOLS_SOURCES})
    target_include_directories(SimTools_shared PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
    target_compile_features(SimTools_shared PUBLIC cxx_std_14)
    target_compile_definitions(SimTools_shared PRIVATE SIMTOOLS_EXPORTS)
    target_link_libraries(SimTools_shared PUBLIC Eigen3::Eigen)
endif()

# 别名：默认使用静态库
add_library(SimTools ALIAS SimTools_static)

# ============================================================
# 测试程序
# ============================================================

option(BUILD_TESTING "Build testing program" ON)

if(BUILD_TESTING)
    add_executable(SimTools_test SimTools_test.cpp)
    target_link_libraries(SimTools_test PRIVATE SimTools_static)
    target_compile_definitions(SimTools_test PRIVATE SIMTOOLS_STATIC)
    target_include_directories(SimTools_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

    # 设置为启动项目（Visual Studio）
    set_target_properties(SimTools_test PROPERTIES
        VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    )
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

include(CPack)
