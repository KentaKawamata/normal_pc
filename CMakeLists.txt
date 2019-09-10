cmake_minimum_required(VERSION 3.12)

set(SOURCE_NAME
        src/main.cpp)

set(USE_Boost 1 "Find and link Boost library")
set(USE_PCL 1 "Find and link Point Cloud Library")

#project : プログラム全体の構成単位
#基本的にプログラム全体を格納しているディレクトリの名前をプロジェクト名とする
if(NOT PROJECT_NAME)
    get_filename_component(DEFAULT_PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME_WE)
    set(PROJECT_NAME ${DEFAULT_PROJECT_NAME} CACHE STRING "Project name" FORCE)

endif(NOT PROJECT_NAME)
project(${PROJECT_NAME} C CXX)

set(CMAKE_CXX_STANDARD 14)
set(CXX_SOURCE_FILE_EXTS cpp;cxx)
set(CXX_INCLUDE_FILE_EXTS hpp;h)
set(CMAKE_C_COMPILER gcc)
set(CMAKE_CXX_COMPILER g++)

#ビルドの対象と実行ファイル名
add_executable(${PROJECT_NAME} ${SOURCE_NAME})

#Boostライブラリを探す
if(USE_Boost)
    #必要なライブラリの場合はREQUIREDを付ける
    find_package(Boost REQUIRED)

    if(Boost_FOUND)
        message("---- Found Boost ${Boost_INCLUDE_DIRS}")
        set(INCLUDE_DIRECTORIES ${INCLUDE_DIRECTORIES} ${Boost_INCLUDE_DIR})

    else(Boost_FOUND)
        message(FATAL_ERROR " ---- Not Found Boost")

    endif(Boost_FOUND)

    set(ALL_LINK_LIBRARIES ${ALL_LINK_LIBRARIES} ${Boost_LIBRARIES} Boost::thread)

endif(USE_Boost)

if(USE_PCL)
    find_package(PCL 1.2 REQUIRED)

    if(PCL_FOUND)
        message("---- Found PCL ${PCL_INCLUDE_DIRS}")
        #set(INCLUDE_DIRECTORIES ${INCLUDE_DIRECTORIES} ${PCL_INCLUDE_DIRS})
        include_directories(${PCL_INCLUDE_DIRS})
        link_directories(${PCL_LIBRARY_DIRS})
        add_definitions(${PCL_DEFINITIONS})

    else(PCL_FOUND)
        message(FATAL_ERROR "---- Not Found PCL")

    endif(PCL_FOUND)

    set(ALL_LINK_LIBRARIES ${ALL_LINK_LIBRARIES} ${PCL_LIBRARIES})
    target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

endif(USE_PCL)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} -lpthread -lm)
