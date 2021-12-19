cmake_minimum_required(VERSION 3.15)

project(sfm)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Detect clang. Not officially reported by cmake.
execute_process(COMMAND "${CMAKE_CXX_COMPILER}" "-v" ERROR_VARIABLE CXX_VER_STDERR)
if ("${CXX_VER_STDERR}" MATCHES ".*clang.*")
    set (CMAKE_COMPILER_IS_CLANGXX 1)
endif ()

if (CMAKE_COMPILER_IS_CLANGXX)
    message("Compiling using clang")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    message("Compiling using msvc")
    set(warnings "/W4 /WX /EHsc")
else()
    message( FATAL_ERROR "This project should only be compiled with clang or msvc. Currently detected ${CMAKE_CXX_COMPILER_ID}." )
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

message(" ")
if (CMAKE_BUILD_TYPE MATCHES Debug)
    message("DEBUG BUILD")
    add_definitions(
            -DDEBUG
    )
elseif(CMAKE_BUILD_TYPE MATCHES Release)
    message("RELEASE BUILD")
else()
    message( FATAL_ERROR "Build type not recognised: ${CMAKE_BUILD_TYPE}." )
endif ()
message(" ")

# Create sfm executable target
add_executable(
        sfm
        src/triangulator.cpp headers/triangulator.hpp headers/point_cloud.hpp headers/pose.h headers/constants.h headers/matching.hpp tests/feature_matcher_tests.cpp src/sfm_utilities.cpp headers/sfm_utilities.hpp)

target_sources(
        sfm
        PUBLIC
        headers/feature_matcher.hpp
        headers/image.hpp
        src/bundle_adjuster.cpp
        src/core.cpp
        src/feature_matcher.cpp
)

target_include_directories(
        sfm
        PUBLIC
        ${CMAKE_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/headers
        /Users/meganfinch/opencv-4.5.3/include
        /Users/meganfinch/opencv-4.5.3/cmake-build-debug
        /Users/meganfinch/opencv-4.5.3/modules/calib3d/include
        /Users/meganfinch/opencv-4.5.3/modules/core/include
        /Users/meganfinch/opencv-4.5.3/modules/features2d/include
        /Users/meganfinch/opencv-4.5.3/modules/highgui/include
        /Users/meganfinch/opencv-4.5.3/modules/flann/include
        /Users/meganfinch/opencv-4.5.3/modules/imgcodecs/include
        /Users/meganfinch/opencv-4.5.3/modules/videoio/include
        # /Users/meganfinch/opencv_contrib/modules/sfm/include
)

set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

if (CMAKE_COMPILER_IS_CLANGXX)
    #target_compile_options(sfm PRIVATE "-Werror")
    target_compile_options(sfm PRIVATE "-Weverything")
    target_compile_options(sfm PRIVATE "-Wno-c99-extensions")
    target_compile_options(sfm PRIVATE "-Wno-c++98-compat")
    target_compile_options(sfm PRIVATE "-Wno-c++98-compat-pedantic")
    target_compile_options(sfm PRIVATE "-Wno-padded")

endif()

target_link_directories(
        sfm
        PUBLIC
        "${CMAKE_SOURCE_DIR}"
        #"/Users/meganfinch/opencv-4.5.3/cmake-build-release/lib"
)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

target_link_libraries(
        sfm
        Threads::Threads
        #opencv_calib3d
        #opencv_core
        #opencv_dnn
        #opencv_features2d
        #opencv_flann
        #opencv_gapi
        #opencv_highgui
        #opencv_imgcodecs
        #opencv_imgproc
        #opencv_ml
        #opencv_objdetect
        #opencv_photo
        #opencv_stitching
        #opencv_videoio
        #opencv_video
        -I/opt/homebrew/opt/opencv/include/opencv4 -L/opt/homebrew/opt/opencv/lib -lopencv_gapi -lopencv_stitching -lopencv_alphamat -lopencv_aruco -lopencv_barcode -lopencv_bgsegm -lopencv_bioinspired -lopencv_ccalib -lopencv_dnn_objdetect -lopencv_dnn_superres -lopencv_dpm -lopencv_face -lopencv_freetype -lopencv_fuzzy -lopencv_hfs -lopencv_img_hash -lopencv_intensity_transform -lopencv_line_descriptor -lopencv_mcc -lopencv_quality -lopencv_rapid -lopencv_reg -lopencv_rgbd -lopencv_saliency -lopencv_sfm -lopencv_stereo -lopencv_structured_light -lopencv_phase_unwrapping -lopencv_superres -lopencv_optflow -lopencv_surface_matching -lopencv_tracking -lopencv_highgui -lopencv_datasets -lopencv_text -lopencv_plot -lopencv_videostab -lopencv_videoio -lopencv_viz -lopencv_wechat_qrcode -lopencv_xfeatures2d -lopencv_shape -lopencv_ml -lopencv_ximgproc -lopencv_video -lopencv_dnn -lopencv_xobjdetect -lopencv_objdetect -lopencv_calib3d -lopencv_imgcodecs -lopencv_features2d -lopencv_flann -lopencv_xphoto -lopencv_photo -lopencv_imgproc -lopencv_core
        Eigen3::Eigen
)

# Set up unit testing
option(ENABLE_UNIT_TESTS "Enable unit testing" ON)
message(STATUS "Enable testing: ${ENABLE_UNIT_TESTS}")

if (ENABLE_UNIT_TESTS)
    include(FetchContent)

    FetchContent_Declare(
            googletest
            GIT_REPOSITORY "https://github.com/google/googletest"
            GIT_TAG release-1.10.0
    )

    FetchContent_MakeAvailable(googletest)

    enable_testing()
endif()