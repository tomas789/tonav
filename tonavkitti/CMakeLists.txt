set (TONAVKITTI_SRC_FILES
        src/main.cpp
        src/tonav_kitti.cpp
)

set (TONAVKITTI_HEADER_FILES
        include/tonav_kitti.h
)
add_executable(tonavkitti ${TONAVKITTI_SRC_FILES} ${TONAVKITTI_HEADER_FILES})
target_include_directories (tonavkitti PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries (tonavkitti LINK_PUBLIC tonav)

set_target_properties(tonavkitti PROPERTIES
        CXX_STANDARD 11
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS OFF
)

find_package(OpenCV REQUIRED core viz)
target_include_directories (tonavkitti PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(tonavkitti LINK_PUBLIC ${OpenCV_LIBS})
