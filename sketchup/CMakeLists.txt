file(GLOB COMMON_SRC "*.cpp" "*.hpp")

include_directories($${CMAKE_CURRENT_LIST_DIR} ../ext)
add_definitions(-DGSLAPI_NO_EXPORTS)
add_library(sketchup_wrapper SHARED ${COMMON_SRC} ${MODEL_SRC})
if (APPLE)
    target_link_libraries(sketchup_wrapper ${sketchup_api})
else()
    target_link_libraries(sketchup_wrapper sketchup sketchup_api)
endif ()
