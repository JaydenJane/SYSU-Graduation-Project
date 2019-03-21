include_directories(../VLib/include)

aux_source_directory(. DIR_API_SRCS)
add_library(apiLib ${DIR_API_SRCS})

#target_link_libraries(apiLib vLib)
