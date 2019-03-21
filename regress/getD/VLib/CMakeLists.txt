include_directories(include)
aux_source_directory(common DIR_V_SRCS)

add_library(vLib ${DIR_V_SRCS})
