
get_filename_component(SELF_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
get_filename_component(maoris_INCLUDE_DIRS "${SELF_DIR}/../../include/maoris" ABSOLUTE)
set(maoris_LIBRARIES "${SELF_DIR}/libMAORIS.so")
