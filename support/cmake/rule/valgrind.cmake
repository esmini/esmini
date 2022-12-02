include_guard()

macro(enable_valgrind)
    find_program(
        MEMORYCHECK_COMMAND
        NAMES valgrind)
    set(MEMORYCHECK_SUPPRESSIONS_FILE
        "${CMAKE_SOURCE_DIR}/support/valgrind/valgrind.supp"
        CACHE FILEPATH
              "File that contains suppressions for the memory checker")
    set(MEMORYCHECK_COMMAND_OPTIONS
        "--leak-check=full --show-leak-kinds=all --error-exitcode=1 --errors-for-leak-kinds=all"
        CACHE STRING
              "valgrind command line options")
endmacro()
