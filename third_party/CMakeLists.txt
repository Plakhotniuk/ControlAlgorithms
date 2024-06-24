if (NOT TARGET gtest)
    add_subdirectory(googletest)
endif ()

if (NOT TARGET eigen)
    add_subdirectory(eigen)
endif ()
