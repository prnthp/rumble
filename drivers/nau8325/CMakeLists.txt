if(CONFIG_NAU8325)
    zephyr_include_directories(include)

    zephyr_library()
    zephyr_library_sources(
        src/nau8325.c
    )
endif()
