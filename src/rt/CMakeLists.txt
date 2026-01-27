find_program(XENO_CONFIG xeno-config REQUIRED)

execute_process(
    COMMAND ${XENO_CONFIG} --skin=posix --cflags
    OUTPUT_VARIABLE XENO_CFLAGS
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

execute_process(
    COMMAND ${XENO_CONFIG} --skin=posix --ldflags
    OUTPUT_VARIABLE XENO_LDFLAGS
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

separate_arguments(XENO_CFLAGS)
separate_arguments(XENO_LDFLAGS)

add_executable(rt_trajectory_generator
    main.cpp
    rt_sockets.cpp
    client.cpp
    server.cpp
)

target_compile_options(rt_trajectory_generator PRIVATE ${XENO_CFLAGS})
target_link_libraries(rt_trajectory_generator PRIVATE ${XENO_LDFLAGS})

install(TARGETS rt_trajectory_generator
    RUNTIME DESTINATION bin
)
