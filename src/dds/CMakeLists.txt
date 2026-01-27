find_package(fastcdr 2 REQUIRED)
find_package(fastdds 3 REQUIRED)

file(GLOB_RECURSE DDS_GENERATED_SOURCES 
    "${CMAKE_SOURCE_DIR}/generated/idl/**/msg/*.cxx"
)

add_executable(JointStatePublisher
    JointStatePublisher.cpp
    ${DDS_GENERATED_SOURCES}
)
target_link_libraries(JointStatePublisher
    fastdds
    fastcdr
)

add_executable(JointStateSubscriber
    JointStateSubscriber.cpp
    ${DDS_GENERATED_SOURCES}
)
target_link_libraries(JointStateSubscriber
    fastdds
    fastcdr
)

install(TARGETS JointStatePublisher JointStateSubscriber
    RUNTIME DESTINATION bin
)
