add_executable(
    goap_test 
    src/goap_test.cpp
)

target_link_libraries(
    goap_test
    PRIVATE
        goap
)

add_test(
    NAME goap_test
    COMMAND goap_test
)