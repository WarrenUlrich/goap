add_executable(
    increment_test 
    src/increment_test.cpp
)

target_link_libraries(
    increment_test
    PRIVATE
        goap
)

add_test(
    NAME increment_test
    COMMAND increment_test
)