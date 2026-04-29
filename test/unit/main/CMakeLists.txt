idf_component_register(
    SRCS
        "test_door_fsm.c"
        "../../../main/fsm/door_fsm.c"
        "../../../main/nvs/fault_nvs.c"
    INCLUDE_DIRS
        "../../../include"
        "../../../main"
    REQUIRES
        unity
        freertos
        nvs_flash
        esp_timer
        esp_system
)

target_compile_definitions(${COMPONENT_LIB} PRIVATE FSM_UNIT_TEST)