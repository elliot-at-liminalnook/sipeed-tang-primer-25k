# link_proto.c is pulled in from the parent directory rather than copied.
# The same file compiles into this firmware and into the Verilator harness
# in servo/sim, so the wire format has exactly one implementation on the C
# side and cannot drift between them.
idf_component_register(
    SRCS "main.c" "../../link_proto.c"
    INCLUDE_DIRS "." "../.."
    REQUIRES esp_wifi esp_netif nvs_flash driver lwip
)
