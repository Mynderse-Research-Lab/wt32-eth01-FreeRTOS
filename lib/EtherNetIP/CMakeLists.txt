# EtherNetIP component - EtherNet/IP + CIP originator encoding and transport.
#
# Pure C++17 encoding (host-testable) plus ESP-IDF socket/scanner scaffolding.
# Scanner is Kconfig-gated (default off); see docs/EIP_MIGRATION.md.

idf_component_register(
    SRCS
        "src/EipEncapsulation.cpp"
        "src/EipCpf.cpp"
        "src/CipMessageRouter.cpp"
        "src/EipConnectionManager.cpp"
        "src/Kinetix5100Assembly.cpp"
        "src/Hcs01ControlStatus.cpp"
        "src/Hcs01Assembly.cpp"
        "src/EipSession.cpp"
        "src/EipIoConnection.cpp"
        "src/EipSocketEspIdf.cpp"
        "src/EipScannerTask.cpp"
    INCLUDE_DIRS
        "src"
    REQUIRES
        lwip
        esp_netif
        freertos
        log
)
