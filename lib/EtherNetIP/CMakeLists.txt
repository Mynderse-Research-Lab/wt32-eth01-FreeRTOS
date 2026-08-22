# EtherNetIP component - EtherNet/IP + CIP originator encoding and transport.
#
# Pure C++17 encoding (host-testable) plus ESP-IDF socket/scanner scaffolding.
# Scanner is Kconfig-gated (default off); see docs/LOW_LEVEL_GANTRY_CONTROL.md.

idf_component_register(
    SRCS
        "src/EipEncapsulation.cpp"
        "src/EipCpf.cpp"
        "src/CipMessageRouter.cpp"
        "src/EipConnectionManager.cpp"
        "src/Kinetix5100Assembly.cpp"
        "src/Kinetix5100ParamAccess.cpp"
        "src/Kinetix5100TuningClient.cpp"
        "src/Hcs01ControlStatus.cpp"
        "src/Hcs01Assembly.cpp"
        "src/Hcs01ComwsClient.cpp"
        "src/EipSession.cpp"
        "src/EipIoConnection.cpp"
        "src/EipScanner.cpp"
        "src/EipMultiScanner.cpp"
        "src/EipProcessImage.cpp"
        "src/EipSocketW5500.cpp"
        "src/EipScannerTask.cpp"
        "src/EipClass1TimingStats.cpp"
        "src/EipReliabilityStats.cpp"
    INCLUDE_DIRS
        "src"
    REQUIRES
        lwip
        esp_netif
        W5500
        Spi3Bus
        freertos
        log
        esp_timer
)
