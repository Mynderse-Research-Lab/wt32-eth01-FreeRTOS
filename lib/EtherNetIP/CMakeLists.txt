# EtherNetIP component - pure EtherNet/IP + CIP originator encoding layer.
#
# This is hardware-independent, freestanding C++17 (no ESP-IDF / FreeRTOS
# includes). Today it is exercised only by the host unit tests in test/host/;
# the firmware does NOT yet link it (see docs/EIP_MIGRATION.md, Phase 2/3).
#
# To wire it into the firmware later:
#   1. Add this dir to EXTRA_COMPONENT_DIRS in idf/CMakeLists.txt.
#   2. Add `EtherNetIP` to the REQUIRES of the component that uses it.
#
# Public headers live in src/. Consumers should REQUIRES EtherNetIP and
# #include "EtherNetIP.h".

idf_component_register(
    SRCS
        "src/EipEncapsulation.cpp"
        "src/EipCpf.cpp"
        "src/CipMessageRouter.cpp"
        "src/EipConnectionManager.cpp"
        "src/Kinetix5100Assembly.cpp"
        "src/Hcs01ControlStatus.cpp"
        "src/Hcs01Assembly.cpp"
    INCLUDE_DIRS
        "src"
)
