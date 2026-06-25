// Umbrella header for the EtherNetIP originator encoding layer.
//
// This component is a pure, freestanding C++17 implementation of the subset of
// EtherNet/IP + CIP needed to drive the project's servo drives as an
// originator/scanner. It has NO ESP-IDF dependency and is exercised entirely by
// host unit tests (test/host/test_eip_encoding.cpp).
//
// The live UDP/TCP transport and the motion adapter that consume these encoders
// are intentionally deferred to later phases - see docs/EIP_MIGRATION.md.

#ifndef ETHERNET_IP_ETHERNET_IP_H
#define ETHERNET_IP_ETHERNET_IP_H

#include "CipMessageRouter.h"
#include "EipByteBuffer.h"
#include "EipConnectionManager.h"
#include "EipCpf.h"
#include "EipEncapsulation.h"
#include "Hcs01Assembly.h"
#include "Hcs01ControlStatus.h"
#include "Kinetix5100Assembly.h"

#endif  // ETHERNET_IP_ETHERNET_IP_H
