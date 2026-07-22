// Umbrella header for the EtherNetIP originator encoding + transport layer.
//
// Pure C++17 encoders (EipEncapsulation, EipCpf, CIP MR, assemblies) compile on
// the host and are regression-tested in test/host/. ESP-IDF-only pieces
// (EipSocketEspIdf, EipScannerTask) live alongside them but are not included
// here — see docs/LOW_LEVEL_GANTRY_CONTROL.md for phase boundaries.

#ifndef ETHERNET_IP_ETHERNET_IP_H
#define ETHERNET_IP_ETHERNET_IP_H

#include "CipMessageRouter.h"
#include "EipByteBuffer.h"
#include "EipConnectionManager.h"
#include "EipCpf.h"
#include "EipEncapsulation.h"
#include "EipIoConnection.h"
#include "EipProcessImage.h"
#include "EipMultiScanner.h"
#include "EipScanner.h"
#include "EipSession.h"
#include "EipTransport.h"
#include "Hcs01Assembly.h"
#include "Hcs01ControlStatus.h"
#include "Kinetix5100Assembly.h"

#endif  // ETHERNET_IP_ETHERNET_IP_H
