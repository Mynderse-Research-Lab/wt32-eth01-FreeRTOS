# WT32-ETH01 gantry controller

Firmware for the gantry of the battery sorting cell. An ESP-IDF application on a
WT32-ETH01 moves a three-axis gantry (Kinetix 5100 X and Z, Rexroth HCS01 theta)
over EtherNet/IP and picks batteries off the moving conveyor. It talks to the
vision node, the conveyor and the supervisor over CellNet Layer 2 on the cell
switch.

**Contact:** the Gantry team, Mynderse Research Lab.

## Read next

1.  [Knowledge transfer](docs/KNOWLEDGE_TRANSFER.md): the cell, its subsystems,
    how they connect, and the working rules.
2.  [Gantry firmware](GANTRY_FIRMWARE.md): how this firmware works: what runs,
    boot, tasks, one move and one pick end to end.
3.  [Libraries](lib/README.md): the thirteen components by layer; each has a
    README and a REFERENCE.
4.  [Contributing](CONTRIBUTING.md): build, tests and engineering rules.

Also: the [open items](docs/OPEN_ITEMS.md) and the [doc index](docs/INDEX.md).

## Quick start

```powershell
cmake -S test/host -B build/host
cmake --build build/host
ctest --test-dir build/host --output-on-failure
idf.py -C idf build
```

The console is in [Gantry firmware §5](GANTRY_FIRMWARE.md#5-console-contract).
Flashing over Ethernet is in the
[GantryOta reference](lib/GantryOta/REFERENCE.md#partitions-and-flashing), and
the programming wiring in the
[hardware doc](docs/EXPECTED_ELECTROMECHANICAL_ASSEMBLY.md#entering-programming-mode).
