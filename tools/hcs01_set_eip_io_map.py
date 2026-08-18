#!/usr/bin/env python3
"""Load the 18/14 EtherNet/IP cyclic map on a live HCS01 via Service Tool COMWS.

Does not touch CIP/FKM IP (P-0-4089.0.13). Uses engineering HTTP on .22.
"""

from __future__ import annotations

import sys

from hcs01_comws import ComwsError, Hcs01Comws

CMD_MDT = [
    "P-0-4077.0.0",
    "S-0-0282.0.0",
    "S-0-0259.0.0",
    "S-0-0260.0.0",
    "S-0-0359.0.0",
]
ACT_AT = [
    "P-0-4078.0.0",
    "S-0-0051.0.0",
    "S-0-0040.0.0",
    "S-0-0390.0.0",
]

# Module-level aliases for ad-hoc snippets that `from hcs01_set_eip_io_map import …`.
_cli = Hcs01Comws()
BASE = _cli.base
LOGIN = _cli.login_url


def http_get(url: str, timeout: float = 15.0) -> str:
    return _cli.http_get(url, timeout=timeout)


def http_post(url: str, data: str, timeout: float = 15.0) -> str:
    return _cli.http_post(url, data, timeout=timeout)


def getvar(idn: str) -> str:
    return _cli.getvar(idn)


def getlst(idn: str, count: int = 16) -> str:
    return _cli.getlst(idn, count=count)


def setvar(idn: str, value: str) -> str:
    # Allow raw "0,1,…" keys used by older snippets.
    return _cli.setvar(idn, value)


def setlst(idn: str, items: list[str]) -> str:
    return _cli.setlst(idn, items)


def run_command(idn: str, timeout_s: float = 20.0) -> None:
    _cli.run_command(idn, timeout_s=timeout_s)


def main() -> int:
    cli = Hcs01Comws()
    print("login", cli.login())
    print("P-0-4084", cli.getvar("P-0-4084.0.0"))
    print("P-0-4071", cli.getvar("P-0-4071.0.0"))
    print("P-0-4082", cli.getvar("P-0-4082.0.0"))
    print("P-0-4081", cli.getlst("P-0-4081.0.0"))
    print("P-0-4080", cli.getlst("P-0-4080.0.0"))
    print("CIP IP  ", cli.getvar("P-0-4089.0.13"))

    print("\n--- S-0-0420 parameter mode ---")
    try:
        cli.run_command("S-0-0420.0.0")
    except ComwsError as e:
        print("PM command:", e)
        print("diagnostic", cli.getvar("S-0-0095.0.0"))

    print("\n--- write profile + maps ---")
    for ident, val, is_list, items in [
        ("P-0-4084.0.0", "0xFFFE", False, None),
        ("P-0-4081.0.0", None, True, CMD_MDT),
        ("P-0-4080.0.0", None, True, ACT_AT),
    ]:
        if is_list:
            r = cli.setlst(f"{ident}.7", items)
            if "error" in r.lower():
                r = cli.setlst(ident, items)
            print(ident, r)
        else:
            r = cli.setvar_try_element7(ident, val)
            print(ident, r)

    print("\n--- S-0-0422 operating mode ---")
    try:
        cli.run_command("S-0-0422.0.0", timeout_s=40.0)
    except ComwsError as e:
        print("OM command:", e)
        print("diagnostic", cli.getvar("S-0-0095.0.0"))

    print("\n--- verify ---")
    print("P-0-4084", cli.getvar("P-0-4084.0.0"))
    print("P-0-4071", cli.getvar("P-0-4071.0.0"))
    print("P-0-4082", cli.getvar("P-0-4082.0.0"))
    print("P-0-4081", cli.getlst("P-0-4081.0.0"))
    print("P-0-4080", cli.getlst("P-0-4080.0.0"))
    print("CIP IP  ", cli.getvar("P-0-4089.0.13"))
    print("P-0-4073", cli.getvar("P-0-4073.0.0"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
