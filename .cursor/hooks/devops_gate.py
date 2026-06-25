#!/usr/bin/env python3
"""DevOps finalization gate (Cursor beforeShellExecution hook).

Runs the host (native) Unity unit-test suite before a `git commit` / `git push`
and blocks the action if the tests fail. This enforces the project's "finalize a
feature/improvement/debug session only when host tests pass" convention.

Design notes:
- The matcher in .cursor/hooks.json already narrows this to git commit/push, so
  this script just runs the gate and reports a decision.
- Output is a single JSON object on stdout with a `permission` of allow/deny/ask
  plus optional user_message / agent_message (the only fields beforeShellExecution
  consumes). All build/test chatter is captured, never written to stdout.
- Genuine test failures -> DENY. Missing local toolchain (no cmake/ctest, or the
  project cannot even be configured because no build generator is installed) ->
  ASK, so a developer machine without a CMake generator is not permanently
  wedged. The slow full ESP-IDF build is left to CI, not run here.
- Cross-platform: pure Python + cmake/ctest, no jq/bash dependency.

Interpreter note: .cursor/hooks.json invokes this with the Windows `py` launcher
(the only reliable Python on the primary dev machine). On macOS/Linux, change the
hook command in .cursor/hooks.json from `py` to `python3`.
"""

import json
import shutil
import subprocess
import sys
from pathlib import Path

# Repo root is two levels up from .cursor/hooks/devops_gate.py.
REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_DIR = REPO_ROOT / "test" / "host"
BUILD_DIR = REPO_ROOT / "build" / "host"

TAIL_CHARS = 1500  # how much captured output to surface in messages


def emit(permission, user_message=None, agent_message=None):
    out = {"permission": permission}
    if user_message:
        out["user_message"] = user_message
    if agent_message:
        out["agent_message"] = agent_message
    print(json.dumps(out))
    sys.exit(0)


def tail(text):
    text = (text or "").strip()
    return text[-TAIL_CHARS:]


def run(cmd):
    return subprocess.run(
        cmd,
        cwd=str(REPO_ROOT),
        capture_output=True,
        text=True,
    )


def main():
    # Consume stdin (hook input); we don't need any fields beyond the matcher.
    try:
        sys.stdin.read()
    except Exception:
        pass

    # If the host test project is missing, there is nothing to gate on.
    if not (SRC_DIR / "CMakeLists.txt").exists():
        emit("allow")

    # Toolchain present?
    if shutil.which("cmake") is None or shutil.which("ctest") is None:
        emit(
            "ask",
            user_message=(
                "DevOps gate: cmake/ctest not found, so host unit tests could "
                "not be run before this git command. Proceed only if you are "
                "sure the tests pass (CI will re-check)."
            ),
            agent_message=(
                "Host-test gate skipped: cmake or ctest is not on PATH."
            ),
        )

    # Configure. A failure here is almost always an environment issue (e.g. no
    # build generator installed), so ask rather than hard-deny.
    cfg = run(["cmake", "-S", str(SRC_DIR), "-B", str(BUILD_DIR)])
    if cfg.returncode != 0:
        emit(
            "ask",
            user_message=(
                "DevOps gate: could not configure the host test project "
                "(likely no CMake build generator installed locally). Host "
                "tests were not run; CI will still check them."
            ),
            agent_message="Host-test gate could not configure:\n" + tail(cfg.stderr or cfg.stdout),
        )

    # Build. Once configure succeeds, a build failure is a genuine code problem.
    build = run(["cmake", "--build", str(BUILD_DIR)])
    if build.returncode != 0:
        emit(
            "deny",
            user_message=(
                "DevOps gate: the host test project failed to BUILD - commit/"
                "push blocked. Fix the build, then retry."
            ),
            agent_message="Host-test build failed:\n" + tail(build.stderr or build.stdout),
        )

    # Run the tests.
    test = run(["ctest", "--test-dir", str(BUILD_DIR), "--output-on-failure"])
    if test.returncode != 0:
        emit(
            "deny",
            user_message=(
                "DevOps gate: host unit tests FAILED - commit/push blocked. "
                "Run `ctest --test-dir build/host --output-on-failure` to see "
                "details, fix the tests, then retry."
            ),
            agent_message="Host unit tests failed:\n" + tail(test.stdout or test.stderr),
        )

    emit("allow")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:  # noqa: BLE001 - failClosed will block on crash
        # Surface the error; with failClosed:true a non-JSON crash would block.
        emit(
            "ask",
            user_message=(
                "DevOps gate crashed before it could run host tests; proceed "
                "with caution. CI will still check."
            ),
            agent_message="DevOps gate exception: " + repr(exc),
        )
