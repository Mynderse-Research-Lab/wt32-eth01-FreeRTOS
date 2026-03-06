# Build Instructions (ESP-IDF)

This repository no longer tracks the `idf` folder in git.
To build with ESP-IDF, restore the last tracked `idf` project locally, then build.

## 1) Prerequisites

- ESP-IDF installed (recommended: v5.1 as used by this project)
- WT32 board connected (optional for build, required for flash/monitor)

## 2) Open an ESP-IDF shell

Use an ESP-IDF-enabled shell (or run the export script manually):

### Windows PowerShell

```powershell
. "$env:USERPROFILE\esp-idf\export.ps1"
```

## 3) Restore local `idf` project files (not committed)

From repository root:

```powershell
cd D:\Projects\wt32-eth01-base
$LAST_IDF_COMMIT = git rev-list -n 1 HEAD -- idf/CMakeLists.txt
git restore --source $LAST_IDF_COMMIT --worktree -- idf
```

This restores the `idf` project files into your working tree for local build use.

## 4) Build

```powershell
cd D:\Projects\wt32-eth01-base\idf
idf.py fullclean
idf.py build
```

## 5) Flash and monitor (optional)

```powershell
idf.py -p COM3 flash monitor
```

Use your actual COM port if different.
