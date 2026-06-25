#Requires -Version 5.1
param(
  [switch]$Flat
)
<#
.SYNOPSIS
  Reproducible build: Mermaid -> PNG (@mermaid-js/mermaid-cli) -> DOCX (pandoc).

.DESCRIPTION
  Run from repository root:
    .\tools\srs_build\build.ps1
    .\tools\srs_build\build.ps1 -Flat

  -Flat builds from Pickup_algo_and_MQTTBridge_SRS_flat.md to Pickup_algo_and_MQTTBridge_SRS_flat.docx
  (preprocessed markdown: docs/srs/_build/SRS_flat_preprocessed.md).

  Prerequisites:
    - Node.js 18+ and npm on PATH (or Scoop node at ~/scoop/apps/nodejs/current)
    - pandoc on PATH (e.g. scoop install pandoc)

  Outputs:
    - docs/srs/_build/diagram_*.mmd, diagram_*.png, SRS_preprocessed.md (or SRS_flat_preprocessed.md)
    - Pickup_algo_and_MQTTBridge_SRS.docx (or Pickup_algo_and_MQTTBridge_SRS_flat.docx) at repo root
#>
$ErrorActionPreference = 'Stop'
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RepoRoot = Resolve-Path (Join-Path $ScriptDir '..\..')

# Prefer Scoop-installed Node when npm is not on PATH (common in IDE shells).
$nodeScoop = Join-Path $env:USERPROFILE 'scoop\apps\nodejs\current'
if (Test-Path (Join-Path $nodeScoop 'node.exe')) {
  $env:PATH = "$nodeScoop;$env:PATH"
}

Push-Location $ScriptDir
try {
  if (-not (Get-Command npm -ErrorAction SilentlyContinue)) {
    throw "npm not found. Install Node.js 18+ (e.g. scoop install nodejs-lts) or add Scoop node to PATH."
  }
  Write-Host "npm ci in $ScriptDir"
  npm ci

  if ($Flat) {
    Write-Host "node render-diagrams.mjs (flat SRS)"
  } else {
    Write-Host "node render-diagrams.mjs"
  }
  if (-not (Get-Command node -ErrorAction SilentlyContinue)) {
    throw "node not found on PATH after Scoop prepend."
  }
  if ($Flat) {
    node .\render-diagrams.mjs Pickup_algo_and_MQTTBridge_SRS_flat.md SRS_flat_preprocessed.md
  } else {
    node .\render-diagrams.mjs
  }

  $pandoc = Get-Command pandoc -ErrorAction SilentlyContinue
  if (-not $pandoc) {
    throw "pandoc not found. Install: scoop install pandoc"
  }

  if ($Flat) {
    $pre = Join-Path $RepoRoot 'docs\srs\_build\SRS_flat_preprocessed.md'
    $out = Join-Path $RepoRoot 'Pickup_algo_and_MQTTBridge_SRS_flat.docx'
  } else {
    $pre = Join-Path $RepoRoot 'docs\srs\_build\SRS_preprocessed.md'
    $out = Join-Path $RepoRoot 'Pickup_algo_and_MQTTBridge_SRS.docx'
  }
  $resPath = Join-Path $RepoRoot 'docs\srs\_build'

  if (-not (Test-Path $pre)) {
    throw "Missing preprocessed SRS: $pre (render-diagrams.mjs failed?)"
  }

  Set-Location $RepoRoot
  Write-Host "pandoc -> $out"
  & pandoc $pre `
    --from=markdown+pipe_tables+backtick_code_blocks+fenced_code_attributes `
    --to=docx `
    --toc --toc-depth=3 `
    --resource-path="$resPath" `
    --output $out

  if (-not (Test-Path $out)) { throw "pandoc did not create $out" }
  Get-Item $out | Format-List FullName, Length, LastWriteTime
}
finally {
  Pop-Location
}
