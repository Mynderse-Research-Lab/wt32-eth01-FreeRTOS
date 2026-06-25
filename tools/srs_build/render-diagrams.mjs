/**
 * Reads an SRS Markdown file under the repo root (default: Pickup_algo_and_MQTTBridge_SRS.md),
 * extracts ```mermaid blocks, renders each with @mermaid-js/mermaid-cli (mmdc), writes a
 * preprocessed .md into docs/srs/_build/ with ![...](diagram_N.png) in the same folder.
 *
 * Usage (from tools/srs_build after `npm ci`):
 *   node render-diagrams.mjs
 *   node render-diagrams.mjs Pickup_algo_and_MQTTBridge_SRS_flat.md SRS_flat_preprocessed.md
 *
 * Args are optional: input path relative to repo root, then output basename inside _build/.
 */
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { execFileSync } from "node:child_process";

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const repoRoot = path.resolve(__dirname, "..", "..");

const inputRel =
  process.argv[2] || "Pickup_algo_and_MQTTBridge_SRS.md";
const preprocessedBasename =
  process.argv[3] || "SRS_preprocessed.md";

const srsMd = path.isAbsolute(inputRel)
  ? inputRel
  : path.join(repoRoot, inputRel);
const buildDir = path.join(repoRoot, "docs", "srs", "_build");

const MERMAID_BLOCK = /```mermaid\n([\s\S]*?)```/g;

/** Run mermaid-cli without spawning .cmd shims (Windows spawnSync EINVAL on .cmd). */
function runMmdc(mmdPath, pngPath) {
  const cliJs = path.join(
    __dirname,
    "node_modules",
    "@mermaid-js",
    "mermaid-cli",
    "src",
    "cli.js",
  );
  if (!fs.existsSync(cliJs)) {
    console.error("Missing mermaid-cli. Run: npm ci (in tools/srs_build)");
    process.exit(1);
  }
  const args = [
    cliJs,
    "-i",
    mmdPath,
    "-o",
    pngPath,
    "-w",
    "1400",
    "-H",
    "900",
    "-b",
    "white",
  ];
  execFileSync(process.execPath, args, {
    cwd: __dirname,
    stdio: "inherit",
    env: { ...process.env },
  });
}

function main() {
  if (!fs.existsSync(srsMd)) {
    console.error("Missing SRS:", srsMd);
    process.exit(1);
  }
  fs.mkdirSync(buildDir, { recursive: true });

  const source = fs.readFileSync(srsMd, "utf8");
  let index = 0;
  const out = source.replace(MERMAID_BLOCK, (_full, inner) => {
    index += 1;
    const base = `diagram_${index}`;
    const mmdPath = path.join(buildDir, `${base}.mmd`);
    const pngPath = path.join(buildDir, `${base}.png`);
    const code = String(inner).trim() + "\n";
    fs.writeFileSync(mmdPath, code, "utf8");

    runMmdc(mmdPath, pngPath);

    if (!fs.existsSync(pngPath)) {
      console.error("mmdc did not produce:", pngPath);
      process.exit(1);
    }
    // Same directory as SRS_preprocessed.md — pandoc resolves relative paths.
    return `![Figure ${index}](${base}.png)`;
  });

  if (index === 0) {
    console.warn("No ```mermaid blocks found; copying SRS verbatim to _build.");
  }

  const preprocessed = path.join(buildDir, preprocessedBasename);
  fs.writeFileSync(preprocessed, out, "utf8");
  console.log("Wrote:", preprocessed);
  console.log("Diagrams rendered:", index);
}

main();
