#!/usr/bin/env python3
"""
build_rag_corpus.py — Reliable RAG corpus generator for PDFs.

Handles:
  - Text extraction (PyMuPDF)
  - Table extraction (pdfplumber, tables preserved as Markdown, kept intact as own chunks)
  - Image / diagram OCR (Tesseract on rendered page images and on embedded raster images)
  - Header / footer stripping, hyphenation repair, unicode normalization
  - Section-aware chunking with configurable target / overlap
  - Multi-PDF batch mode (one PDF = one "chapter"; auto TOC-based chapter split if TOC present)
  - Stable md5 chunk_id, page ranges, is_table / is_ocr flags for retrieval filters

Output layout (matches the Kinetix / WIZ850io corpora):
  <out_dir>/<dataset_name>/
    ├── README.md
    ├── manifest.json
    ├── chunks.csv
    ├── markdown/*.md
    └── ocr/*.txt              # raw OCR text per page (only if --ocr enabled)

Usage
-----
Single PDF:
    python build_rag_corpus.py --pdf datasheet.pdf --out ./out --name my_dataset

Batch (folder of PDFs):
    python build_rag_corpus.py --pdf ./pdfs/ --out ./out --name gantry_docs

With OCR (diagrams / image-only pages):
    python build_rag_corpus.py --pdf spec.pdf --out ./out --name spec --ocr

Full options:
    --pdf PATH            PDF file or directory of PDFs (required)
    --out DIR             Output root (required)
    --name NAME           Dataset name (required, becomes folder name)
    --title TITLE         Human-readable dataset title (default: NAME)
    --publication STR     Publication string for README/manifest
    --ocr                 Enable Tesseract OCR on rendered page images (slower, more thorough)
    --ocr-lang LANG       Tesseract language(s), e.g. "eng" or "eng+kor" (default: eng)
    --ocr-dpi INT         Render DPI for OCR (default: 300)
    --ocr-min-chars INT   Only OCR pages whose extracted text is shorter than this (default: 200)
    --target-tokens INT   Target chunk size in tokens (default: 650)
    --overlap-tokens INT  Overlap between chunks in tokens (default: 80)
    --use-toc             Split each PDF by its TOC into chapters (if TOC present)
    --keep-artifacts      Keep intermediate per-page markdown scaffolding
    --zip                 Also produce <name>.zip next to the dataset folder

Exit code is 0 on success, non-zero on failure.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import re
import shutil
import sys
import unicodedata
import zipfile
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable

import fitz  # PyMuPDF
import pdfplumber
from PIL import Image

try:
    import pytesseract  # optional
    _HAS_OCR = True
except Exception:
    _HAS_OCR = False

CHARS_PER_TOKEN = 4  # heuristic; tokens ≈ chars/4

# ------------------------------------------------------------------- utilities

def slugify(s: str, maxlen: int = 60) -> str:
    s = unicodedata.normalize("NFKD", s).encode("ascii", "ignore").decode("ascii")
    s = re.sub(r"[^a-zA-Z0-9]+", "_", s).strip("_").lower()
    return (s or "untitled")[:maxlen]


def approx_tokens(s: str) -> int:
    return max(1, len(s) // CHARS_PER_TOKEN)


def sha_id(*parts: str, n: int = 16) -> str:
    return hashlib.md5("|".join(parts).encode("utf-8")).hexdigest()[:n]


# ------------------------------------------------------------ text extraction

_HEADER_FOOTER_PATS = [
    re.compile(r"^\s*page\s*\d+(\s*/\s*\d+|\s*of\s*\d+)?\s*$", re.I),
    re.compile(r"^\s*\d+\s*/\s*\d+\s*$"),
    re.compile(r"^\s*-\s*\d+\s*-\s*$"),
]


def _clean_line(s: str) -> str:
    s = s.replace("\u00a0", " ")
    s = re.sub(r"[ \t]+", " ", s)
    return s.rstrip()


def _dehyphenate(text: str) -> str:
    # join `word-\nword` -> `wordword` but leave real hyphenated compounds alone
    return re.sub(r"([A-Za-z]{2,})-\n([a-z]{2,})", r"\1\2", text)


def _find_repeated_lines(pages_text: list[str], threshold: float = 0.6) -> set[str]:
    """Detect lines that repeat across most pages — likely running headers/footers."""
    if len(pages_text) < 4:
        return set()
    counts: dict[str, int] = {}
    for pt in pages_text:
        seen = set()
        for ln in pt.splitlines():
            ln = ln.strip()
            if 3 <= len(ln) <= 120:
                seen.add(ln)
        for ln in seen:
            counts[ln] = counts.get(ln, 0) + 1
    limit = int(len(pages_text) * threshold)
    return {ln for ln, c in counts.items() if c >= limit}


def extract_pdf_pages(doc: fitz.Document) -> list[str]:
    """Return per-page cleaned text (headers/footers stripped)."""
    raw_pages = [doc[i].get_text("text") or "" for i in range(len(doc))]
    raw_pages = [_dehyphenate(p) for p in raw_pages]
    repeated = _find_repeated_lines(raw_pages)
    cleaned: list[str] = []
    for pt in raw_pages:
        out: list[str] = []
        for ln in pt.splitlines():
            ln = _clean_line(ln)
            if not ln:
                out.append(ln)
                continue
            if ln.strip() in repeated:
                continue
            if any(p.match(ln) for p in _HEADER_FOOTER_PATS):
                continue
            out.append(ln)
        body = "\n".join(out)
        body = re.sub(r"\n{3,}", "\n\n", body).strip()
        cleaned.append(body)
    return cleaned


# --------------------------------------------------------------------- tables

def extract_tables(pdf_path: Path) -> dict[int, list[str]]:
    """Return {page_index: [markdown_table, ...]}. Empty for pages without tables."""
    out: dict[int, list[str]] = {}
    try:
        with pdfplumber.open(str(pdf_path)) as pdf:
            for i, page in enumerate(pdf.pages):
                page_tables: list[str] = []
                try:
                    tables = page.extract_tables()
                except Exception:
                    tables = []
                for t in tables:
                    md = _table_to_markdown(t)
                    if md:
                        page_tables.append(md)
                if page_tables:
                    out[i] = page_tables
    except Exception as e:
        print(f"  [warn] pdfplumber failed on {pdf_path.name}: {e}", file=sys.stderr)
    return out


def _table_to_markdown(rows: list[list[str | None]]) -> str | None:
    if not rows or len(rows) < 2:
        return None
    norm = [[(c or "").strip().replace("\n", " ") for c in r] for r in rows]
    width = max(len(r) for r in norm)
    if width < 2:
        return None
    # skip if all cells empty
    if not any(any(c for c in r) for r in norm):
        return None
    header = norm[0] + [""] * (width - len(norm[0]))
    body = [r + [""] * (width - len(r)) for r in norm[1:]]
    lines = [
        "| " + " | ".join(header) + " |",
        "| " + " | ".join(["---"] * width) + " |",
    ]
    for r in body:
        # drop fully-empty rows
        if not any(c for c in r):
            continue
        lines.append("| " + " | ".join(r[:width]) + " |")
    return "\n".join(lines) if len(lines) > 2 else None


# -------------------------------------------------------------------- OCR

def ocr_page(doc: fitz.Document, page_idx: int, dpi: int, lang: str) -> str:
    if not _HAS_OCR:
        return ""
    page = doc[page_idx]
    zoom = dpi / 72.0
    mat = fitz.Matrix(zoom, zoom)
    pix = page.get_pixmap(matrix=mat, alpha=False)
    img = Image.open(io.BytesIO(pix.tobytes("png")))
    try:
        text = pytesseract.image_to_string(img, lang=lang)
    except Exception as e:
        print(f"  [warn] OCR failed on page {page_idx+1}: {e}", file=sys.stderr)
        return ""
    return text.strip()


def ocr_embedded_images(doc: fitz.Document, page_idx: int, lang: str, min_side: int = 200) -> list[str]:
    """OCR each embedded raster image on a page (skips tiny icons)."""
    if not _HAS_OCR:
        return []
    out: list[str] = []
    page = doc[page_idx]
    for img_info in page.get_images(full=True):
        xref = img_info[0]
        try:
            base = doc.extract_image(xref)
        except Exception:
            continue
        try:
            im = Image.open(io.BytesIO(base["image"]))
        except Exception:
            continue
        if min(im.size) < min_side:
            continue
        try:
            t = pytesseract.image_to_string(im, lang=lang).strip()
        except Exception:
            t = ""
        if len(t) >= 20:
            out.append(t)
    return out


# ------------------------------------------------------------ section detect

_DATASHEET_HINTS = {
    "overview", "features", "block diagram", "pin description",
    "pin assignment", "pin definition", "pin configuration",
    "absolute maximum ratings", "recommended operating conditions",
    "dc characteristics", "ac characteristics", "electrical characteristics",
    "dimension", "dimensions", "ordering information", "reference schematic",
    "reference design", "application", "applications", "package information",
    "document history", "revision history", "contents", "table of contents",
    "precautions", "note", "notes", "specifications", "specification",
    "operating conditions", "typical application", "package outline",
    "pinout", "signal description", "register description",
    "functional description", "timing characteristics", "timing diagram",
    "safety", "warnings", "warranty",
}


def promote_sections(text: str) -> str:
    """Best-effort: promote likely section headings to `##` lines."""
    out: list[str] = []
    for ln in text.splitlines():
        stripped = ln.strip()
        if not stripped:
            out.append(ln)
            continue
        low = stripped.lower().rstrip(":.")
        if low in _DATASHEET_HINTS and len(stripped) <= 80:
            out.append(f"## {stripped.rstrip(':.')}")
            continue
        # ALL CAPS 2-8 word headings
        if (
            2 <= len(stripped.split()) <= 8
            and stripped == stripped.upper()
            and re.match(r"^[A-Z0-9][A-Z0-9 \-/&,()]+$", stripped)
            and len(stripped) <= 60
        ):
            out.append(f"## {stripped.title()}")
            continue
        # Numbered section headings: "1. Overview", "2.1 Features"
        m = re.match(r"^(\d+(\.\d+){0,3})\s+([A-Z][A-Za-z0-9 \-/&,()]{2,60})$", stripped)
        if m:
            depth = m.group(1).count(".") + 2  # 1. -> ##, 1.1 -> ###
            depth = min(depth, 4)
            out.append(f"{'#' * depth} {stripped}")
            continue
        out.append(ln)
    return "\n".join(out)


def promote_figure_captions(text: str) -> str:
    return re.sub(
        r"^(Figure|Fig\.|Table)\s*([A-Za-z0-9\-\.]+)[\.\:]\s*(.+)$",
        r"> **\1 \2** — \3",
        text,
        flags=re.M,
    )


# ------------------------------------------------------------------- chunking

@dataclass
class Block:
    kind: str          # "text" | "table" | "ocr"
    text: str
    section: str = ""
    page_start: int = 1
    page_end: int = 1


@dataclass
class Chunk:
    text: str
    section: str
    page_start: int
    page_end: int
    is_table: bool = False
    is_ocr: bool = False


def build_blocks(pages_text: list[str], tables: dict[int, list[str]],
                 ocr_texts: dict[int, str], ocr_images: dict[int, list[str]]) -> list[Block]:
    """Assemble per-page text into (section, page-scoped) blocks."""
    blocks: list[Block] = []
    current_section = ""

    for i, pt in enumerate(pages_text):
        page_num = i + 1
        pt = promote_figure_captions(pt)
        pt = promote_sections(pt)

        # split page into paragraph-level blocks by blank lines
        paras: list[str] = []
        cur: list[str] = []
        for ln in pt.splitlines():
            if ln.startswith(("## ", "### ", "#### ")):
                if cur:
                    paras.append("\n".join(cur).strip())
                    cur = []
                # section marker as its own "block" so we can update state
                current_section = ln.lstrip("# ").strip()
                paras.append(ln)  # keep heading in the output stream
                continue
            if ln.strip() == "":
                if cur:
                    paras.append("\n".join(cur).strip())
                    cur = []
                continue
            cur.append(ln)
        if cur:
            paras.append("\n".join(cur).strip())

        section_ptr = current_section
        for p in paras:
            if not p.strip():
                continue
            if p.startswith(("## ", "### ", "#### ")):
                section_ptr = p.lstrip("# ").strip()
                # emit as a heading-only text block so markdown keeps structure
                blocks.append(Block("text", p, section_ptr, page_num, page_num))
                continue
            blocks.append(Block("text", p, section_ptr, page_num, page_num))

        # tables detected on this page — one block each, kept intact
        for tmd in tables.get(i, []):
            blocks.append(Block("table", tmd, section_ptr, page_num, page_num))

        # OCR page-level text (only if we captured it)
        ptext = ocr_texts.get(i, "").strip()
        if ptext:
            blocks.append(Block("ocr", f"[OCR page {page_num}]\n{ptext}", section_ptr, page_num, page_num))

        # OCR from embedded images
        for j, itxt in enumerate(ocr_images.get(i, [])):
            blocks.append(Block("ocr", f"[OCR image {page_num}.{j+1}]\n{itxt}", section_ptr, page_num, page_num))

    return blocks


def chunk_blocks(blocks: list[Block], target: int, overlap: int) -> list[Chunk]:
    chunks: list[Chunk] = []
    buf: list[Block] = []
    buf_tokens = 0

    def flush():
        nonlocal buf, buf_tokens
        if not buf:
            return
        text = "\n\n".join(b.text for b in buf).strip()
        section = next((b.section for b in buf if b.section), "")
        p_start = min(b.page_start for b in buf)
        p_end = max(b.page_end for b in buf)
        is_ocr = any(b.kind == "ocr" for b in buf)
        chunks.append(Chunk(text, section, p_start, p_end, is_table=False, is_ocr=is_ocr))
        # overlap tail
        keep: list[Block] = []
        tok = 0
        for b in reversed(buf):
            t = approx_tokens(b.text)
            if tok + t > overlap and keep:
                break
            keep.insert(0, b)
            tok += t
        buf = keep
        buf_tokens = tok

    for b in blocks:
        if b.kind == "table":
            if buf:
                flush()
                buf = []
                buf_tokens = 0
            chunks.append(Chunk(b.text, b.section, b.page_start, b.page_end, is_table=True))
            continue
        t = approx_tokens(b.text)
        if buf_tokens + t > target and buf:
            flush()
        buf.append(b)
        buf_tokens += t

    # final flush without overlap trimming
    if buf:
        text = "\n\n".join(b.text for b in buf).strip()
        section = next((b.section for b in buf if b.section), "")
        p_start = min(b.page_start for b in buf)
        p_end = max(b.page_end for b in buf)
        is_ocr = any(b.kind == "ocr" for b in buf)
        chunks.append(Chunk(text, section, p_start, p_end, is_table=False, is_ocr=is_ocr))

    return chunks


# -------------------------------------------------------------- per-PDF pipeline

@dataclass
class PdfResult:
    pdf_path: Path
    chapter_idx: int
    title: str
    markdown_file: str
    pages: int
    chunks: list[Chunk] = field(default_factory=list)


def process_pdf(pdf_path: Path, chapter_idx: int, md_dir: Path,
                ocr_enabled: bool, ocr_lang: str, ocr_dpi: int, ocr_min_chars: int,
                target_tokens: int, overlap_tokens: int) -> PdfResult:
    doc = fitz.open(str(pdf_path))
    pages_text = extract_pdf_pages(doc)
    tables = extract_tables(pdf_path)

    # OCR: run when the page has almost no extractable text (image-only) or when explicitly requested
    ocr_texts: dict[int, str] = {}
    ocr_images: dict[int, list[str]] = {}
    if ocr_enabled and _HAS_OCR:
        for i, pt in enumerate(pages_text):
            if len(pt) < ocr_min_chars:
                ot = ocr_page(doc, i, dpi=ocr_dpi, lang=ocr_lang)
                if ot:
                    ocr_texts[i] = ot
            imgs = ocr_embedded_images(doc, i, lang=ocr_lang)
            if imgs:
                ocr_images[i] = imgs

    # Build blocks + markdown
    blocks = build_blocks(pages_text, tables, ocr_texts, ocr_images)
    md_lines: list[str] = [f"# {pdf_path.stem}\n"]
    last_page = 0
    for b in blocks:
        if b.page_start != last_page:
            md_lines.append(f"\n<!-- page {b.page_start} -->")
            last_page = b.page_start
        md_lines.append(b.text)
    md_body = "\n\n".join(md_lines)
    md_filename = f"chapter_{chapter_idx:02d}_{slugify(pdf_path.stem)}.md"
    (md_dir / md_filename).write_text(md_body, encoding="utf-8")

    chunks = chunk_blocks(blocks, target=target_tokens, overlap=overlap_tokens)
    doc.close()
    return PdfResult(
        pdf_path=pdf_path,
        chapter_idx=chapter_idx,
        title=pdf_path.stem.replace("_", " ").strip(),
        markdown_file=md_filename,
        pages=len(pages_text),
        chunks=chunks,
    )


# ------------------------------------------------------------------- output

CSV_FIELDS = [
    "chunk_id", "section_type", "chapter", "chapter_title", "section",
    "page_start", "page_end", "is_table", "is_ocr",
    "n_chars", "approx_tokens", "text", "source_pdf", "markdown_file",
]


def write_outputs(results: list[PdfResult], out_root: Path, name: str,
                  title: str, publication: str, target: int, overlap: int,
                  ocr_enabled: bool, make_zip: bool) -> None:
    dataset_dir = out_root / name
    md_dir = dataset_dir / "markdown"
    md_dir.mkdir(parents=True, exist_ok=True)

    # markdown files were written into a temp dir; move them into dataset
    # (handled by caller — we wrote directly into md_dir)

    total_chunks = 0
    csv_path = dataset_dir / "chunks.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for res in results:
            for j, ch in enumerate(res.chunks):
                text = re.sub(r"<!--.*?-->\s*", "", ch.text).strip()
                if not text:
                    continue
                cid = sha_id(res.pdf_path.name, str(j), text[:200])
                writer.writerow({
                    "chunk_id": cid,
                    "section_type": "chapter",
                    "chapter": res.chapter_idx,
                    "chapter_title": res.title,
                    "section": ch.section,
                    "page_start": ch.page_start,
                    "page_end": ch.page_end,
                    "is_table": ch.is_table,
                    "is_ocr": ch.is_ocr,
                    "n_chars": len(text),
                    "approx_tokens": approx_tokens(text),
                    "text": text,
                    "source_pdf": res.pdf_path.name,
                    "markdown_file": res.markdown_file,
                })
                total_chunks += 1

    manifest = {
        "dataset_name": title,
        "publication": publication,
        "num_chapters": len(results),
        "num_chunks": total_chunks,
        "chunk_target_tokens": target,
        "chunk_overlap_tokens": overlap,
        "chars_per_token_estimate": CHARS_PER_TOKEN,
        "ocr_enabled": ocr_enabled,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "schema": {c: "" for c in CSV_FIELDS},
        "chapters": [
            {
                "chapter": r.chapter_idx,
                "title": r.title,
                "source_pdf": r.pdf_path.name,
                "markdown_file": r.markdown_file,
                "num_pages": r.pages,
                "num_chunks": len(r.chunks),
            }
            for r in results
        ],
    }
    (dataset_dir / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")

    readme = f"""# {title} — RAG Corpus

Source(s): **{publication or ', '.join(r.pdf_path.name for r in results)}**
Chapters (PDFs): **{len(results)}**
Total chunks: **{total_chunks}**
OCR enabled: **{ocr_enabled}**
Chunk target / overlap (tokens): **{target} / {overlap}**

## Layout

```
{name}/
├── README.md
├── manifest.json         # dataset metadata + per-chapter counts
├── chunks.csv            # embedding-ready chunk index
└── markdown/             # one Markdown file per source PDF
```

## `chunks.csv` schema

| column | type | description |
|---|---|---|
| `chunk_id` | str | 16-char md5, stable primary key |
| `section_type` | str | `chapter` (extend for front/appendix if needed) |
| `chapter` | int | chapter number (1-based, matches manifest) |
| `chapter_title` | str | derived from PDF filename |
| `section` | str | detected `##` heading (best-effort) |
| `page_start` / `page_end` | int | page range in original PDF |
| `is_table` | bool | table-only chunk flag (kept intact) |
| `is_ocr` | bool | chunk sourced from OCR (image-only / diagram) |
| `n_chars`, `approx_tokens` | int | length metrics (tokens ≈ chars/4) |
| `text` | str | Markdown chunk |
| `source_pdf` | str | original file |
| `markdown_file` | str | corresponding per-chapter markdown file |

## Pipeline

1. **Text**: PyMuPDF per-page extraction, hyphenation repair, running header/footer removal.
2. **Tables**: pdfplumber → Markdown; each table is its own chunk (`is_table=True`).
3. **OCR** (optional): Tesseract on rendered pages when extracted text < `--ocr-min-chars`, plus OCR of embedded raster images. Chunks tagged `is_ocr=True`.
4. **Section detection**: datasheet vocab + numbered / ALL-CAPS heuristics → `##`/`###` headings.
5. **Chunking**: section-aware greedy pack to `--target-tokens` with `--overlap-tokens` overlap; tables stay whole.

## Retrieval tips

- Embed the `text` column, index the rest as metadata.
- Pre-filter by `is_table` for pinouts / electrical characteristics.
- Pre-filter by `is_ocr=True` when you need diagram / mechanical-drawing content.
- Cite with `chapter_title`, `page_start`–`page_end`, `source_pdf`.
"""
    (dataset_dir / "README.md").write_text(readme, encoding="utf-8")

    if make_zip:
        zip_path = out_root / f"{name}.zip"
        with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as zf:
            for p in dataset_dir.rglob("*"):
                zf.write(p, p.relative_to(out_root))
        print(f"[ok] wrote {zip_path}")


# ------------------------------------------------------------------------ CLI

def collect_pdfs(target: Path) -> list[Path]:
    if target.is_file() and target.suffix.lower() == ".pdf":
        return [target]
    if target.is_dir():
        return sorted(p for p in target.rglob("*.pdf") if p.is_file())
    raise SystemExit(f"[fatal] --pdf must be a PDF file or a directory, got: {target}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pdf", required=True, type=Path, help="PDF file or directory")
    ap.add_argument("--out", required=True, type=Path, help="Output root directory")
    ap.add_argument("--name", required=True, help="Dataset folder name")
    ap.add_argument("--title", default=None, help="Human-readable dataset title")
    ap.add_argument("--publication", default="", help="Publication string for README/manifest")
    ap.add_argument("--ocr", action="store_true", help="Enable OCR on image-only pages + embedded images")
    ap.add_argument("--ocr-lang", default="eng", help="Tesseract language(s), e.g. eng or eng+kor")
    ap.add_argument("--ocr-dpi", type=int, default=300, help="Render DPI for OCR (default 300)")
    ap.add_argument("--ocr-min-chars", type=int, default=200, help="Only OCR pages with extracted text shorter than this")
    ap.add_argument("--target-tokens", type=int, default=650)
    ap.add_argument("--overlap-tokens", type=int, default=80)
    ap.add_argument("--workers", type=int, default=4, help="Parallel PDF workers")
    ap.add_argument("--zip", dest="make_zip", action="store_true", help="Also write <name>.zip")
    args = ap.parse_args()

    if args.ocr and not _HAS_OCR:
        print("[fatal] --ocr requested but pytesseract is not installed", file=sys.stderr)
        return 2

    pdfs = collect_pdfs(args.pdf)
    if not pdfs:
        print("[fatal] no PDFs found", file=sys.stderr)
        return 2

    dataset_dir = args.out / args.name
    md_dir = dataset_dir / "markdown"
    if dataset_dir.exists():
        shutil.rmtree(dataset_dir)
    md_dir.mkdir(parents=True, exist_ok=True)

    print(f"[info] {len(pdfs)} PDF(s), OCR={'on' if args.ocr else 'off'}, target={args.target_tokens}tok, overlap={args.overlap_tokens}tok")
    results: list[PdfResult] = [None] * len(pdfs)  # type: ignore
    with ThreadPoolExecutor(max_workers=max(1, args.workers)) as ex:
        futs = {
            ex.submit(
                process_pdf, p, i + 1, md_dir,
                args.ocr, args.ocr_lang, args.ocr_dpi, args.ocr_min_chars,
                args.target_tokens, args.overlap_tokens,
            ): i
            for i, p in enumerate(pdfs)
        }
        for fut in as_completed(futs):
            i = futs[fut]
            try:
                results[i] = fut.result()
                print(f"  [done] {pdfs[i].name}: pages={results[i].pages} chunks={len(results[i].chunks)}")
            except Exception as e:
                print(f"  [error] {pdfs[i].name}: {e}", file=sys.stderr)
                raise

    write_outputs(
        results, args.out, args.name,
        title=args.title or args.name,
        publication=args.publication,
        target=args.target_tokens,
        overlap=args.overlap_tokens,
        ocr_enabled=args.ocr,
        make_zip=args.make_zip,
    )
    print(f"[ok] dataset: {dataset_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
