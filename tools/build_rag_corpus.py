#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import os
import re
import shutil
import sys
import traceback
import unicodedata
import zipfile
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable

import fitz
import pdfplumber
from PIL import Image

try:
    import pytesseract
    _HAS_PYTESSERACT = True
except Exception:
    pytesseract = None
    _HAS_PYTESSERACT = False

CHARS_PER_TOKEN = 4

_HEADER_FOOTER_PATS = [
    re.compile(r"^\s*page\s*\d+(\s*/\s*\d+|\s*of\s*\d+)?\s*$", re.I),
    re.compile(r"^\s*\d+\s*/\s*\d+\s*$"),
    re.compile(r"^\s*-\s*\d+\s*-\s*$"),
]

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

CSV_FIELDS = [
    "chunk_id", "section_type", "chapter", "chapter_title", "section",
    "page_start", "page_end", "is_table", "is_ocr",
    "n_chars", "approx_tokens", "text", "source_pdf", "markdown_file",
]


@dataclass
class Block:
    kind: str
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


@dataclass
class PdfResult:
    pdf_path: str
    chapter_idx: int
    title: str
    markdown_file: str
    pages: int
    chunks: list[Chunk] = field(default_factory=list)
    success: bool = True
    skipped: bool = False
    error: str = ""


def now_utc() -> str:
    return datetime.now(timezone.utc).isoformat()


def slugify(s: str, maxlen: int = 80) -> str:
    s = unicodedata.normalize("NFKD", s).encode("ascii", "ignore").decode("ascii")
    s = re.sub(r"[^a-zA-Z0-9]+", "_", s).strip("_").lower()
    return (s or "untitled")[:maxlen]


def approx_tokens(s: str) -> int:
    return max(1, len(s) // CHARS_PER_TOKEN)


def sha_id(*parts: str, n: int = 16) -> str:
    return hashlib.md5("|".join(parts).encode("utf-8")).hexdigest()[:n]


def normalize_text(s: str) -> str:
    s = s.replace("\u00a0", " ")
    s = re.sub(r"[ \t]+", " ", s)
    s = re.sub(r"\n{3,}", "\n\n", s)
    return s.strip()


def dehyphenate(text: str) -> str:
    return re.sub(r"([A-Za-z]{2,})-\n([a-z]{2,})", r"\1\2", text)


def clean_line(s: str) -> str:
    s = s.replace("\u00a0", " ")
    s = re.sub(r"[ \t]+", " ", s)
    return s.rstrip()


def find_repeated_lines(pages_text: list[str], threshold: float = 0.6) -> set[str]:
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


def resolve_tesseract_cmd(explicit: str | None = None) -> str | None:
    candidates = []
    if explicit:
        candidates.append(explicit)
    env_cmd = os.environ.get("TESSERACT_CMD")
    if env_cmd:
        candidates.append(env_cmd)
    candidates.extend([
        shutil.which("tesseract") or "",
        r"C:\Program Files\Tesseract-OCR\tesseract.exe",
        r"C:\Program Files\TesseractOCR\tesseract.exe",
        r"C:\Program Files (x86)\Tesseract-OCR\tesseract.exe",
        r"C:\Program Files (x86)\TesseractOCR\tesseract.exe",
    ])
    for cand in candidates:
        if cand and Path(cand).is_file():
            return cand
    return None


def configure_ocr(explicit_cmd: str | None = None) -> tuple[bool, str | None]:
    if not _HAS_PYTESSERACT:
        return False, None
    cmd = resolve_tesseract_cmd(explicit_cmd)
    if cmd:
        pytesseract.pytesseract.tesseract_cmd = cmd
        return True, cmd
    return True, None


def extract_pdf_pages(doc: fitz.Document) -> list[str]:
    raw_pages = [doc[i].get_text("text") or "" for i in range(len(doc))]
    raw_pages = [dehyphenate(p) for p in raw_pages]
    repeated = find_repeated_lines(raw_pages)
    cleaned = []
    for pt in raw_pages:
        out = []
        for ln in pt.splitlines():
            ln = clean_line(ln)
            if not ln:
                out.append(ln)
                continue
            if ln.strip() in repeated:
                continue
            if any(p.match(ln) for p in _HEADER_FOOTER_PATS):
                continue
            out.append(ln)
        cleaned.append(normalize_text("\n".join(out)))
    return cleaned


def table_to_markdown(rows: list[list[str | None]]) -> str | None:
    if not rows or len(rows) < 2:
        return None
    norm = [[(c or "").strip().replace("\n", " ") for c in r] for r in rows]
    width = max(len(r) for r in norm)
    if width < 2:
        return None
    if not any(any(c for c in r) for r in norm):
        return None
    header = norm[0] + [""] * (width - len(norm[0]))
    body = [r + [""] * (width - len(r)) for r in norm[1:]]
    lines = [
        "| " + " | ".join(header[:width]) + " |",
        "| " + " | ".join(["---"] * width) + " |",
    ]
    for r in body:
        if not any(c for c in r):
            continue
        lines.append("| " + " | ".join(r[:width]) + " |")
    return "\n".join(lines) if len(lines) > 2 else None


def extract_tables(pdf_path: Path) -> dict[int, list[str]]:
    out: dict[int, list[str]] = {}
    try:
        with pdfplumber.open(str(pdf_path)) as pdf:
            for i, page in enumerate(pdf.pages):
                page_tables = []
                try:
                    tables = page.extract_tables() or []
                except Exception:
                    tables = []
                for t in tables:
                    md = table_to_markdown(t)
                    if md:
                        page_tables.append(md)
                if page_tables:
                    out[i] = page_tables
    except Exception as e:
        print(f"  [warn] pdfplumber failed on {pdf_path.name}: {e}", file=sys.stderr, flush=True)
    return out


def ocr_page(doc: fitz.Document, page_idx: int, dpi: int, lang: str) -> str:
    page = doc[page_idx]
    zoom = dpi / 72.0
    pix = page.get_pixmap(matrix=fitz.Matrix(zoom, zoom), alpha=False)
    img = Image.open(io.BytesIO(pix.tobytes("png")))
    try:
        text = pytesseract.image_to_string(img, lang=lang)
        return text.strip()
    finally:
        img.close()
        pix = None


def ocr_embedded_images(doc: fitz.Document, page_idx: int, lang: str, min_side: int = 200) -> list[str]:
    out = []
    page = doc[page_idx]
    for img_info in page.get_images(full=True):
        xref = img_info[0]
        try:
            base = doc.extract_image(xref)
            im = Image.open(io.BytesIO(base["image"]))
        except Exception:
            continue
        try:
            if min(im.size) < min_side:
                continue
            t = pytesseract.image_to_string(im, lang=lang).strip()
            if len(t) >= 20:
                out.append(t)
        except Exception:
            pass
        finally:
            im.close()
    return out


def promote_sections(text: str) -> str:
    out = []
    for ln in text.splitlines():
        stripped = ln.strip()
        if not stripped:
            out.append(ln)
            continue
        low = stripped.lower().rstrip(":.")
        if low in _DATASHEET_HINTS and len(stripped) <= 80:
            out.append(f"## {stripped.rstrip(':.')}")
            continue
        if (
            2 <= len(stripped.split()) <= 8
            and stripped == stripped.upper()
            and re.match(r"^[A-Z0-9][A-Z0-9 \-/&,()]+$", stripped)
            and len(stripped) <= 60
        ):
            out.append(f"## {stripped.title()}")
            continue
        m = re.match(r"^(\d+(\.\d+){0,3})\s+([A-Z][A-Za-z0-9 \-/&,()]{2,60})$", stripped)
        if m:
            depth = min(m.group(1).count(".") + 2, 4)
            out.append(f"{'#' * depth} {stripped}")
            continue
        out.append(ln)
    return "\n".join(out)


def promote_figure_captions(text: str) -> str:
    return re.sub(
        r"^(Figure|Fig\.|Table)\s*([A-Za-z0-9\-.]+)[\.:]\s*(.+)$",
        r"> **\1 \2** — \3",
        text,
        flags=re.M,
    )


def build_blocks(
    pages_text: list[str],
    tables: dict[int, list[str]],
    ocr_texts: dict[int, str],
    ocr_images: dict[int, list[str]],
) -> list[Block]:
    blocks: list[Block] = []
    current_section = ""

    for i, pt in enumerate(pages_text):
        page_num = i + 1
        pt = promote_figure_captions(promote_sections(pt))

        paras = []
        cur = []
        for ln in pt.splitlines():
            if ln.startswith(("## ", "### ", "#### ")):
                if cur:
                    paras.append("\n".join(cur).strip())
                    cur = []
                current_section = ln.lstrip("# ").strip()
                paras.append(ln)
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
                blocks.append(Block("text", p, section_ptr, page_num, page_num))
                continue
            blocks.append(Block("text", p, section_ptr, page_num, page_num))

        for tmd in tables.get(i, []):
            blocks.append(Block("table", tmd, section_ptr, page_num, page_num))

        ptext = ocr_texts.get(i, "").strip()
        if ptext:
            blocks.append(Block("ocr", f"[OCR page {page_num}]\n{ptext}", section_ptr, page_num, page_num))

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

        keep = []
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

    if buf:
        text = "\n\n".join(b.text for b in buf).strip()
        section = next((b.section for b in buf if b.section), "")
        p_start = min(b.page_start for b in buf)
        p_end = max(b.page_end for b in buf)
        is_ocr = any(b.kind == "ocr" for b in buf)
        chunks.append(Chunk(text, section, p_start, p_end, is_table=False, is_ocr=is_ocr))

    return chunks


def collect_pdfs(target: Path) -> list[Path]:
    if target.is_file() and target.suffix.lower() == ".pdf":
        return [target]
    if target.is_dir():
        return sorted(p for p in target.rglob("*.pdf") if p.is_file())
    raise SystemExit(f"[fatal] --pdf must be a PDF file or directory, got: {target}")


def pdf_markdown_name(chapter_idx: int, pdf_path: Path) -> str:
    return f"chapter_{chapter_idx:03d}_{slugify(pdf_path.stem)}.md"


def process_pdf(
    pdf_path: Path,
    chapter_idx: int,
    md_dir: Path,
    ocr_enabled: bool,
    ocr_lang: str,
    ocr_dpi: int,
    ocr_min_chars: int,
    target_tokens: int,
    overlap_tokens: int,
    resume: bool,
) -> PdfResult:
    md_filename = pdf_markdown_name(chapter_idx, pdf_path)
    md_path = md_dir / md_filename

    if resume and md_path.exists() and md_path.stat().st_size > 0:
        return PdfResult(
            pdf_path=str(pdf_path),
            chapter_idx=chapter_idx,
            title=pdf_path.stem.replace("_", " ").strip(),
            markdown_file=md_filename,
            pages=0,
            chunks=[],
            success=True,
            skipped=True,
        )

    doc = fitz.open(str(pdf_path))
    try:
        pages_text = extract_pdf_pages(doc)
        tables = extract_tables(pdf_path)

        ocr_texts: dict[int, str] = {}
        ocr_images: dict[int, list[str]] = {}

        if ocr_enabled:
            for i, pt in enumerate(pages_text):
                if len(pt) < ocr_min_chars:
                    try:
                        ot = ocr_page(doc, i, dpi=ocr_dpi, lang=ocr_lang)
                        if ot:
                            ocr_texts[i] = ot
                    except Exception as e:
                        print(f"  [warn] OCR failed on {pdf_path.name} page {i+1}: {e}", file=sys.stderr, flush=True)
                try:
                    imgs = ocr_embedded_images(doc, i, lang=ocr_lang)
                    if imgs:
                        ocr_images[i] = imgs
                except Exception:
                    pass

        blocks = build_blocks(pages_text, tables, ocr_texts, ocr_images)

        md_lines = [f"# {pdf_path.stem}\n"]
        last_page = 0
        for b in blocks:
            if b.page_start != last_page:
                md_lines.append("")
                last_page = b.page_start
            md_lines.append(b.text)
        md_body = "\n\n".join(md_lines).strip() + "\n"
        md_path.write_text(md_body, encoding="utf-8")

        chunks = chunk_blocks(blocks, target=target_tokens, overlap=overlap_tokens)

        return PdfResult(
            pdf_path=str(pdf_path),
            chapter_idx=chapter_idx,
            title=pdf_path.stem.replace("_", " ").strip(),
            markdown_file=md_filename,
            pages=len(pages_text),
            chunks=chunks,
            success=True,
        )
    finally:
        doc.close()


def write_outputs(
    results: list[PdfResult],
    failures: list[dict],
    dataset_dir: Path,
    title: str,
    publication: str,
    target: int,
    overlap: int,
    ocr_enabled: bool,
    make_zip: bool,
) -> None:
    md_dir = dataset_dir / "markdown"
    md_dir.mkdir(parents=True, exist_ok=True)

    total_chunks = 0
    csv_path = dataset_dir / "chunks.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for res in results:
            for j, ch in enumerate(res.chunks):
                text = ch.text.strip()
                if not text:
                    continue
                cid = sha_id(Path(res.pdf_path).name, str(j), text[:200])
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
                    "source_pdf": Path(res.pdf_path).name,
                    "markdown_file": res.markdown_file,
                })
                total_chunks += 1

    manifest = {
        "dataset_name": title,
        "publication": publication,
        "created_utc": now_utc(),
        "ocr_enabled": ocr_enabled,
        "chunk_target_tokens": target,
        "chunk_overlap_tokens": overlap,
        "chars_per_token_estimate": CHARS_PER_TOKEN,
        "num_pdfs_succeeded": len(results),
        "num_pdfs_failed": len(failures),
        "num_chunks": total_chunks,
        "chapters": [
            {
                "chapter": r.chapter_idx,
                "title": r.title,
                "source_pdf": Path(r.pdf_path).name,
                "markdown_file": r.markdown_file,
                "num_pages": r.pages,
                "num_chunks": len(r.chunks),
                "skipped": r.skipped,
            }
            for r in results
        ],
    }
    (dataset_dir / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    (dataset_dir / "failures.json").write_text(json.dumps(failures, indent=2), encoding="utf-8")

    readme = f"""# {title} — RAG Corpus

Source(s): **{publication or 'PDF batch'}**
Successful PDFs: **{len(results)}**
Failed PDFs: **{len(failures)}**
Total chunks: **{total_chunks}**
OCR enabled: **{ocr_enabled}**
Chunk target / overlap: **{target} / {overlap}**

## Layout
