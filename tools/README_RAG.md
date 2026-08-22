# RAG Corpus Generator

A single-file, dependency-light Python script that turns PDFs (datasheets, manuals, papers) into a clean, embedding-ready RAG corpus. Handles text, tables, and image-only diagrams via OCR.

## What it produces

Same schema as your existing `servo_drive_rag_dataset` and `wiz850io_rag_dataset`:

```
<name>/
├── README.md
├── manifest.json           # dataset + per-chapter counts
├── chunks.csv              # embedding-ready chunks
└── markdown/               # one .md per source PDF (auditable)
```

`chunks.csv` columns: `chunk_id`, `section_type`, `chapter`, `chapter_title`, `section`, `page_start`, `page_end`, `is_table`, `is_ocr`, `n_chars`, `approx_tokens`, `text`, `source_pdf`, `markdown_file`.

## Requirements

- Python 3.10+
- `pip install pymupdf pdfplumber pillow pytesseract`
- Tesseract binary (`apt install tesseract-ocr` on Debian/Ubuntu, `brew install tesseract` on macOS) — only needed if you use `--ocr`
- Additional Tesseract language packs (e.g. `tesseract-ocr-kor`) if OCRing non-English content

## Quick start

Single PDF:

```bash
python build_rag_corpus.py \
  --pdf ./wiz850io.pdf \
  --out ./out --name wiz850io \
  --title "WIZ850io Datasheet" \
  --publication "WIZnet WIZ850io Datasheet v1.0.4" \
  --zip
```

Batch (a whole folder — walks recursively):

```bash
python build_rag_corpus.py \
  --pdf ./gantry_docs/ \
  --out ./out --name gantry_docs \
  --workers 8 --zip
```

With OCR for diagrams / image-only pinouts:

```bash
python build_rag_corpus.py \
  --pdf ./spec.pdf \
  --out ./out --name spec \
  --ocr --ocr-lang eng --ocr-dpi 300
```

## Pipeline

1. **Text** — PyMuPDF per-page extraction, hyphenation repair, running-header/footer removal (auto-detected as lines that repeat across ≥60% of pages).
2. **Tables** — pdfplumber, converted to Markdown, each table kept as its own chunk (`is_table=True`) so retrieval never shreds rows.
3. **OCR** (opt-in) — Tesseract on
   - rendered page images when extracted text is below `--ocr-min-chars` (default 200 — catches image-only pages)
   - every embedded raster image ≥200px on any page (catches pinout diagrams inside otherwise-text pages)
   OCR chunks are tagged `is_ocr=True`.
4. **Section detection** — datasheet vocabulary (Overview, Features, Pinout, Electrical Characteristics, Dimensions, …) plus numbered (`1.2 Foo`) and ALL-CAPS headings promoted to `##`/`###`.
5. **Figure/table captions** — `Figure 3. Block Diagram` → `> **Figure 3** — Block Diagram` blockquote.
6. **Chunking** — greedy section-aware pack to `--target-tokens` (default 650) with `--overlap-tokens` overlap (default 80). Tables stay intact.
7. **Stable IDs** — 16-char md5 of `(pdf_name, index, first-200-chars)`; re-runs on the same input produce the same IDs.

## Retrieval tips

- Embed the `text` column; index the rest as metadata.
- Filter by `is_table=True` for pinouts, register maps, electrical specs.
- Filter by `is_ocr=True` for diagram/pinout content that came from image OCR.
- Filter by `chapter` for scoped Q&A over a specific manual in a batch corpus.
- Cite with `chapter_title`, `page_start`–`page_end`, `source_pdf`.

## CLI reference

```
--pdf PATH             PDF file or directory (required)
--out DIR              Output root (required)
--name NAME            Dataset folder name (required)
--title STR            Human-readable title (default: --name)
--publication STR      Publication string for README/manifest
--ocr                  Enable Tesseract OCR
--ocr-lang LANG        Tesseract language(s), e.g. "eng" or "eng+kor" (default: eng)
--ocr-dpi INT          Render DPI (default: 300; drop to 200 for speed)
--ocr-min-chars INT    Only OCR pages with < N chars of extractable text (default: 200)
--target-tokens INT    Chunk target size (default: 650)
--overlap-tokens INT   Chunk overlap (default: 80)
--workers INT          Parallel PDF workers (default: 4)
--zip                  Also produce <name>.zip
```

## Extending

- **Domain vocab**: add section keywords to `_DATASHEET_HINTS` in `promote_sections()`.
- **Different tokenizer**: replace `approx_tokens()` with tiktoken or your embedding model's tokenizer.
- **Different chunker**: `chunk_blocks()` is a plain greedy packer; swap for semantic chunking (e.g. sentence-transformers cosine breakpoints) if needed.
- **PDF password / encrypted PDFs**: call `doc.authenticate(pw)` right after `fitz.open` in `process_pdf()`.

## Known limitations

- OCR quality on rotated tables and dense schematic drawings is imperfect — verify critical pin numbers against the source.
- Section detection is heuristic. For structured specs with a real TOC/bookmarks, you can extend `process_pdf()` to consume `doc.get_toc()` for a first-class chapter split.
- pdfplumber occasionally misses ruled-line tables in some layouts; if you hit that, switch its `table_settings` to `{"vertical_strategy":"text","horizontal_strategy":"text"}` in `extract_tables()`.
