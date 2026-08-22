# Rockwell KNX5100C-2198 Servo Drive — RAG Corpus

Source: **Rockwell Automation Publication 2198-UM004D-EN-P - December 2022**
Contents: **2** front-matter section(s), **16** chapters (Ch. 1–16), **7** appendices (App. A–G), **1** index.
Total source PDFs: **26**  ·  Total chunks: **639**

## Layout

```
servo_drive_rag_dataset/
├── README.md
├── manifest.json                # dataset metadata + schema
├── chunks.csv                   # embedding-ready chunk index
└── markdown/                    # one Markdown file per chapter
    ├── chapter_04_connect_the_drive.md
    ├── chapter_05_set_up_ethernet_ip_communication.md
    └── ...
```

## `chunks.csv` schema

| column | type | description |
|---|---|---|
| `chunk_id` | str | 16-char md5, stable primary key |
| `chapter` | int | chapter number (4–16) |
| `chapter_title` | str | chapter title |
| `section` | str | detected `##` heading (best-effort) |
| `page_start` / `page_end` | int | page range in original PDF |
| `is_table` | bool | table-only chunk flag |
| `n_chars`, `approx_tokens` | int | length metrics (tokens ≈ chars/4) |
| `text` | str | Markdown chunk (tables preserved as Markdown; figures noted as `> **Figure N** — caption`) |
| `source_pdf` | str | original file |
| `markdown_file` | str | corresponding per-chapter file |

## Processing pipeline

1. **Text extraction** with PyMuPDF (`fitz`) — high-fidelity per-page text.
2. **Table extraction** with `pdfplumber` — every table is emitted as a Markdown table and preserved as its own chunk (`is_table = true`) so retrieval does not shred rows.
3. **Cleaning** — page headers/footers stripped, hyphenated line-break repair, whitespace normalization.
4. **Figure / table captions** promoted to Markdown blockquote notes (`> **Figure 71** — Keypad and Display`).
5. **Heading detection** — Title-Case short lines promoted to `##` sections (best-effort — treat `section` as a hint, not ground truth).
6. **Chunking** — section-aware, target ≈650 tokens with ≈80-token overlap. Table chunks are kept intact.

## Suggested use

- **Embedding**: iterate `chunks.csv`, embed the `text` column, index alongside all other columns as metadata.
- **Retrieval filter**: pre-filter by `chapter` / `is_table` for scoped questions (e.g. only fault tables in Ch. 16).
- **Citation**: use `chapter_title`, `page_start`–`page_end`, and `source_pdf` for source citations in generated answers.
- **Quality note**: figures and diagrams are referenced by caption only — no OCR was run on image-only content. If you need diagram content, run OCR on the source PDFs separately.
