# WIZnet WIZ850io — RAG Corpus

Source: **WIZ850io Datasheet (WIZnet)**
Pages: **10**
Total chunks: **15**

## Layout

```
wiz850io_rag_dataset/
├── README.md
├── manifest.json                # dataset metadata + schema
├── chunks.csv                   # embedding-ready chunk index
└── markdown/                    # normalized markdown of the datasheet
    └── chapter_01_wiz850io_datasheet.md
```

## `chunks.csv` schema

| column | type | description |
|---|---|---|
| `chunk_id` | str | 16-char md5, stable primary key |
| `section_type` | str | `chapter` for this single-doc corpus |
| `chapter` | int | chapter number (always 1 here) |
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
4. **Figure / table captions** promoted to Markdown blockquote notes (`> **Figure 1** — Block Diagram`).
5. **Heading detection** — Title-Case short lines and common datasheet section names (Features, Pin Description, Electrical Characteristics, Dimensions, …) promoted to `##` sections (best-effort — treat `section` as a hint).
6. **Chunking** — section-aware, target ≈650 tokens with ≈80-token overlap. Table chunks are kept intact.

## Suggested use

- **Embedding**: iterate `chunks.csv`, embed the `text` column, index alongside all other columns as metadata.
- **Retrieval filter**: pre-filter by `is_table` for pinout / electrical-characteristic questions.
- **Citation**: use `chapter_title`, `page_start`–`page_end`, and `source_pdf` for source citations in generated answers.
- **Quality note**: figures and diagrams are referenced by caption only — no OCR was run on image-only content. If you need pinout diagram content that is image-only, run OCR on the source PDF separately.
