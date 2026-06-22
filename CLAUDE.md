# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a LaTeX academic paper — "Exact Prescribed-Time Prescribed-Accuracy Control (EPTPAC) for Spacecraft under Dual Reaction Wheel Saturation." The main manuscript is `main.tex`, targeting an AIAA-style journal (using `new-aiaa.cls`).

## Build

```bash
latexmk -pdf main.tex
```

Compilation produces `main.pdf` plus auxiliary files (`*.aux`, `*.bbl`, `*.blg`, `*.log`, `*.out`, `*.synctex.gz`) which are gitignored except for pre-generated figure PDFs under `fig/`.

## Repository Structure

- **`main.tex`** — Primary manuscript. Edit this file for the main text.
- **`main_case3_expanded.tex`** — Expanded version of Case 3 (comparison with Ref.~\cite{xu_distributed_2022}), likely a supplementary/backup document.
- **`Submission/`** — Submission-ready copy with figures and bibliography (`sample.bib`).
- **`cover_letter/`** — Cover letter for journal submission.
- **`full_article/`** — Full article variant (e.g., for arXiv or preprint).
- **`tikz/`** — Standalone TikZ figure sources (`overall.tex`, `mainwork.tex` and variants).
- **`fig/`** — Pre-generated figure PDFs used by the main manuscript.
- **`s2_venue_compat_check.py`** — Python script to check venue compatibility.
- **`tech_note_examples/`** — Technical note examples.

## Git

- Default branch: `main`
- Remote: `origin` → `https://github.com/WildHair6/EPTPAC_latex`

## Custom Instructions

每次回复开头叫"刘博士"。
