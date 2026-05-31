# Plan: Full Paper → JGCD Technical Note Conversion

## Goal

Convert the 32-page full paper (`main.tex`, ~8,000 body words) into a 20-22 page Technical Note (~5,000 body words) in submission format, targeting published output of 8-10 JGCD pages.

## Reference Examples

- **Xiao et al. 2026**: Technical Note, 3 theorems + 3 proofs inline, no appendix, ~5,000 body words
- **Miller & Akella 2024**: Engineering Note, proof in Appendix B, ~5,700 body words
- **Xia & Su 2018**: Engineering Note, 1 theorem + full proof inline, ~4,300 body words

Key principle from references: **proofs are kept** — inline if compact, in appendix if long.

---

## Phase 1: Structural Deletion (remove what Notes don't have)

### 1A. Remove Abstract
- **File**: `main.tex` lines 41-54
- **Action**: Delete `\begin{abstract}...\end{abstract}` block
- **Verify**: `grep "begin{abstract}" main.tex` returns nothing

### 1B. Remove Nomenclature
- **File**: `main.tex` lines 56-116
- **Action**: Delete entire `\section*{Nomenclature}` block including the longtable
- **Note**: critical symbols (σ, ω, J₀, τ_max, h_w,max, T_f) are already defined in Problem Description; verify no undefined symbols remain
- **Verify**: compile without errors; grep for any `\si{}` commands that lost their variable definitions

### 1C. Clean preamble
- **File**: `main.tex` lines 1-16
- **Action**: Remove unused packages
  - Line 11: `\usepackage[version=4]{mhchem}` — no chemistry
  - Line 4: `\usepackage{textcomp}` — likely unused
  - Line 13: change `\usepackage{longtable,tabularx}` to `\usepackage{tabularx}` — longtable only needed in Nomenclature
  - Line 15: `\usepackage{subfig}` — remove; unify multi-panel figures as single PDFs
- **Verify**: compile without errors

---

## Phase 2: Introduction Compression (~120 lines → ~70 lines)

### Target: ~900 words (matches Xiao 2026: 939 words; Miller 2024: 1,021 words)

### 2A. Keep (core narrative flow)
- Lines 122-136: Mission motivation paragraph (dual saturation coupling problem) — **keep, minor tightening**
- Lines 175-183: Conservatism critique — **keep**
- Lines 185-192: Dual saturation oversight — **keep**
- Lines 211-231: Proposed framework + contributions (3 bullets) — **keep as-is**

### 2B. Compress
- Lines 138-151: FTC → FxTC → PTC evolution — **compress from 14 to 6 lines**; this is well-known background
- Lines 153-171: Two PTC categories review — **compress from 18 to 8 lines**; cite key references without detailed description
- Lines 194-209: Optimization/MPC gap analysis — **compress from 16 to 8 lines**; focus on WHY they fail, not HOW they work

### 2C. Delete
- Lines 233-239: "The remainder of this paper is organized as follows..." — **delete**; Notes don't include this

### 2D. Add Akella citations
- Cite `wallsgrove_globally_2005` at line ~185 (torque saturation discussion)
- Cite `hu_finite-time_2017` at line ~160 (finite-time + saturation)
- Cite `miller_guidance_2024` at line ~200 (guidance/planning discussion)
- **Verify**: grep for all three citation keys in main.tex

---

## Phase 3: Problem + Preliminaries Merge (~230 lines → ~100 lines)

### 3A. Merge into single section
- Combine "Problem Description" (244-409) and "Preliminaries" (410-473) into one section
- New heading: `\section{Problem Formulation}` (no subsections needed)

### 3B. Problem Description (keep, light trim)
- Lines 262-279: Kinematics + dynamics — **keep** (core equations)
- Lines 282-293: Dual saturation constraints — **keep**
- Lines 295-345: Error kinematics derivation — **compress**; show key result, skip intermediate steps
- Lines 350-389: Assumptions 1-3 — **keep, but condense**; integrate disturbance bound and H_budget into a single paragraph with inline conditions rather than formal `\begin{assumption}` environments

### 3C. Preliminaries (keep only what's cited)
- Lemma 1 (PT stability, lines 426-433) — **keep**
- Theorem 1 (practical PT, lines 435-448) + proof (451-463) — **keep**; this is foundational
- Lemma 2 (norm inequality, 465-471) — **keep** if cited in proofs
- Remove: notation table (lines 245-259) — already standard

---

## Phase 4: Main Results — Keep Theorems + Proofs

### Strategy: Preserve mathematical content, move long proofs to appendix

### 4A. Outer-Loop (lines 477-521): Keep as-is
- ~44 lines, well-sized for a note

### 4B. Inner-Loop (lines 522-925): Restructure
- **Keep inline**: Theorem statements + short proofs (<30 lines each)
  - Theorem 2 (545-558) + proof (561-609, ~48 lines) — move to Appendix A
  - Theorem 3 (638-653) + proof (655-713, ~58 lines) — move to Appendix A
  - Theorem 4 (739-754) + proof (756-853, ~97 lines) — move to Appendix B
  - Theorem 5 (855-865) + proof (867-890, ~23 lines) — **keep inline** (short)

- **Keep inline**: Controller design description + sliding surface definition (522-544)
- **Keep inline**: Remarks that provide design guidance (715-728, 896-923)

### 4C. Parameter Synthesis (lines 926-991): Keep, light trim
- Keep 4-step synthesis procedure
- Keep Fig. 2 (overall design flowchart)
- Remark 8 (balanced synthesis, 972-985): keep if space allows

---

## Phase 5: Simulation — 4 Cases → 3 Cases (~386 lines → ~200 lines)

### New structure

| Case | Content | Source |
|------|---------|--------|
| **Case 1: Baseline** | Nominal performance + dual-saturation compliance | Current Case 1 (trimmed) |
| **Case 2: Prescribed Performance** | Time-flexibility + accuracy-sweep side-by-side in one case | Current Case 2 + Case 3 merged |
| **Case 3: Comparison** | Benchmark vs. Ref. (Xu et al. 2022) under dual saturation | Current Case 4 |

### 5A. Case 1: Baseline Performance
- Keep essential tracking figures (attitude, angular velocity, errors ~4 figures)
- Keep constraint satisfaction (torque + momentum ~2 figures)
- Step-disturbance sub-case: reduce to 1-2 sentences of text (no separate figures) or drop
- Delete: redundant individual component figures

### 5B. Case 2: Prescribed Performance (merged)
- **Time-flexibility**: Show convergence at different $T_f$ values — keep 1-2 representative figures (e.g. attitude error comparison across Tf)
- **Accuracy-sweep**: Show tunable precision — keep 1-2 figures (e.g. error norm across epsilon values)
- Display side-by-side to demonstrate both "prescribed" dimensions in one case
- Delete: individual per-axis figures (lines 1184-1228) — replace with norm-based figures or 1 combined panel

### 5C. Case 3: Comparison with Ref. [Xu et al. 2022]
- Keep: dual-saturation comparison (current Case 4.2) — this is the core contribution
- Sub-case 4.1 (torque-only): drop or summarize in 1 sentence
- Keep ~4 comparison figures (attitude, angular velocity, torque, momentum)
- Highlight: exact terminal-time convergence + dual-saturation compliance advantage

### 5D. Figures — from ~34 to ~12-14
- Keep: Fig. 1 (framework architecture), Fig. 2 (parameter synthesis)
- Case 1: ~4-5 figures (tracking, errors, torque, momentum)
- Case 2: ~2-3 figures (time sweep, precision sweep — compact format)
- Case 3: ~3-4 figures (comparison attitude, torque, momentum)
- Delete all individual per-axis figures from old Case 2

---

## Phase 6: Conclusion + Appendices

### 6A. Conclusion (lines 1379-1386)
- Keep as-is (~7 lines, right size for a note)

### 6B. Appendices — consolidate
- **New Appendix A**: Combine Theorem 2 + Theorem 3 proofs (~110 lines)
- **New Appendix B**: Theorem 4 proof (~97 lines) + existing Appendix A ($T_{f,\min}$ estimate, keep)
- **Existing Appendix B** (feedforward mismatch): keep if space; integrate into problem formulation if short enough

---

## Phase 7: Final Polish

### 7A. Add editor citations
- Insert `wallsgrove_globally_2005`, `hu_finite-time_2017`, `miller_guidance_2024` into Introduction

### 7B. Final word count check
- Target: ~5,000 body words
- Verify with `detex main.tex | wc -w`

### 7C. Compile check
- `pdflatex main.tex && bibtex main && pdflatex main.tex && pdflatex main.tex`
- Verify: no undefined references, no missing citations

---

## Verification Checklist

- [ ] No `\begin{abstract}` in file
- [ ] No `\section*{Nomenclature}` in file
- [ ] `\usepackage{mhchem}` removed
- [ ] `\usepackage{subfig}` removed
- [ ] 3 Akella citations present
- [ ] Introduction < 75 lines
- [ ] Only 3 simulation subsections (Baseline + Prescribed Perf + Comparison)
- [ ] All theorems still present (inline or in appendix)
- [ ] All proofs still present (inline or in appendix)
- [ ] ~12-14 figures total (down from 34)
- [ ] Compiles without errors
- [ ] Body word count ~5,000
- [ ] Total pages ~20-22 (submission format, double-spaced)
