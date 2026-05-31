"""
Clean PDF-to-MD conversion artifacts from tech note example markdown files.
Removes: page headers, running headers, page numbers, picture placeholders,
picture text blocks, publication metadata, author affiliation footnotes,
and associate editor lines.
"""
import re
import os

def clean_md_file(filepath):
    # Use \\?\ prefix for Windows long path support
    if os.name == 'nt' and not filepath.startswith('\\\\?\\'):
        filepath = '\\\\?\\' + os.path.abspath(filepath)
    with open(filepath, 'r', encoding='utf-8') as f:
        lines = f.readlines()

    cleaned = []
    in_picture_text = False
    prev_line_empty = True  # track consecutive blanks

    for line in lines:
        stripped = line.strip()

        # --- Skip picture text blocks ---
        if '----- Start of picture text -----' in stripped:
            in_picture_text = True
            continue
        if '----- End of picture text -----' in stripped:
            in_picture_text = False
            continue
        if in_picture_text:
            continue

        # --- Skip picture placeholders ---
        if stripped.startswith('**==> picture') and stripped.endswith('intentionally omitted <==**'):
            continue

        # --- Skip page headers (journal title line) ---
        # e.g., "JOURNAL OF GUIDANCE, CONTROL, AND DYNAMICS Vol. 40, No. 5, May 2017"
        if re.match(r'^JOURNAL OF GUIDANCE', stripped):
            continue

        # --- Skip running headers ---
        # e.g., "J. GUIDANCE, VOL. 40, NO. 5: ENGINEERING NOTES"
        # e.g., "J. GUIDANCE, VOL. 49, NO. 6: TECHNICAL NOTES"
        # e.g., "ENGINEERING NOTES" (standalone)
        # e.g., "TECHNICAL NOTES" (standalone)
        if re.match(r'^J\. GUIDANCE', stripped):
            continue
        if stripped in ('ENGINEERING NOTES', 'TECHNICAL NOTES'):
            continue
        # Also catch cases where running header is split: e.g. line1="J. GUIDANCE, VOL. 40, NO. 5:", line2="ENGINEERING NOTES"
        # Handled by checking consecutive matches below

        # --- Skip AIAA logo placeholder ---
        if stripped == '**==> picture [46 x 46] intentionally omitted <==**':
            continue

        # --- Skip page numbers (standalone digits, possibly with leading/trailing spaces) ---
        # Page numbers are typically 1-4 digit numbers on their own line
        if re.match(r'^\d{1,4}$', stripped):
            # Check it's not part of actual content (e.g. equation numbers, section numbers)
            # In these PDFs, page numbers appear as isolated lines with just the number
            continue

        # --- Skip publication metadata ---
        # Long lines with "Received", copyright, etc.
        if stripped.startswith('Received ') and ('published online' in stripped or 'Copyright' in stripped):
            continue
        # Short copyright/rights lines
        if stripped.startswith('DOI:') or stripped.startswith('https://doi.org'):
            # Keep DOI links as they're useful metadata, but skip the rest
            # Actually, let's keep DOI - it's useful reference info
            pass

        # --- Skip author affiliation footnote markers ---
        # Lines like "*Associate Scientist, ..." or "†Ph.D. Student, ..."
        # Also handle list-item versions: "- *Ph.D. Student, ..."
        # Unicode variants: ∗ (U+2217), ✝ (U+271D), etc.
        footnote_line = re.sub(r'^[-–—]\s+', '', stripped)  # strip markdown list markers
        if re.match(r'^[\*∗✝†‡§¶‖>]', footnote_line) and len(footnote_line) > 5:
            # Check if this looks like an affiliation footnote
            if any(keyword in footnote_line.lower() for keyword in
                   ['student', 'professor', 'scientist', 'university', 'department',
                    'corresponding author', 'fellow', 'member', 'research',
                    'candidate', 'engineer', 'college']):
                continue

        # --- Skip "A preliminary version of this work..." lines (conference paper notices) ---
        if stripped.startswith('A preliminary version of this work') and len(stripped) > 200:
            continue

        # --- Skip "Related work," lines that are conference paper notices ---
        if stripped.startswith('Related work,') and 'presented as' in stripped:
            continue

        # --- Skip "Associate Editor" line ---
        if re.match(r'^[A-Z][a-z]+ [A-Z][a-z]+ Associate Editor', stripped):
            continue

        # --- Normalize: collapse multiple consecutive blank lines ---
        if stripped == '':
            if prev_line_empty:
                continue
            prev_line_empty = True
            cleaned.append(line)
            continue
        else:
            prev_line_empty = False

        cleaned.append(line)

    # Post-processing: strip leading/trailing blank lines
    while cleaned and cleaned[0].strip() == '':
        cleaned.pop(0)
    while cleaned and cleaned[-1].strip() == '':
        cleaned.pop(-1)

    with open(filepath, 'w', encoding='utf-8') as f:
        f.writelines(cleaned)

    print(f"Cleaned: {os.path.basename(filepath)}")

if __name__ == '__main__':
    md_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'md')
    for filename in sorted(os.listdir(md_dir)):
        if filename.endswith('.md'):
            filepath = os.path.join(md_dir, filename)
            clean_md_file(filepath)

    print("\nDone cleaning all MD files.")
