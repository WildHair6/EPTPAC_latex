import json
import os
import ssl
import time
import urllib.parse
import urllib.request

ctx = ssl.create_default_context()
API_KEY = os.environ.get("S2_API_KEY", "")
if not API_KEY:
    raise SystemExit("S2_API_KEY is not set in environment")

HEADERS = {
    "x-api-key": API_KEY,
    "User-Agent": "s2-compat-check/1.0",
}

# Query terms chosen to bias toward each target venue and spacecraft-control domain.
VENUES = {
    "TAES": "IEEE Transactions on Aerospace and Electronic Systems attitude control spacecraft",
    "TIE": "IEEE Transactions on Industrial Electronics attitude control spacecraft",
    "TII": "IEEE Transactions on Industrial Informatics control systems aerospace",
    "AA": "Acta Astronautica spacecraft attitude control",
    "AST": "Aerospace Science and Technology spacecraft control",
    "ASR": "Advances in Space Research spacecraft attitude control",
}

VENUE_KEYWORDS = {
    "TAES": ["aerospace and electronic systems"],
    "TIE": ["industrial electronics"],
    "TII": ["industrial informatics"],
    "AA": ["acta astronautica"],
    "AST": ["aerospace science and technology"],
    "ASR": ["advances in space research"],
}


def fetch(query: str, limit: int = 100):
    q = urllib.parse.quote(query)
    url = (
        "https://api.semanticscholar.org/graph/v1/paper/search"
        f"?query={q}&limit={limit}"
        "&fields=title,year,venue,publicationVenue,openAccessPdf,externalIds"
    )
    req = urllib.request.Request(url, headers=HEADERS)
    with urllib.request.urlopen(req, context=ctx, timeout=40) as r:
        payload = json.loads(r.read())
    return payload.get("data", [])


def norm_venue(paper: dict) -> str:
    pv = paper.get("publicationVenue") or {}
    cand = [
        pv.get("name") or "",
        paper.get("venue") or "",
    ]
    return " ".join(cand).strip().lower()


def has_target_venue(paper: dict, keywords):
    text = norm_venue(paper)
    return any(k in text for k in keywords)


print("Semantic Scholar venue compatibility check\n")
for tag, query in VENUES.items():
    try:
        papers = fetch(query, limit=100)
    except Exception as e:
        print(f"{tag}: API error -> {e}")
        continue

    matched = [p for p in papers if has_target_venue(p, VENUE_KEYWORDS[tag])]
    with_doi = sum(1 for p in matched if (p.get("externalIds") or {}).get("DOI"))
    with_oa = sum(1 for p in matched if (p.get("openAccessPdf") or {}).get("url"))

    print(f"{tag}: total_hits={len(papers):3d}, venue_matched={len(matched):3d}, doi={with_doi:3d}, openAccessPdf={with_oa:3d}")

    for p in matched[:3]:
        title = (p.get("title") or "")[:72]
        year = p.get("year")
        doi = (p.get("externalIds") or {}).get("DOI", "-")
        oa = (p.get("openAccessPdf") or {}).get("url", "")
        mark = "OA" if oa else "NO-OA"
        print(f"  - {year} [{mark}] DOI:{doi} | {title}")

    time.sleep(0.3)
