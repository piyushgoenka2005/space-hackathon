"""
TLE ingestion helpers for AOSE.
Fetches TLEs from Celestrak, caches locally, and converts to ECI state.
"""

from __future__ import annotations

import os
from datetime import datetime, timezone
from typing import List, Tuple, Optional, Union, Sequence

import requests
from sgp4.api import Satrec, jday


def _sanitize_id(name: str) -> str:
    name = name.strip()
    if not name:
        return "UNKNOWN"
    return "_".join(name.split())


def fetch_tle(url: Union[str, Sequence[str]], cache_path: str, timeout_s: int = 12) -> str:
    """Fetch TLE text and cache it locally. Falls back to cache on failure."""
    urls = list(url) if isinstance(url, (list, tuple)) else [url]
    headers = {
        "User-Agent": "AOSE/1.0 (National Space Hackathon 2026)",
        "Accept": "text/plain",
    }
    for u in urls:
        try:
            resp = requests.get(u, timeout=timeout_s, headers=headers)
            resp.raise_for_status()
            text = resp.text.strip()
            # Cache only if response looks like a valid 3-line TLE catalog.
            # CelesTrak returns human-readable error strings for invalid GROUP values.
            looks_like_tle = ("\n1 " in f"\n{text}" and "\n2 " in f"\n{text}")
            if text and looks_like_tle:
                os.makedirs(os.path.dirname(cache_path), exist_ok=True)
                with open(cache_path, "w", encoding="utf-8") as f:
                    f.write(text)
                return text
        except Exception:
            continue

    if os.path.exists(cache_path):
        with open(cache_path, encoding="utf-8") as f:
            return f.read().strip()
    return ""


def parse_tle(text: str, limit: Optional[int] = None) -> List[Tuple[str, str, str]]:
    """Parse TLE text into (name, line1, line2) tuples."""
    lines = [ln.strip() for ln in text.splitlines() if ln.strip()]
    out: List[Tuple[str, str, str]] = []
    i = 0
    while i + 2 < len(lines):
        name = lines[i]
        l1 = lines[i + 1]
        l2 = lines[i + 2]
        if l1.startswith("1 ") and l2.startswith("2 "):
            out.append((name, l1, l2))
        i += 3
        if limit and len(out) >= limit:
            break
    return out


def tle_to_eci(name: str, line1: str, line2: str, when: datetime) -> Optional[Tuple[str, Tuple[float, float, float, float, float, float]]]:
    """Convert TLE to ECI state at a given time. Returns (id, state_km_kms)."""
    if when.tzinfo is None:
        when = when.replace(tzinfo=timezone.utc)
    jd, fr = jday(when.year, when.month, when.day, when.hour, when.minute, when.second + when.microsecond / 1e6)
    sat = Satrec.twoline2rv(line1, line2)
    err, r, v = sat.sgp4(jd, fr)
    if err != 0:
        return None
    # Use NORAD catalog number to guarantee uniqueness across repeated names
    # (e.g., many debris entries share the same object name label).
    cat = line1[2:7].strip() if len(line1) >= 7 else "UNK"
    sid = f"{_sanitize_id(name)}_{cat}"
    state = (r[0], r[1], r[2], v[0], v[1], v[2])
    return sid, state
