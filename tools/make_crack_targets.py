"""tools/make_crack_targets.py — printable crack test targets for the D405 + OpenCrack live test.

Builds a print-ready PDF of real crack photographs (plus no-crack control surfaces) to tape down in front of
the wrist camera, a reference PDF showing what the model finds on each digital original, and the credits the
licences require.

Why these photos: OpenCrack (the model) was trained on a consolidation of 32 public crack datasets (CRACK500,
DeepCrack, SDNET2018, the METU concrete set, CrackTree, masonry and facade sets, ...) and its train/test split
is not published, so any dataset image may be one it trained on. These are photographs from Wikimedia Commons
with free licences (CC0 / public domain / CC BY / CC BY-SA), outside those datasets. They were chosen
2026-10-22 from 217 candidates: close-up surfaces in concrete, asphalt, masonry and plaster, easy to hard,
and four control surfaces without cracks, two of them crack-like (board-formed grooves, paint drips) and one
where the model marks paving joints as cracks.

  ./env.sh python tools/make_crack_targets.py                 # -> data/crack_targets/ (git-ignored)
  ./env.sh python tools/make_crack_targets.py --no-model      # skip the model reference (no GPU needed)

Outputs: crack_test_prints.pdf (cover + one photo per page + credits; print on matte paper, fit to page),
model_reference.pdf (each original with the model's crack mask in red: what the camera should reproduce),
manifest.json (sources, licences, sha256, the model's result per image) and CREDITS.md.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
import time
import urllib.parse
import urllib.request
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from crackvision.config import ConfigError, add_common_args, load_config
from crackvision.logging_setup import EXIT_OK, EXIT_RUNTIME, EXIT_USAGE

API = "https://commons.wikimedia.org/w/api.php"
USER_AGENT = "rebot-crack-vision-research/1.0 (crack test targets; Wikimedia API etiquette)"
DPI = 300
LETTER = (11.0, 8.5)                 # inches, landscape
MARGIN_IN, CAPTION_IN = 0.35, 0.45
PRINT_LONG_SIDE = 3300               # px: 11 in at 300 dpi
FONT_DIR = Path("/usr/share/fonts/truetype/dejavu")

# (role, Wikimedia Commons file title, what it is). Order = print order: easy cracks first, controls last.
TARGETS: list[tuple[str, str, str]] = [
    ("crack", "Crack (2136639056).jpg", "single crack in rough render (start here)"),
    ("crack", "Cracked Surface3.jpg", "network of cracks in old concrete"),
    ("crack", "Courtabœuf 2011 025.jpg", "branching cracks in asphalt"),
    ("crack", "Popękana nawierzchnia drogi (ubt).JPG", "cracked concrete road, wide cracks and spalling"),
    ("crack", "It starts with a crack... (35815932072).jpg", "star-shaped crack in asphalt"),
    ("crack", "Concrete cracked.jpg", "Y-shaped crack in exposed-aggregate concrete"),
    ("crack", "(20250222) Germany 12.jpg", "pavement cracks beside a road marking"),
    ("crack", "Corrosion crack.JPG", "crack along a concrete block (corrosion)"),
    ("crack", "Crack monitor in Dnipro.jpg", "crack in a plaster wall with a crack gauge"),
    ("crack", "Heilbronn - Böckingen - Kreuzgrund - Saarlandstraße - wellenförmiger Riss im Fußweg (RQCE5068).jpg",
     "wavy crack with moss in a footpath"),
    ("crack", "Deteriorated asphalt.jpg", "alligator cracking around a road arrow (hard: markings)"),
    ("crack", "Brickwork on Tickenham Road - geograph.org.uk - 6604404.jpg", "stepped crack in brickwork"),
    ("crack", "Darmsheim Brücke03 2010-06-29.jpg", "fine vertical cracks in a concrete bridge wall (hard)"),
    ("crack", "Cracked Wall.jpg", "hairline crack in a painted wall (hard)"),
    ("crack", "Qew bruecke nf beton kaputt 07 von 46.jpg", "thin crack in a concrete bridge pier (hard)"),
    ("no_crack", "Concrete kerbing texture.jpg", "plain concrete: no crack"),
    ("no_crack", "Concrete surface.jpg", "board-formed concrete grooves: no crack (hard)"),
    ("no_crack", "Paint (15983685512).jpg", "paint drips on concrete: no crack (hard)"),
    ("no_crack", "Betonwerksteinplatten betonfarben.JPG", "paving-slab joints: no crack (the model marks joints)"),
]


def _get_json(params: dict[str, Any]) -> dict[str, Any]:
    url = API + "?" + urllib.parse.urlencode(dict(params, format="json"))
    with urllib.request.urlopen(urllib.request.Request(url, headers={"User-Agent": USER_AGENT}), timeout=60) as r:
        return json.load(r)


def _clean(html: str) -> str:
    return re.sub(r"\s+", " ", re.sub(r"<[^>]+>", "", html or "")).strip()


def fetch_metadata(title: str) -> dict[str, Any]:
    """Licence, author and a print-size image URL for one Commons file (long side <= PRINT_LONG_SIDE)."""
    page = next(iter(_get_json({"action": "query", "titles": "File:" + title, "prop": "imageinfo",
                                "iiprop": "size|url|mime|extmetadata|user"})["query"]["pages"].values()))
    ii = page["imageinfo"][0]
    w, h = ii["width"], ii["height"]
    em = ii.get("extmetadata", {})
    artist = _clean(em.get("Artist", {}).get("value", ""))
    return {
        "title": title,
        "commons_page": "https://commons.wikimedia.org/wiki/File:" + urllib.parse.quote(title.replace(" ", "_")),
        "original_size": [w, h],
        "download_url": ii["url"],      # the original: custom-size thumbnails are rate-limited; resized locally
        "license": em.get("LicenseShortName", {}).get("value", ""),
        "license_url": em.get("LicenseUrl", {}).get("value", ""),
        "author": artist or f"{ii.get('user', 'unknown')} (Wikimedia Commons uploader)",
    }


def download(url: str, dst: Path, retries: int = 6) -> None:
    """Fetch the original and store it resized to the print size (long side PRINT_LONG_SIDE), as JPEG q95.
    Backs off on HTTP 429 (Wikimedia rate limit), honouring Retry-After."""
    import io
    import urllib.error
    for attempt in range(retries):
        try:
            with urllib.request.urlopen(urllib.request.Request(url, headers={"User-Agent": USER_AGENT}),
                                        timeout=180) as r:
                data = r.read()
            break
        except urllib.error.HTTPError as exc:
            if exc.code != 429 or attempt == retries - 1:
                raise
            time.sleep(float(exc.headers.get("Retry-After") or 0) or 5 * 2 ** attempt)
    img = Image.open(io.BytesIO(data)).convert("RGB")
    img.thumbnail((PRINT_LONG_SIDE, PRINT_LONG_SIDE), Image.LANCZOS)
    img.save(dst, quality=95)


def _font(size_px: int, bold: bool = False):
    path = FONT_DIR / ("DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf")
    return ImageFont.truetype(str(path), size_px) if path.is_file() else ImageFont.load_default()


def _page(img: Image.Image, caption: str) -> Image.Image:
    """One letter page (orientation follows the photo): photo fitted inside the margins, caption underneath."""
    landscape = img.width >= img.height
    W, H = (int(LETTER[0] * DPI), int(LETTER[1] * DPI)) if landscape else (int(LETTER[1] * DPI), int(LETTER[0] * DPI))
    m, cap = int(MARGIN_IN * DPI), int(CAPTION_IN * DPI)
    box = (W - 2 * m, H - 2 * m - cap)
    scale = min(box[0] / img.width, box[1] / img.height)
    photo = img.resize((round(img.width * scale), round(img.height * scale)), Image.LANCZOS)
    page = Image.new("RGB", (W, H), "white")
    page.paste(photo, (m + (box[0] - photo.width) // 2, m + (box[1] - photo.height) // 2))
    ImageDraw.Draw(page).text((m, H - m - cap + int(0.12 * DPI)), caption, fill=(90, 90, 90), font=_font(int(0.11 * DPI)))
    return page


def _text_page(lines: list[tuple[str, int, bool]]) -> Image.Image:
    W, H = int(LETTER[1] * DPI), int(LETTER[0] * DPI)          # portrait
    page = Image.new("RGB", (W, H), "white")
    d, y, m = ImageDraw.Draw(page), int(0.6 * DPI), int(0.6 * DPI)
    for text, size_pt, bold in lines:
        f = _font(int(size_pt / 72 * DPI), bold)
        d.text((m, y), text, fill="black", font=f)
        y += int(size_pt / 72 * DPI * 1.45)
    return page


def overlay(rgb: np.ndarray, mask: np.ndarray) -> Image.Image:
    out = rgb.copy()
    out[mask > 0] = (255, 0, 0)
    return Image.fromarray(out)


def build(out: Path, use_model: bool, cfg) -> int:
    img_dir = out / "images"
    img_dir.mkdir(parents=True, exist_ok=True)
    entries: list[dict[str, Any]] = []
    for k, (role, title, label) in enumerate(TARGETS, 1):
        meta = fetch_metadata(title)
        dst = img_dir / f"{k:02d}_{role}.jpg"
        if not dst.is_file():
            download(meta["download_url"], dst)
            time.sleep(1.0)                                # Wikimedia etiquette: one request at a time, paced
        entries.append({"page": k, "role": role, "label": label, "file": dst.name,
                        "sha256": hashlib.sha256(dst.read_bytes()).hexdigest()} | meta)
        print(f"{k:2d} {role:8} {label}  [{meta['license']}]", flush=True)

    predictor = None
    if use_model:
        from crackvision.live_predictor import LivePredictor
        predictor = LivePredictor(cfg, tta=True)          # TTA on: the batch path's settings
    pages, refs = [], []
    for e in entries:
        img = Image.open(img_dir / e["file"]).convert("RGB")
        caption = (f"#{e['page']}  {'CRACK' if e['role'] == 'crack' else 'NO CRACK (control)'}: {e['label']}.   "
                   f"Photo: {e['author'][:60]}, {e['license']}, via Wikimedia Commons (resized).")
        pages.append(_page(img, caption))
        if predictor is not None:
            rgb = np.asarray(img)
            mask = predictor.predict(rgb)
            e["model_on_original"] = {"crack_fraction": round(float(mask.mean()), 5), "crack_pixels": int(mask.sum()),
                                      "size": [img.width, img.height]}
            refs.append(_page(overlay(rgb, mask), f"#{e['page']} model on the digital original: "
                                                  f"{100 * mask.mean():.2f}% crack pixels (red). {e['label']}"))

    cover = [("Crack test prints: D405 + OpenCrack nnU-Net", 20, True), ("", 8, False),
             ("Real photographs from Wikimedia Commons, outside the public crack datasets the model", 11, False),
             ("was trained on. 15 cracks (easy first) and 4 no-crack controls.", 11, False), ("", 8, False),
             ("Printing: colour, MATTE paper (gloss glares), 'fit to page'. Laser or inkjet.", 11, True),
             ("Testing: tape the page flat, light it evenly (the D405 has no light of its own), camera", 11, False),
             ("15-30 cm away, square to the page. Watch `scripts/camera.py live`, then hold still for", 11, False),
             ("`scripts/camera.py snap`. Compare with model_reference.pdf (the model on the original file).", 11, False),
             ("", 8, False), ("Page  Expect     Model on original   What", 11, True)]
    for e in entries:
        ref = e.get("model_on_original", {}).get("crack_fraction")
        cover.append((f"{e['page']:>4}  {'crack' if e['role'] == 'crack' else 'none':9}  "
                      f"{('%.2f %%' % (100 * ref)) if ref is not None else 'n/a':>12}       {e['label'][:62]}", 10, False))
    credits = [("Credits (licences require attribution)", 16, True), ("", 6, False)]
    for e in entries:
        credits += [(f"#{e['page']} {e['title'][:88]}", 9, True),
                    (f"    {e['author'][:80]} - {e['license']} {e['license_url']}", 8, False),
                    (f"    {e['commons_page'][:110]}", 8, False)]
    pdf_pages = [_text_page(cover)] + pages + [_text_page(credits[:60])]
    pdf_pages[0].save(out / "crack_test_prints.pdf", save_all=True, append_images=pdf_pages[1:], resolution=DPI)
    if refs:
        refs[0].save(out / "model_reference.pdf", save_all=True, append_images=refs[1:], resolution=DPI)
    (out / "manifest.json").write_text(json.dumps({"schema": "crackvision.crack_targets/1", "targets": entries},
                                                  indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    lines = ["# Credits — crack test prints", "",
             "All photographs from Wikimedia Commons, resized for printing. Licences as listed; no endorsement implied.", ""]
    lines += [f"{e['page']}. [{e['title']}]({e['commons_page']}) — {e['author']}, "
              f"[{e['license']}]({e['license_url']})" for e in entries]
    (out / "CREDITS.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {out / 'crack_test_prints.pdf'} ({len(pdf_pages)} pages)"
          + (f", {out / 'model_reference.pdf'}" if refs else ""))
    return EXIT_OK


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(prog="make_crack_targets.py", description=__doc__.split("\n\n")[0])
    add_common_args(p)
    p.add_argument("--out", type=Path, default=None, help="default data/crack_targets/")
    p.add_argument("--no-model", action="store_true", help="skip the model reference (no GPU)")
    args = p.parse_args(argv)
    try:
        cfg = load_config(path=args.config, root=args.root)
    except ConfigError as exc:
        print(f"make_crack_targets: {exc}", file=sys.stderr)
        return EXIT_USAGE
    if args.dry_run:
        for k, (role, title, label) in enumerate(TARGETS, 1):
            print(f"{k:2d} {role:8} {title}  ({label})")
        return EXIT_OK
    try:
        return build(args.out or cfg.root / "data" / "crack_targets", not args.no_model, cfg)
    except OSError as exc:
        print(f"make_crack_targets: {exc}", file=sys.stderr)
        return EXIT_RUNTIME


if __name__ == "__main__":
    raise SystemExit(main())
