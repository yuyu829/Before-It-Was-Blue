#!/usr/bin/env python3
"""Build the offline ESP32 copy of the website.

    python3 tools/build_esp32.py

Reads index.html (the Vercel site) and writes esp32/BeforeItWasBlue/data/ :
  index.html  - same page, set up for the ESP32:
                * talks to the ESP32 directly (data-device="esp32")
                * fonts from the ESP32 instead of Google Fonts
                * exhibition settings: 45 s firing and its own wording (see CHANGES)
  assets/     - plate images, logo and voice clips (assets/voice)
  fonts/      - Manrope + Inter
Upload the data folder with "Upload LittleFS" after running this.
"""
import pathlib, re, shutil, sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
OUT = ROOT / "esp32" / "BeforeItWasBlue" / "data"

FONTS = """@font-face{font-family:"Manrope";font-weight:200;font-display:swap;src:url(fonts/manrope-latin-200-normal.woff2) format("woff2")}
@font-face{font-family:"Manrope";font-weight:300;font-display:swap;src:url(fonts/manrope-latin-300-normal.woff2) format("woff2")}
@font-face{font-family:"Manrope";font-weight:400;font-display:swap;src:url(fonts/manrope-latin-400-normal.woff2) format("woff2")}
@font-face{font-family:"Inter";font-weight:400;font-display:swap;src:url(fonts/inter-latin-400-normal.woff2) format("woff2")}
@font-face{font-family:"Inter";font-weight:500;font-display:swap;src:url(fonts/inter-latin-500-normal.woff2) format("woff2")}
"""

# (find, replace, why) - each must match exactly once
CHANGES = [
    ('<html lang="en">', '<html lang="en" data-device="esp32">', "talk to the ESP32 directly"),
    ('<link rel="preconnect" href="https://fonts.googleapis.com"><link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>\n', "", "no internet on the ESP32 network"),
    ('<link href="https://fonts.googleapis.com/css2?family=Inter:wght@400;500&family=Manrope:wght@200;300;400&display=swap" rel="stylesheet">\n', "", "no internet on the ESP32 network"),
    ("<style>\n", "<style>\n" + FONTS, "fonts served by the ESP32"),
    ("const FIRE_SECONDS=10;", "const FIRE_SECONDS=45;", "45 s firing to match the wearable"),
    ("guide:'Press and hold the dot to fire'", "guide:'Press and hold the porcelain to fire'", "exhibition wording"),
    ("Too hot, try again 🥵", "Too hot 🥵", "exhibition wording"),
]


def main():
    html = (ROOT / "index.html").read_text(encoding="utf-8")
    for find, repl, why in CHANGES:
        n = html.count(find)
        if n != 1:
            sys.exit(f"build_esp32: expected 1 match for {find!r} ({why}), found {n}. Update CHANGES.")
        html = html.replace(find, repl)
    if re.search(r"https?://", re.sub(r"http://www\.w3\.org/2000/svg", "", html)):
        sys.exit("build_esp32: page still loads something from the internet")

    if OUT.exists():
        shutil.rmtree(OUT)
    (OUT / "assets").mkdir(parents=True)
    (OUT / "fonts").mkdir()
    (OUT / "index.html").write_text(html, encoding="utf-8")
    shutil.copytree(ROOT / "assets", OUT / "assets", dirs_exist_ok=True)
    for f in (ROOT / "tools" / "fonts").iterdir():
        shutil.copy2(f, OUT / "fonts" / f.name)
    size = sum(p.stat().st_size for p in OUT.rglob("*") if p.is_file())
    print(f"built {OUT.relative_to(ROOT)} ({size/1024:.0f} KB)")


if __name__ == "__main__":
    main()
