#!/usr/bin/env python3
"""
Download ESRI World Imagery tiles and save them as an MBTiles file
for offline use with the GeoViewDisplay rviz plugin.

Usage:
    python3 download_mbtiles.py --lat 41.917507 --lon 12.592535 --output map.mbtiles
    python3 download_mbtiles.py --lat 41.917507 --lon 12.592535 --output map.mbtiles --zoom 15,16,17,18,19 --radius 8
"""

XYZ_URL = (
    "https://services.arcgisonline.com/ArcGIS/rest/services"
    "/World_Imagery/MapServer/tile/{z}/{y}/{x}"
)

import argparse
import math
import sqlite3
import time
import sys
from pathlib import Path

try:
    import requests
except ImportError:
    print("Install requests:  pip install requests")
    sys.exit(1)


def lat_lon_to_tile(lat: float, lon: float, z: int) -> tuple[int, int]:
    lat_r = math.radians(lat)
    n = 2 ** z
    x = int((lon + 180.0) / 360.0 * n)
    y = int((1.0 - math.log(math.tan(lat_r) + 1.0 /
            math.cos(lat_r)) / math.pi) / 2.0 * n)
    return x, y


def create_mbtiles(path: str) -> sqlite3.Connection:
    conn = sqlite3.connect(path)
    conn.execute("""
        CREATE TABLE IF NOT EXISTS tiles (
            zoom_level  INTEGER NOT NULL,
            tile_column INTEGER NOT NULL,
            tile_row    INTEGER NOT NULL,
            tile_data   BLOB    NOT NULL,
            PRIMARY KEY (zoom_level, tile_column, tile_row)
        )
    """)
    conn.execute("""
        CREATE TABLE IF NOT EXISTS metadata (
            name  TEXT PRIMARY KEY,
            value TEXT
        )
    """)
    conn.execute(
        "INSERT OR REPLACE INTO metadata VALUES ('name',   'geo_view_tiles')")
    conn.execute(
        "INSERT OR REPLACE INTO metadata VALUES ('type',   'overlay')")
    conn.execute("INSERT OR REPLACE INTO metadata VALUES ('version','1.0')")
    conn.execute("INSERT OR REPLACE INTO metadata VALUES ('format', 'png')")
    conn.commit()
    return conn


def download(
    lat: float, lon: float,
    zoom_levels: list[int], radius: int,
    url_template: str, output: str,
    delay_s: float,
) -> None:
    session = requests.Session()
    session.headers.update({"User-Agent": "geo_view_mbtiles_downloader/1.0"})

    conn = create_mbtiles(output)

    total = sum((2 * radius + 1) ** 2 for _ in zoom_levels)
    done = 0
    skipped = 0
    errors = 0

    for z in zoom_levels:
        cx, cy = lat_lon_to_tile(lat, lon, z)
        n = 2 ** z

        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                x = (cx + dx) % n
                y = max(0, min(n - 1, cy + dy))
                y_tms = n - 1 - y   # MBTiles uses TMS (Y flipped)

                # Skip if already present
                row = conn.execute(
                    "SELECT 1 FROM tiles WHERE zoom_level=? AND tile_column=? AND tile_row=?",
                    (z, x, y_tms),
                ).fetchone()
                if row:
                    skipped += 1
                    done += 1
                    continue

                url = (
                    url_template
                    .replace("{z}", str(z))
                    .replace("{x}", str(x))
                    .replace("{y}", str(y))   # XYZ y (not TMS)
                )

                try:
                    resp = session.get(url, timeout=10)
                    resp.raise_for_status()
                    conn.execute(
                        "INSERT OR REPLACE INTO tiles VALUES (?,?,?,?)",
                        (z, x, y_tms, resp.content),
                    )
                    conn.commit()
                    done += 1
                    print(
                        f"  [{done}/{total}] z={z} x={x} y={y} ({len(resp.content)//1024} KB)")
                    if delay_s > 0:
                        time.sleep(delay_s)
                except Exception as e:
                    errors += 1
                    done += 1
                    print(f"  [{done}/{total}] z={z} x={x} y={y}  ERROR: {e}")

    conn.close()
    print(
        f"\nDone. {done - skipped - errors} downloaded, {skipped} already cached, {errors} errors.")
    print(
        f"Output: {Path(output).resolve()}  ({Path(output).stat().st_size // 1024} KB)")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--lat",    type=float, required=True,  help="Center latitude (degrees)")
    ap.add_argument("--lon",    type=float, required=True,  help="Center longitude (degrees)")
    ap.add_argument("--output", required=True,              help="Output .mbtiles file")
    ap.add_argument("--zoom",   default="17,18,19",
                    help="Comma-separated zoom levels (default: 17,18,19)")
    ap.add_argument("--radius", type=int, default=5,
                    help="Tile radius around center per zoom level (default: 5)")
    ap.add_argument("--delay",  type=float, default=0.05,
                    help="Delay between requests in seconds (default: 0.05)")
    args = ap.parse_args()

    zoom_levels = [int(z.strip()) for z in args.zoom.split(",")]
    print(f"Center:  lat={args.lat}  lon={args.lon}")
    print(f"Zooms:   {zoom_levels}  radius={args.radius}")
    print(f"Output:  {args.output}")
    total = sum((2 * args.radius + 1) ** 2 for _ in zoom_levels)
    print(f"Tiles:   up to {total}\n")

    download(
        lat=args.lat, lon=args.lon,
        zoom_levels=zoom_levels,
        radius=args.radius,
        url_template=XYZ_URL,
        output=args.output,
        delay_s=args.delay,
    )


if __name__ == "__main__":
    main()
