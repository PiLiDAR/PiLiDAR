"""Entry point for the PiLiDAR application.

Running the file without arguments starts a complete scan in headless mode.
Passing ``--gui`` opens a small graphical interface that allows beginners to
start, stop and parameterise the process without touching the command line.
"""

from __future__ import annotations

import argparse
from typing import Optional

from lib.config import Config
from lib.scan_controller import ScanController

BANNER = r"""
 ____  _ _     _ ____    _    ____
|  _ \(_) |   (_)  _ \  / \  |  _ \
| |_) | | |   | | | | |/ _ \ | |_) |
|  __/| | |___| | |_| / ___ \|  _ <
|_|   |_|_____|_|____/_/   \_\_| \_\
"""


def run_headless_scan() -> None:
    """Execute the full pipeline and print helpful hints."""

    # Headless-Modus = Alles läuft automatisch ohne Benutzeroberfläche.
    # Ideal, wenn der Scan per SSH oder beim Systemstart erfolgen soll.

    print(BANNER)
    print("Starte PiLiDAR im Headless-Modus...")
    config = Config()
    controller = ScanController(config)

    try:
        result = controller.run_scan()
    except KeyboardInterrupt:
        controller.request_stop()
        print("Scan durch Nutzer abgebrochen.")
        return

    scan_dir = result.get("scan_dir") if result else None
    if scan_dir:
        print(f"Alle Daten liegen im Ordner: {scan_dir}")
        if config.get("ENABLE_3D"):
            print(f"  Intensitätspunktwolke: {config.intensity_pcd_path}")
            if config.get("ENABLE_VERTEXCOLOUR"):
                print(f"  Farbige Punktwolke:    {config.vertex_pcd_path}")
        if config.get("ENABLE_CAM"):
            print(f"  Panorama:              {config.pano_path}")


def launch_gui() -> None:
    """Launch the Tkinter based GUI."""

    # GUI-Modus erklärt jeden Schritt, ideal für Einsteiger*innen am Pi mit Bildschirm.

    print(BANNER)
    print("Starte grafische Oberfläche...")
    config = Config()
    controller = ScanController(config)

    from lib.gui import PiLiDARApp

    PiLiDARApp(controller).run()


def parse_args(args: Optional[list[str]] = None) -> argparse.Namespace:
    """Interpret command-line switches such as ``--gui`` for non-technical users."""

    parser = argparse.ArgumentParser(
        description=(
            "PiLiDAR Steuerprogramm – ohne Angaben läuft ein Komplettscan im Hintergrund."  # noqa: E501
        )
    )
    parser.add_argument(
        "--gui",
        action="store_true",
        help=(
            "Öffnet eine einfache Oberfläche mit Buttons und erklärt jeden Schritt. "
            "Ohne --gui läuft der Scan vollautomatisch im Terminal."
        ),
    )
    return parser.parse_args(args=args)


def main() -> None:
    args = parse_args()
    if args.gui:
        launch_gui()
    else:
        run_headless_scan()


if __name__ == "__main__":
    main()
