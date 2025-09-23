# PiLiDAR – Vollautomatischer 3D-Scanner für den Raspberry Pi 5

Diese Version von **PiLiDAR** richtet sich vollständig auf den Raspberry Pi 5
aus.  Sie kombiniert den über **USB** angebundenen **STL27L LiDAR**, eine
**Raspberry Pi Camera Module 2** und einen über einen **A4988** Treiber
angeschlossenen **NEMA17 Schrittmotor mit Planetengetriebe**.  Ziel des
Projekts ist es, absolute Einsteiger*innen sicher zum fertigen Scan zu führen –
vom Erfassen der Rohdaten über das automatische Panoramastitching bis hin zur
farbigen Punktwolke.

*Keine Zusatzhardware nötig*: Es werden **keine** Taster, I²C-Sensoren oder
Beschleunigungssensoren verbaut.  Alle Geräte werden rein per Software
gesteuert.

Die folgenden Kapitel führen dich kommentiert durch den kompletten Workflow.

---

## 1. Installation und Vorbereitung

1. **Repository klonen**

   ```bash
   git clone https://github.com/<dein-account>/PiLiDAR.git
   cd PiLiDAR
   ```

2. **Python-Umgebung vorbereiten**  
   Eine virtuelle Umgebung verhindert Konflikte mit anderen Projekten:

   ```bash
   python3 -m venv .venv
   source .venv/bin/activate  # Windows: .venv\Scripts\activate
   pip install --upgrade pip
   pip install -r requirements.txt
   ```

3. **Hardware verbinden**  
   Die Tabelle zeigt, welche Pins auf dem Raspberry Pi 5 genutzt werden.  Die
   Ansteuerung erfolgt ausschließlich über PWM-Impulse am STEP-Pin des A4988.

   | Komponente | Anschluss | Pi-GPIO |
   |------------|-----------|---------|
   | STL27L LiDAR | USB-Kabel | USB 3.0 Port (blau) |
   | A4988 DIR    | Richtung des Motors | GPIO 26 |
   | A4988 STEP   | PWM-Impulse | GPIO 19 |
   | A4988 ENABLE | Aktiviert den Treiber (Low = aktiv) | GPIO 17 |
   | A4988 MS1–MS3 | Mikrostepping (16 ×) | GPIO 5, 6, 13 |
   | Relais für Motorversorgung | Schaltet Stepper-Strom | GPIO 24 |
   | Kamera | CSI-Anschluss | Raspberry Pi Camera Module 2 |

   *Warum kein Taster?*  Der komplette Ablauf startet per Software (GUI oder
   Kommandozeile).  Zusätzliche Schalter würden Einsteiger*innen nur verwirren
   – daher verzichten wir bewusst darauf.

4. **Serielle Schnittstelle freigeben**  
   Der STL27L meldet sich als `/dev/ttyUSB0`.  Der folgende Befehl sorgt dafür,
   dass der Pi ohne Root-Rechte darauf zugreifen kann:

   ```bash
   sudo chmod a+rw /dev/ttyUSB0
   ```

5. **Panorama-Software (optional aber empfohlen)**  
   Für das automatische Stitchen sollte [Hugin](https://hugin.sourceforge.io/)
   installiert sein.  Unter Raspberry Pi OS gelingt das mit `sudo apt install
   hugin-tools`.

---

## 2. Nutzung

### 2.1 Grafische Benutzeroberfläche

Die GUI richtet sich an Einsteiger*innen.  Sie erklärt jeden Schritt und
startet den vollautomatischen Workflow (LiDAR lesen → Motor drehen → Fotos
aufnehmen → Panorama → Punktwolke) per Mausklick.

```bash
python PiLiDAR.py --gui
```

*Wichtige Funktionen in der GUI*

- **Scan ID**: optionaler Name für den Durchlauf, wird als Ordnername benutzt.
- **Horizontale Auflösung**: bestimmt das Schrittmaß des Motors in Grad.
- **Scanwinkel**: begrenzt den vertikalen Scanbereich.
- **Schalter**: Kamera, LiDAR und 3D-Punktwolke können einzeln aktiviert werden.
- Der Statusbereich protokolliert jeden Arbeitsschritt. Nach Abschluss zeigt
  ein Hinweis den Speicherort aller Ergebnisse an.

### 2.2 Headless-Modus (Kommandozeile)

Wer direkt per Terminal arbeiten möchte, startet einfach:

```bash
python PiLiDAR.py
```

Der Lauf erzeugt automatisch alle Daten.  Abbruch ist jederzeit mit `Strg+C`
möglich, der Controller stoppt den Motor, deaktiviert den A4988 über den
ENABLE-Pin und fährt den LiDAR geordnet herunter.

### 2.3 Was im Hintergrund passiert

1. **Initialisierung** – Das Skript erstellt den Scan-Ordner, schaltet das
   Motor-Relais ein und aktiviert den A4988 über den ENABLE-Pin.
2. **Kamera-Kalibrierung** – Die Belichtung wird automatisch bestimmt, sodass
   die Bilder für das Panorama konsistent sind.
3. **Fotoaufnahme** – Der Stepper dreht den Aufbau in gleichmäßigen 90°-Schritten
   (die Pulsweite wird per PWM erzeugt) und löst die Kamera aus.
4. **LiDAR-Scan** – Der STL27L liefert über USB die Messpakete.  Der Motor wird
   synchron per PWM-Impulsen bewegt, bis der gesamte Scanwinkel abgefahren ist.
5. **Panorama & Punktwolken** – Hugin erstellt aus den Bildern ein Panorama, das
   anschließend genutzt wird, um die Punktwolke farblich einzufärben.
6. **Aufräumen** – LiDAR-Motor und Stepper werden gestoppt, der A4988 deaktiviert
   und das Relais freigegeben.  Dadurch ist der Aufbau sofort wieder sicher.

---

## 3. Ergebnisdaten und Ordnerstruktur

Jeder Scan landet in einem eigenen Unterordner im Verzeichnis `scans/`.

```
scans/<SCAN-ID>/
├── img/                 # Einzelbilder für das Panorama
├── tmp/                 # Temporäre Dateien während der Verarbeitung
├── logs/                # Statusmeldungen und Zusatzinformationen
├── lidar/               # Legacy-Ordner (ältere Rohformate)
├── <SCAN-ID>_lidar.pkl  # Kompletter LiDAR-Rohdatensatz
├── <SCAN-ID>_intensity.ply  # Punktwolke mit Intensitätsfärbung
├── <SCAN-ID>_vertex.ply     # Punktwolke mit Bildfarben (falls Panorama vorhanden)
└── <SCAN-ID>_blended_fused.jpg  # Panorama aus den Kamerabildern
```

Alle Dateien werden automatisch erzeugt und mit sprechenden Namen versehen.
Eine manuelle Nacharbeit ist nicht notwendig.

## 4. Welche LiDAR-Daten werden gespeichert?

Der neue Treiber speichert den kompletten Informationsgehalt jeder Messung:

- `timestamp`: Zeitstempel des Pakets in Millisekunden
- `speed`: Drehgeschwindigkeit des Sensors
- `angles_rad`: 12 Start-/Endwinkel innerhalb des Pakets (Radiant)
- `distances_mm`: 12 Distanzwerte in Millimetern
- `intensities`: 12 Intensitätswerte (0–255)
- `cartesian`: bereits umgerechnete X/Y-Koordinaten pro Messpunkt
- `z_angle`: aktuelle Plattformposition des Steppers für dieses Paket
- `z_angles`: Liste aller vertikalen Drehwinkel des Scans
- `angular`: polare Punktlisten für jede Ebene
- `cartesian_list`: kartesische Punktlisten für jede Ebene

Diese Daten ermöglichen es, später eigene Auswertungen (z. B. Filter oder
Registrierungen) aufzubauen, ohne erneut messen zu müssen.

---

## 5. Wichtige Skripte und Ordner

- `PiLiDAR.py` – Startskript, bietet Headless- und GUI-Modus.
- `lib/scan_controller.py` – zentrale Steuerlogik, koordiniert Motor, LiDAR,
  Kamera und Nachbearbeitung.
- `lib/lidar_driver.py` – neuer LiDAR-Treiber mit Strom-/Motorsteuerung,
  umfassender Datenspeicherung und Stop-Mechanismus.
- `lib/gui.py` – Tkinter Oberfläche für einfache Bedienung.
- `lib/pointcloud.py` – erzeugt Punktwolken (Intensität + Vertexfarben) und
  kapselt sämtliche Nachbearbeitungsschritte.
- `lib/config.py` – liest `config.json`, verwaltet Pfade, GPIO-Pins und sorgt
  für eine saubere Ordnerstruktur pro Scan.
- `old/` – enthält archivierte Testskripte (`meshing_test.py`, `process_3D.py`,
  usw.) sowie die ursprünglichen Installationshinweise.

Alle Module sind im Code umfangreich kommentiert. Dadurch lässt sich jeder
Arbeitsschritt nachvollziehen.

---

## 6. Hardwarehinweise

- **Stepper & Getriebe**: Der Motor läuft mit 16-fach Mikrostepping.  Die
  Schrittimpulse werden als 50 % PWM über den STEP-Pin erzeugt.  Die Frequenz
  ergibt sich aus `STEP_DELAY` in der `config.json` und sorgt für einen sehr
  gleichmäßigen Lauf des getriebeübersetzten Motors.
- **Enable-Pin**: Der A4988 ist standardmäßig deaktiviert (Enable = High).  Die
  Software zieht den Pin bei Bedarf auf Low und gibt den Motor nach dem Scan
  sofort wieder frei – so bleibt er handwarm und sicher.
- **LiDAR-Stromversorgung**: Der Sensor wird ausschließlich über USB versorgt,
  zusätzliche Relais oder I²C-Module sind nicht nötig.
- **Panorama**: Für bestmögliche Ergebnisse sollten alle Bilder mit identischer
  Belichtung aufgenommen werden.  Die automatisch ermittelten Werte sind ein
  guter Ausgangspunkt; bei Bedarf lassen sie sich im GUI anpassen.

![PiLiDAR v2](images/pilidar_covershot_v2.jpg)

---

## 7. Fehlersuche & Tipps

- **LiDAR startet nicht**: Prüfe die serielle Verbindung (`/dev/ttyUSB0`) und
  ob `sudo chmod a+rw /dev/ttyUSB0` gesetzt wurde.
- **Punktwolke fehlt**: Stelle sicher, dass `ENABLE_3D` in `config.json` oder
  in der GUI aktiviert ist. Für Vertexfarben muss zusätzlich ein Panorama
  erzeugt werden.
- **Ruckeln während des Scans**: Reduziere `TARGET_SPEED` oder erhöhe
  `STEP_DELAY` leicht. Beide Werte lassen sich über die GUI anpassen.

---

## 8. Lizenz

PiLiDAR steht unter der MIT-Lizenz (siehe `LICENSE.md`). Beiträge und
Verbesserungen sind ausdrücklich willkommen.
