# Handleiding: SDC `main_experimental.py`

Zelfrijdende kart met rijstrookdetectie, verkeerstekens en YOLO-objectherkenning.

---

## Inhoudsopgave

1. [Overzicht](#overzicht)
2. [Vereisten](#vereisten)
3. [Camera-index vinden](#camera-index-vinden)
4. [Arduino-poort vinden](#arduino-poort-vinden)
5. [Opstarten](#opstarten)
6. [Bediening tijdens gebruik](#bediening)
7. [Configuratie (parameters)](#configuratie)
8. [Hoe het werkt](#hoe-het-werkt)
   - [Rijstrookdetectie](#rijstrookdetectie)
   - [Verkeersregeling (stoplichten/stopborden)](#verkeersregeling)
   - [YOLO-objectherkenning](#yolo-objectherkenning)
   - [Kart-aansturing](#kart-aansturing)
9. [Weergave en debugging](#weergave-en-debugging)
10. [Veelvoorkomende problemen](#veelvoorkomende-problemen)

---

## Overzicht

Dit script bestuurt een zelfrijdende kart. Het combineert drie hoofdfuncties:

- **Rijstrookdetectie** — detecteert witte/gele rijstrookmarkering via kleurmaskering en Hough-lijndetectie, en berekent een stuurhoek.
- **Verkeersherkenning** — reageert op stopborden (10 seconden stoppen) en rode/groene lichten (stoppen totdat groen wordt gedetecteerd).
- **Hardware-uitvoer** — stuurt een Arduino-gebaseerde `KartController` aan via USB.

Het script werkt in twee modi: **cameramodus** (live) en **videomodus** (opgenomen bestand).

---

## Vereisten

- Python 3.8+
- Bibliotheken: `ultralytics`, `opencv-python`, `numpy`, `tkinter`
- Eigen module: `kart_control` (met `KartController` en `angle_to_fraction`)
- YOLO-model: `object_models/best.pt` (alleen nodig als YOLO ingeschakeld is)
- Hardware: Arduino op `/dev/ttyUSB0` (optioneel, zie `KART_ENABLED`)

---

## Camera-index vinden

De camera-index is het getal dat je meegeeft aan het script (bijv. `camera 0` of `camera 1`). Op Linux worden camera's aangeduid als `/dev/video0`, `/dev/video1`, enz. De index in het script komt overeen met dat nummer.

### Methode 1 — Apparaatbestanden bekijken (Linux)

```bash
ls /dev/video*
```

Typische uitvoer:
```
/dev/video0   /dev/video1   /dev/video2
```

Ingebouwde webcam is meestal `/dev/video0` (index `0`). Een externe USB-camera is dan `/dev/video1` of `/dev/video2` (index `1` of `2`).

### Methode 2 — `v4l2-utils` (aanbevolen, geeft naam + info)

```bash
# Installeer het hulpprogramma (eenmalig)
sudo apt install v4l-utils

# Toon alle camera's met naam
v4l2-ctl --list-devices
```

Voorbeelduitvoer:
```
USB Camera (usb-0000:00:14.0-2):
    /dev/video0
    /dev/video1

Integrated Camera (usb-0000:00:14.0-6):
    /dev/video2
```

Gebruik de eerste `/dev/videoX` per apparaat als index. In dit voorbeeld: USB-camera = index `0`, ingebouwde camera = index `2`.

### Methode 3 — Snel testen met Python

Probeer meerdere indices door en kijk welke een beeld geeft:

```python
import cv2

for i in range(5):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        ret, frame = cap.read()
        print(f"Index {i}: {'OK - beeld ontvangen' if ret else 'open maar geen beeld'}")
        cap.release()
    else:
        print(f"Index {i}: niet beschikbaar")
```

Sla dit op als `check_cameras.py` en voer het uit:

```bash
python check_cameras.py
```

### Methode 4 — Grafisch bekijken met `ffplay`

```bash
# Camera 0 bekijken
ffplay /dev/video0

# Camera 1 bekijken
ffplay /dev/video1
```

Sluit af met `q`. De camera die het juiste beeld toont is de juiste index.

### Methode 5 — `cheese` (grafische tool)

```bash
sudo apt install cheese
cheese
```

Cheese toont een cameralijst. De volgorde komt overeen met de device-nummering.

---

### Tips bij meerdere camera's

- Sluit externe USB-camera's één voor één aan en voer `ls /dev/video*` uit na elke aansluiting. Het nieuw verschenen device is de camera.
- Bij USB-hubs kan de index wisselen bij herstart. Gebruik dan een udev-regel om een vaste naam toe te wijzen:

```bash
# Bekijk USB-info van de camera
udevadm info --name=/dev/video0 --attribute-walk | grep -E "idVendor|idProduct|serial"
```

Maak daarna `/etc/udev/rules.d/99-camera.rules` aan:

```
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="XXXX", ATTRS{idProduct}=="YYYY", SYMLINK+="video_kart"
```

Gebruik dan `/dev/video_kart` in plaats van een index, en pas `_open_cap()` in het script aan.

---

## Arduino-poort vinden

De Arduino communiceert via een seriële USB-poort. Op Linux verschijnt dit als `/dev/ttyUSB0`, `/dev/ttyUSB1`, `/dev/ttyACM0`, enz. De juiste poort moet je invullen bij `KART_PORT` bovenaan het script.

### Methode 1 — Apparaatbestanden bekijken

```bash
ls /dev/tty{USB,ACM}*
```

Voorbeelduitvoer:
```
/dev/ttyUSB0   /dev/ttyACM0
```

Arduino Uno/Mega verschijnen meestal als `/dev/ttyACM0`. Arduino-klonen met een CH340/CP2102-chip verschijnen als `/dev/ttyUSB0`.

### Methode 2 — Poort herkennen door in/uitpluggen

De betrouwbaarste methode: kijk wat er verschijnt nadat je de Arduino aansluit.

```bash
# Stap 1: lijst vóór aansluiten
ls /dev/tty{USB,ACM}* 2>/dev/null

# Sluit nu de Arduino aan via USB

# Stap 2: lijst ná aansluiten — het nieuwe item is de Arduino
ls /dev/tty{USB,ACM}* 2>/dev/null
```

Of in één commando dat het verschil toont:

```bash
before=$(ls /dev/tty{USB,ACM}* 2>/dev/null); read -p "Sluit Arduino aan en druk Enter..."; after=$(ls /dev/tty{USB,ACM}* 2>/dev/null); diff <(echo "$before") <(echo "$after")
```

### Methode 3 — `dmesg` (systeemlog)

Sluit de Arduino aan en voer daarna uit:

```bash
dmesg | tail -20
```

Zoek naar regels zoals:
```
usb 1-2: New USB device found, idVendor=2341, idProduct=0043
ch341-uart converter now attached to ttyUSB0
```

Het laatste woord (`ttyUSB0`) is de poort. Zet `/dev/` ervoor: `/dev/ttyUSB0`.

### Methode 4 — `arduino-cli`

```bash
arduino-cli board list
```

Uitvoer:
```
Port          Protocol  Type              Board Name
/dev/ttyACM0  serial    Serial Port (USB) Arduino Uno
```

De kolom `Port` is direct bruikbaar als `KART_PORT`.

### Methode 5 — Python testen

Sla dit op als `check_ports.py`:

```python
import serial.tools.list_ports

for port in serial.tools.list_ports.comports():
    print(f"{port.device:20s} — {port.description}  [{port.manufacturer}]")
```

```bash
python check_ports.py
```

Voorbeelduitvoer:
```
/dev/ttyUSB0         — USB-Serial Controller  [QinHeng Electronics]
/dev/ttyACM0         — Arduino Uno             [Arduino LLC]
```

### Rechten instellen (eenmalig)

Als je een `Permission denied`-fout krijgt bij het openen van de poort:

```bash
# Voeg je gebruiker toe aan de groep 'dialout' (permanent)
sudo usermod -aG dialout $USER
# Log daarna opnieuw in om het effect te activeren

# Of tijdelijk voor de huidige sessie:
sudo chmod a+rw /dev/ttyUSB0
```

### Vaste poortnaam instellen (udev-regel)

Standaard kan de poort wisselen (`ttyUSB0` wordt `ttyUSB1`) als je andere USB-apparaten aansluit. Stel een vaste naam in:

```bash
# Bekijk de USB-info van de Arduino
udevadm info --name=/dev/ttyUSB0 --attribute-walk | grep -E "idVendor|idProduct"
```

Maak `/etc/udev/rules.d/99-arduino.rules` aan:

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0043", SYMLINK+="ttyARDUINO"
```

Herlaad de regels:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Pas daarna `KART_PORT` aan in het script:

```python
KART_PORT = "/dev/ttyARDUINO"
```

De poort is nu altijd dezelfde, ongeacht andere aangesloten USB-apparaten.

---

## Opstarten

```bash
# Cameramodus (standaard camera-index 1)
python main_experimental.py camera

# Cameramodus met specifieke camera
python main_experimental.py camera 0

# Videomodus
python main_experimental.py video pad/naar/video.mp4
```

---

## Bediening

| Toets | Actie |
|-------|-------|
| `q`   | Programma afsluiten |
| `e`   | Noodstop (emergency stop) |
| `o`   | YOLO-detectie aan/uitzetten |
| `p`   | Pauze / hervat (alleen videomodus) |

---

## Configuratie

Alle instellingen staan bovenaan het bestand als constanten. De belangrijkste:

### Hardware

| Parameter | Standaard | Beschrijving |
|-----------|-----------|--------------|
| `KART_PORT` | `/dev/ttyUSB0` | Seriële poort van de Arduino |
| `KART_ENABLED` | `True` | Zet op `False` voor simulatiemodus zonder hardware |
| `YOLO_ENABLED` | `False` | Standaard staat YOLO uit (kost veel rekenkracht) |

### ROI (regio van interesse)

De ROI is een trapeziumvorm die het interessante gedeelte van het beeld afbakent. Tweak dit als rijstroken gemist worden.

| Parameter | Standaard | Beschrijving |
|-----------|-----------|--------------|
| `ROI_TOP_Y` | `0.5` | Bovenkant ROI (fractie van beeldhoogte) |
| `ROI_BOTTOM_Y` | `0.95` | Onderkant ROI |
| `ROI_TOP_LEFT` | `0.10` | Linkerbovenhoek (fractie van beeldbreedte) |
| `ROI_TOP_RIGHT` | `0.90` | Rechterbovenhoek |
| `ROI_STEER_SHIFT` | `0.002` | Hoeveel de ROI meeschuift per stuurgraad |

### Rijstrookdetectie

| Parameter | Standaard | Beschrijving |
|-----------|-----------|--------------|
| `HOUGH_THRESHOLD` | `20` | Minimaal aantal lijnpunten voor Hough |
| `HOUGH_MIN_LEN` | `30` | Minimale lijnlengte (pixels) |
| `HOUGH_MAX_GAP` | `80` | Maximale onderbreking in een lijn (pixels) |
| `HOUGH_MIN_SLOPE` | `0.40` | Filters bijna-horizontale lijnen weg |
| `DOT_MIN_AREA` | `5000` | Blobs kleiner dan dit worden als wegkattenogen gefilterd |
| `MAX_HISTORY` | `7` | Aantal frames voor stuurhoek-smoothing |

### Verkeersregeling

| Parameter | Standaard | Beschrijving |
|-----------|-----------|--------------|
| `STOP_HOLD_DURATION` | `10.0 s` | Hoe lang stoppen bij stopbord |
| `BRAKE_RAMP_DURATION` | `1.0 s` | Remtijd voordat volledig stilstand |
| `DETECTION_CONFIDENCE` | `0.5` | Minimale YOLO-betrouwbaarheid voor reactie |

---

## Hoe het werkt

### Rijstrookdetectie

De detectie verloopt in de klasse `LaneDetector` (aparte thread) via deze stappen:

1. **ROI-masker** — een adaptief trapezium wordt berekend op basis van de huidige stuurhoek (`_roi_mask`). Zo volgt het zoekgebied de rijrichting.

2. **Kleurmaskering** (`_edge_image`) — detecteert witte en gele rijstrookmarkeringen in HLS-kleurruimte. Op het witte masker worden kleine ronde blobs (wegkattenogen/reflectors) verwijderd via:
   - Morfologische opening (7×7 ellips)
   - `connectedComponentsWithStats` met `DOT_MIN_AREA` als drempelwaarde

3. **Randdetectie** — CLAHE-verbetering van het L-kanaal → Gaussisch waas → Canny → `AND` met kleurmasker.

4. **Hough-lijndetectie** (`HoughLinesP`) — vindt lijnstukken. Lijnen worden ingedeeld als links (negatieve helling, links van midden) of rechts (positieve helling, rechts van midden).

5. **Polynoomfitting** (`_fit_poly`) — past een tweedegraadspolynoom aan door de lijnpunten. Kleine verticale spans worden genegeerd.

6. **Smoothing** (`_smooth_poly`) — combineert de nieuwe polynoom met de vorige via exponentieel voortschrijdend gemiddelde (alpha afhankelijk van betrouwbaarheid).

7. **Stuurhoek** (`_compute_steering`) — berekent het verschil tussen het rijstrookcentrum en het beeldmidden, normaliseert dit en past historische smoothing toe.

```
Stuurhoek = 0°   → rechtdoor
Stuurhoek > 0°   → rechts
Stuurhoek < 0°   → links
Maximum:   ±90°
```

---

### Verkeersregeling

De toestandsmachine `TrafficStateController` doorloopt deze toestanden:

```
DRIVING
  │  stopbord of rood licht gedetecteerd
  ▼
BRAKING  (1 seconde remmen)
  │
  ├─→ STOPPED_SIGN  → na 10 seconden → RESUMING
  │
  └─→ STOPPED_RED   → zodra groen gedetecteerd → RESUMING
                                                      │
                                                      ▼
                                                   DRIVING
```

Detectie-labels (hoofdletterongevoelig):

| Reactie | Labels |
|---------|--------|
| Stoppen | `stop-sign` |
| Remmen (rood) | `red` |
| Rijden (groen) | `green` |

---

### YOLO-objectherkenning

YOLO draait in een aparte thread (`YoloDetector`) om de hoofdlus niet te vertragen. Standaard staat YOLO **uit** (`YOLO_ENABLED = False`). Zet het aan met de `o`-toets of zet `YOLO_ENABLED = True` bovenaan het bestand.

Het model (`object_models/best.pt`) moet getraind zijn op in elk geval de labels `stop-sign`, `red` en `green`.

---

### Kart-aansturing

De `KartController` (externe module) ontvangt twee soorten commando's:

| Methode | Beschrijving |
|---------|--------------|
| `kart.steer(fraction)` | Stuurwaarde van -1.0 (volledig links) tot +1.0 (volledig rechts) |
| `kart.brake(1.0)` | Volledig remmen |
| `kart.release()` | Remmen loslaten |
| `kart.estop()` | Noodstop |

De stuurhoek in graden wordt omgezet naar een fractie via `angle_to_fraction()`.

---

## Weergave en debugging

Er zijn drie vensters:

| Venster | Inhoud |
|---------|--------|
| `SDC View [EXP]` | Hoofdweergave: rijstroken, stuurlijn, FPS, verkeersstatus |
| `Lane Mask` | Kleurmasker binnen de ROI (wit/geel) — handig voor afstelling |
| Tkinter-venster | Grafisch stuurwiel met hoek en richting |

Op het hoofdscherm:
- **Groene lijnen** — daadwerkelijk gedetecteerde rijstroken
- **Blauwgrijze stippellijnen** — geschatte rijstrook (spiegel van de andere kant)
- **Geel gestippeld trapezium** — actieve ROI
- **Gekleurde pijl** — stuurrichting (groen = rechtdoor, cyaan = bocht, blauw = scherpe bocht)
- **Verkeersbanner** — huidige toestand van de verkeersregelaar

Consolelogs verschijnen elke 10 frames (`PRINT_EVERY = 10`).

---

## Veelvoorkomende problemen

| Probleem | Mogelijke oorzaak | Oplossing |
|----------|-------------------|-----------|
| Rijstroken worden niet gedetecteerd | ROI dekt de lijnen niet | Pas `ROI_TOP_Y`, `ROI_TOP_LEFT/RIGHT` aan |
| Wegkattenogen verstoren detectie | `DOT_MIN_AREA` te laag | Verhoog `DOT_MIN_AREA` (bijv. naar 8000) |
| Stuurhoek springt veel | Te weinig smoothing | Verhoog `MAX_HISTORY` of verlaag `ALPHA_FRESH` |
| Arduino niet gevonden | Verkeerde poort of niet aangesloten | Pas `KART_PORT` aan of zet `KART_ENABLED = False` |
| YOLO erg traag | Zware GPU-last | Laat `YOLO_ENABLED = False` en schakel alleen in als nodig |
| Camera opent niet | Verkeerd device-nummer | Probeer `camera 0` in plaats van `camera 1` |