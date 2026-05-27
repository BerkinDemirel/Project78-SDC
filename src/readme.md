# Kart Controller — Arduino & Web-testpanel

Handleiding voor de Arduino-firmware (`src/main.cpp`) en het bijbehorende web-testpanel (`debug_webui/steering_test_panel.html`).

---

## Inhoudsopgave

1. [Overzicht](#overzicht)
2. [Benodigdheden](#benodigdheden)
3. [Arduino-poort vinden](#arduino-poort-vinden)
4. [Firmware installeren](#firmware-installeren)
5. [Web-testpanel gebruiken](#web-testpanel-gebruiken)
6. [Stuurkalibratie](#stuurkalibratie)
7. [Serieel protocol](#serieel-protocol)
8. [Tuning-parameters](#tuning-parameters)
9. [Veelvoorkomende problemen](#veelvoorkomende-problemen)

---

## Overzicht

De firmware bestuurt twee BTS7960B motorcontrollers:

| Kanaal | Type | Feedback |
|--------|------|----------|
| Stuur | Gesloten lus (P-regelaar) | Potmeter op `A0` |
| Rem | Open lus (tijdgestuurd) | Geen sensor |

Bij opstarten staat de rem standaard **volledig aangetrokken**. De host (Python-script of testpanel) moet `{"cmd":"release"}` sturen om te beginnen. Een hardware watchdog reset de Arduino als de firmware vastloopt.

---

## Benodigdheden

- Arduino Uno (of compatible bord met ATmega328P)
- Arduino IDE 2.x of `arduino-cli`
- Bibliotheek: **ArduinoJson** (Benoit Blanchon)
- Google Chrome of Microsoft Edge (voor het web-testpanel)

---

## Arduino-poort vinden

### Methode 1 — Apparaatbestanden bekijken

```bash
ls /dev/tty{USB,ACM}*
```

Arduino Uno → `/dev/ttyACM0`  
Arduino-kloon (CH340/CP2102) → `/dev/ttyUSB0`

### Methode 2 — Voor/na aansluiten vergelijken

```bash
# Voer uit vóór aansluiten, sluit aan, druk Enter
before=$(ls /dev/tty{USB,ACM}* 2>/dev/null); read -p "Sluit Arduino aan en druk Enter..."; after=$(ls /dev/tty{USB,ACM}* 2>/dev/null); diff <(echo "$before") <(echo "$after")
```

### Methode 3 — dmesg

```bash
# Sluit Arduino aan, voer dan uit:
dmesg | tail -15
```

Zoek naar: `ttyUSB0` of `ttyACM0` aan het einde van een regel.

### Methode 4 — Python

```python
import serial.tools.list_ports
for p in serial.tools.list_ports.comports():
    print(f"{p.device:20s} — {p.description}  [{p.manufacturer}]")
```

### Rechten instellen (eenmalig)

```bash
sudo usermod -aG dialout $USER
# Daarna opnieuw inloggen
```

### Vaste poortnaam via udev (optioneel)

```bash
# Zoek idVendor en idProduct op
udevadm info --name=/dev/ttyUSB0 --attribute-walk | grep -E "idVendor|idProduct"
```

Maak `/etc/udev/rules.d/99-arduino.rules` aan:

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0043", SYMLINK+="ttyARDUINO"
```

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Gebruik daarna `/dev/ttyARDUINO` als poort.

---

## Firmware installeren

### Stap 1 — ArduinoJson installeren

**Via Arduino IDE:**  
Tools → Manage Libraries → zoek `ArduinoJson` → Install

**Via CLI:**

```bash
arduino-cli lib install "ArduinoJson"
```

### Stap 2 — Uploaden

**Via Arduino IDE:**

1. Open `kart_controller.ino`
2. Tools → Board → **Arduino Uno**
3. Tools → Port → kies de juiste poort (zie boven)
4. Klik **Upload** (→)

**Via CLI:**

```bash
arduino-cli compile --fqbn arduino:avr:uno kart_controller/
arduino-cli upload  --fqbn arduino:avr:uno --port /dev/ttyACM0 kart_controller/
```

### Stap 3 — Verbinding controleren

Open de Serial Monitor op **115200 baud**. Je ziet:

```json
{"status":"ready","brake":"applied"}
{"sp":606,"st":606,"bf":1.00,"bt":1.00,"es":false}
```

Als de telemetrie binnenkomt is de firmware correct actief.

---

## Web-testpanel gebruiken

Het bestand `steering_test_panel.html` communiceert via **WebSerial** direct met de Arduino. Geen server of installatie nodig.

> **Vereiste browser:** Google Chrome of Microsoft Edge (versie 89+).  
> Firefox en Safari ondersteunen WebSerial **niet**.

### Openen

1. Open Chrome of Edge
2. Sleep `steering_test_panel.html` naar het browservenster, of gebruik **Bestand → Bestand openen**
3. Klik **⬡ Connect serial**
4. Kies de juiste poort in het dialoogvenster
5. **CONNECTED** verschijnt groen en telemetrie stroomt binnen

### Schermoverzicht

| Element | Beschrijving |
|---------|--------------|
| **pot (raw)** | Live potmeterwaarde van het stuur (0–1023) |
| **target (raw)** | Doelpositie die de firmware nastreeft |
| **brake fraction** | Geschatte remstand (0.00 = los, 1.00 = volledig) |
| **e-stopped** | `YES` als noodstop actief is |
| **Manual steer slider** | Stuurt direct `steer`-commando (−1.0 tot +1.0) |
| **↑ Release brake** | Stuurt `{"cmd":"release"}` — rem los |
| **⊕ Centre** | Zet stuurhoek naar 0.0 |
| **⊗ E-STOP** | Volledige rem + vergrendeld |
| **Boogmeter** | Groen = huidige positie · Amber = target |
| **Sequence tests** | Automatische rijpatronen (sweep, step, slalom, ramp) |
| **Serial log** | Alle verzonden (groen) en ontvangen (blauw) berichten |

---

## Stuurkalibratie

De firmware heeft drie vaste potmeterwaarden nodig om de stuurhoek correct te berekenen:

| Constante | Betekenis |
|-----------|-----------|
| `STEER_POT_MIN` | Potmeterwaarde bij volledig **rechts** sturen |
| `STEER_POT_MAX` | Potmeterwaarde bij volledig **links** sturen |
| `STEER_POT_CTR` | Potmeterwaarde bij de gewenste **rechte** uitstand |

> De namen `MIN`/`MAX` volgen de fysieke draairichting van de potmeter, niet de rijrichting.

---

### Voorbereiding

- Arduino aangesloten en firmware actief
- Testpanel open in Chrome/Edge en verbonden (**CONNECTED**)
- Kart op een veilige plek (wielen vrij van de grond, of genoeg ruimte)

---

### Stap 1 — Rem lossen

Klik op **↑ Release brake**.  
Het stuur beweegt niet zolang de rem aangetrokken is.

---

### Stap 2 — Volledig naar rechts → `STEER_POT_MIN`

1. Zet de slider helemaal naar **rechts** (`+1.0`)
2. Wacht tot het stuur volledig uitgeslagen is en stilstaat (~1–2 sec)
3. Lees de waarde af bij **pot (raw 0–1023)**
4. Noteer de waarde

```
STEER_POT_MIN = [jouw waarde]    ← volledig rechts
```

Standaard voorbeeld: `452`

---

### Stap 3 — Volledig naar links → `STEER_POT_MAX`

1. Zet de slider helemaal naar **links** (`−1.0`)
2. Wacht tot het stuur volledig uitgeslagen is en stilstaat
3. Lees de waarde af bij **pot (raw 0–1023)**
4. Noteer de waarde

```
STEER_POT_MAX = [jouw waarde]    ← volledig links
```

Standaard voorbeeld: `743`

---

### Stap 4 — Centreren → `STEER_POT_CTR`

1. Klik op **⊕ Centre** (slider naar 0.0)
2. Kijk naar de kart: staan de wielen recht?
3. Zo niet: pas de slider kleine stapjes aan (`+0.02` / `−0.02`) tot de wielen visueel recht staan
4. Lees de waarde af bij **pot (raw 0–1023)**
5. Noteer de waarde

```
STEER_POT_CTR = [jouw waarde]    ← rechte uitstand
```

Standaard voorbeeld: `606`

> Het centrum is zelden het rekenkundig gemiddelde van min en max. Gebruik altijd de **visueel rechte positie** als referentie.

---

### Stap 5 — Waarden aanpassen in de firmware

Open `kart_controller.ino` en zoek bovenaan:

```cpp
int STEER_POT_MIN = 452;
int STEER_POT_CTR = 606;
int STEER_POT_MAX = 743;
```

Vervang de getallen door jouw gemeten waarden en upload opnieuw (zie [Firmware installeren](#firmware-installeren)).

---

### Stap 6 — Verificatie

1. Verbind opnieuw via het testpanel
2. Slider naar `+1.0` → groene naald wijst rechts, amber naald volgt
3. Slider naar `−1.0` → beide naalden naar links
4. **⊕ Centre** → kart staat recht, **pot**-waarde ≈ `STEER_POT_CTR`

---

## Serieel protocol

Alle communicatie verloopt via newline-afgebakende JSON op **115200 baud**.

### Commando's (host → Arduino)

| Commando | Beschrijving |
|----------|--------------|
| `{"cmd":"steer","value":0.35}` | Stuur −1.0 (links) tot +1.0 (rechts) |
| `{"cmd":"brake","value":0.8}` | Rem 0.0 (los) tot 1.0 (volledig) — geen vergrendeling |
| `{"cmd":"release"}` | Rem volledig lossen + noodstop opheffen |
| `{"cmd":"estop"}` | Volledig remmen + vergrendelen |
| `{"cmd":"brake_reset"}` | Hersynchroniseer open-lus schatting naar 0.0 (gebruik na handmatig lossen) |
| `{"cmd":"enable","axis":"steer","on":true}` | Stuurkanaal in/uitschakelen |
| `{"cmd":"calibrate"}` | Automatisch min/max kalibreren (rijdt naar beide uitersten) |

### Telemetrie (Arduino → host, elke 100 ms)

```json
{"sp":606,"st":606,"bf":0.00,"bt":0.00,"es":false}
```

| Veld | Betekenis |
|------|-----------|
| `sp` | Huidige potmeterwaarde (steer position) |
| `st` | Doelpositie (steer target) |
| `bf` | Huidige remstand schatting (brake fraction) |
| `bt` | Rem doelstand (brake target) |
| `es` | Noodstop actief (`true`/`false`) |

---

## Tuning-parameters

### Stuur

| Parameter | Standaard | Beschrijving |
|-----------|-----------|--------------|
| `STEER_DEADBAND` | `8` | Foutmarge waarbinnen de motor stopt (in pot-eenheden) |
| `STEER_PWM_MIN` | `60` | Minimaal PWM-signaal (voorkomt stotteren) |
| `STEER_PWM_MAX` | `200` | Maximaal PWM-signaal |
| `STEER_KP` | `1.8` | P-versterking van de regelaar |

### Rem

| Parameter | Standaard | Beschrijving |
|-----------|-----------|--------------|
| `BRAKE_PWM_MIN` | `80` | Minimaal PWM voor de remactuator |
| `BRAKE_PWM_MAX` | `220` | Maximaal PWM voor de remactuator |
| `BRAKE_APPLY_MS` | `600` | Tijd (ms) om van volledig los naar volledig aangetrokken te gaan |
| `BRAKE_RELEASE_MS` | `600` | Tijd (ms) om van volledig aan naar volledig los te gaan |

---

## Veelvoorkomende problemen

| Probleem | Mogelijke oorzaak | Oplossing |
|----------|-------------------|-----------|
| Geen telemetrie na upload | Verkeerde baudrate in Serial Monitor | Zet op **115200 baud** |
| `Permission denied` op poort | Gebruiker niet in `dialout` groep | `sudo usermod -aG dialout $USER` en opnieuw inloggen |
| Stuur beweegt niet | Rem nog aangetrokken | Stuur `{"cmd":"release"}` of klik **↑ Release brake** |
| Stuur stuurt de verkeerde kant op | `MIN`/`MAX` omgewisseld | Wissel `STEER_POT_MIN` en `STEER_POT_MAX` om |
| Stuur bereikt target niet | `STEER_KP` te laag | Verhoog stapsgewijs (bijv. `1.8` → `2.2`) |
| Stuur blijft wiebelen | `STEER_DEADBAND` te klein | Verhoog naar bijv. `15` |
| WebSerial werkt niet | Verkeerde browser | Gebruik Chrome of Edge, open via `file://` of `localhost` |
| Arduino reset zichzelf | Watchdog getriggerd (firmware vastgelopen) | Normaal gedrag — controleer of de loop niet blokkeert |
| Rem komt niet los | Open-lus schatting niet gesynchroniseerd | Stuur `{"cmd":"brake_reset"}` na handmatig lossen |