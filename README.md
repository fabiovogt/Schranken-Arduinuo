# Schrankenleuchte

Arduino-Projekt fuer eine zeitgesteuerte Warn- bzw. Verbotsanzeige mit DS3231-Echtzeituhr.

Die LED zeigt anhand von Uhrzeit, Sommerzeit, Wochenenden und NRW-Feiertagen an, ob ein Zustand aktuell erlaubt oder verboten ist:

- erlaubt: LED aus
- 30 Sekunden vor Ende eines erlaubten Zeitfensters: LED blinkt langsam
- verboten: LED leuchtet dauerhaft
- RTC fehlt oder Uhrzeit ungueltig: LED blinkt schnell als Fehleranzeige

## Hardware

- Arduino-kompatibles Board
- DS3231 RTC-Modul ueber I2C
- LED an `D3`
- optionale Spiegelung auf die Onboard-LED `D13`

## Verwendete Bibliotheken

- `Wire`
- `RTClib`
- `avr/pgmspace.h`

## Zeitmodell

Die RTC wird bewusst in lokaler deutscher Zeit betrieben, nicht in UTC. Die Sommerzeit wird im Sketch nach EU-Regel berechnet.

Erlaubte Zeitfenster:

- Sommer, Werktag: `05:30-13:00` und `15:00-23:00`
- Sommer, Wochenende/Feiertag: `07:00-13:00` und `15:00-23:00`
- Winter, Werktag: `05:30-22:00`
- Winter, Wochenende/Feiertag: `07:00-13:00` und `15:00-23:00`

Zusatzregeln:

- Feiertage sind fuer NRW im Sketch fest hinterlegt.
- Die Feiertagstabelle deckt die Jahre `2026` bis `2035` ab.
- Ausserhalb dieses Bereichs gelten nur Wochenenden als Sondertage.

## Konfiguration

Im Sketch gibt es drei Schalter:

- `SET_RTC_ON_UPLOAD`: setzt die RTC beim Flashen einmalig auf die Compile-Zeit
- `ENABLE_SERIAL_DEBUG`: aktiviert serielle Statusausgaben und Befehle
- `USE_DEBUG_LED`: spiegelt den LED-Zustand auf `D13`

## RTC setzen

Bei aktiviertem Serial-Debug kann die Uhr per serieller Konsole gesetzt werden:

```text
SET YYYY-MM-DD HH:MM:SS
```

Beispiel:

```text
SET 2026-04-03 19:30:00
```

## Verhalten im Betrieb

Der Sketch pollt die RTC alle 200 ms und aktualisiert die LED blockierungsfrei, damit Blinkmuster sauber bleiben. Wenn die RTC nicht gefunden wird oder `lostPower()` meldet und die Zeit noch nicht neu bestaetigt wurde, geht das System in einen latched Fehlerzustand, bis die Uhr gueltig gesetzt wird.

## Flashen

1. Projekt in der Arduino IDE oder PlatformIO als Arduino-Sketch oeffnen.
2. Benoetigte Bibliothek `RTClib` installieren.
3. Board und Port auswaehlen.
4. Sketch hochladen.
5. Falls noetig, RTC ueber den seriellen Monitor setzen.

## Projektstruktur

```text
.
├── ForbiddenIndicator/
│   └── ForbiddenIndicator.ino
└── README.md
```
