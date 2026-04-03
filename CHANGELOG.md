# Changelog

## Unreleased

- Dokumentation des Projekts in `README.md` ergaenzt, inklusive Hardware, Zeitfenster, RTC-Setup und Betriebsverhalten.
- RTC-Handling stabilisiert, darunter robusteres serielles Setzen der Uhr und saubere Behandlung ungueltiger RTC-Zustaende.
- Automatische Sommer-/Winterzeit fuer Deutschland implementiert.
- RTC-Speicherung auf UTC umgestellt, damit die Umschaltung zwischen Winter- und Sommerzeit automatisch und eindeutig funktioniert.
- Hinweise zur Migration aelterer Firmware dokumentiert: Nach dem Update sollte die RTC einmal neu gesetzt werden, wenn sie zuvor in Lokalzeit gespeichert wurde.
