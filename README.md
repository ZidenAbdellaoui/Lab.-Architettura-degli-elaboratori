# Lab.-Architettura-degli-elaboratori
Sistema di ausilio al parcheggio posteriore per veicoli basato su microcontrollore AVR. Fornisce feedback visivo chiaro sulla distanza dagli ostacoli, evidenzia le situazioni di pericolo ed è ottimizzato per la massima efficienza energetica.
# 🚗 Sistema di Ausilio al Parcheggio (AVR Bare-Metal)

> Sistema di ausilio al parcheggio posteriore basato su microcontrollore AVR (ATmega328P). Fornisce feedback visivo su display Nokia 5110 tramite comunicazione SPI diretta ai registri, con avviso di pericolo e ottimizzazione energetica in deep sleep.

[![Wokwi Simulation](https://img.shields.io/badge/Simulazione-Wokwi-blue)](https://wokwi.com/projects/431755786936655873)

---

## 📌 Features
- **Gestione Efficienza Energetica:** Normalmente in `SLEEP_MODE_PWR_DOWN`. Risveglio tramite interrupt esterno (`INT0`) su pulsante; rientro automatico in sleep dopo 2 minuti di inattività (> 2m o "No Echo").
- **Misurazione Non Bloccante:** Calcolo della distanza (HC-SR04) gestito da `Timer1` e Pin Change Interrupt (`PCINT0`).
- **Interfaccia Grafica Nokia 5110 (PCD8544):** Pilotata tramite interfaccia hardware SPI programmata direttamente sui registri MCU (senza librerie esterne).
- **Allarme Visivo Dinamico:** Triangolo di pericolo lampeggiante con pattern grafici variabili per distanze inferiori a 5 cm.

---

## 🛠️ Componenti Hardware
| Componente | Interfaccia / Pin MCU | Descrizione |
| :--- | :--- | :--- |
| **ATmega328P** | Bare-Metal C | Microcontrollore |
| **HC-SR04** | TRIG (PB1), ECHO (PB0/PCINT0) | Sensore ad ultrasuoni |
| **Nokia 5110** | Hardware SPI (SCK, MOSI), DC, CE, RST | Display LCD 84x48 |
| **Pulsante** | INT0 (PD2) | Risveglio da Power Down |

---

## 🏗️ Architettura del Software
Il software è strutturato in moduli indipendenti:
- `Timer`: Base tempi non bloccante (Timer0 CTC 1ms).
- `SPI & LCD Control`: Inizializzazione, gestione buffer e stampa di font (PROGMEM) e grafica tramite registro `SPDR`.
- `Distance Meter`: Trigger a 10µs e misurazione tempo di volo ultrasuono via ISR.
- `Power Management`: Transizioni tra stato Attivo e Sleep.

---

## 🔗 Riferimenti e Simulazione
- Progetto e Simulazione Live su [Wokwi](https://wokwi.com/projects/431755786936655873)
- Relazione completa del corso di Architettura degli Elaboratori (UNICAM).
