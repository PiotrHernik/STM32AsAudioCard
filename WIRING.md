# Podłączenie 4-mikrofonowej macierzy ICS43434 do STM32F411CE (Black Pill)

## Tabela podłączeń

| Mikrofon | SEL  | WS (LRCL) | BCLK    | DOUT    | Peryferium     |
|----------|------|-----------|---------|---------|----------------|
| mic1     | GND  | **PA4**   | **PA5** | **PA7** | I2S1 (master)  |
| mic2     | 3V3  | PA4       | PA5     | PA7     | I2S1 (master)  |
| mic3     | GND  | **PA15**  | **PB3** | **PB5** | I2S3 (slave)   |
| mic4     | 3V3  | PA15      | PB3     | PB5     | I2S3 (slave)   |

Każdy mikrofon dodatkowo: **VDD → 3V3**, **GND → GND**.

Pin **SEL** decyduje, w której połowie ramki WS mikrofon nadaje:
- `SEL → GND`  → mikrofon nadaje w lewej połowie (sloty parzyste w buforze DMA)
- `SEL → 3V3`  → mikrofon nadaje w prawej połowie (sloty nieparzyste)

Dzięki temu **dwa mikrofony dzielą jedną linię DOUT** — gadają na zmianę.

## Jumpery synchronizacji (krytyczne!)

Dwa krótkie kable na płytce, które rozprowadzają zegar mastera do slave'a:

```
PA4  ━━━━━━━━━━━━ PA15    (WS: master out  →  slave in)
PA5  ━━━━━━━━━━━━ PB3     (BCLK: master out → slave in)
```

To są **te same fizyczne sygnały** co LRCL/BCLK mikrofonów — możesz po prostu
poprowadzić jedną wspólną szynę WS i jedną wspólną szynę BCLK, do której
podpinasz wszystkie 4 mikrofony **i** pin slave'a (PA15 dla WS, PB3 dla BCLK).

## Schemat poglądowy

```
                  STM32F411CE Black Pill
                  ┌──────────────────────┐
        ┌─ PA4 ───┤ I2S1_WS  (master)    │
        │  PA5 ───┤ I2S1_CK  (master)    │
        │  PA7 ───┤ I2S1_SD              │
        │         │                      │
        │  PA15 ──┤ I2S3_WS  (slave)     │── jumper wire ←┐
        │  PB3 ───┤ I2S3_CK  (slave)     │── jumper wire ←┤
        │  PB5 ───┤ I2S3_SD              │                │
        │  3V3 ───┤                      │                │
        │  GND ───┤                      │                │
        │         └──────────────────────┘                │
        │                                                 │
        │ (PA4 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ PA15) │ ← WS bridge
        │ (PA5 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ PB3 ) │ ← CK bridge
        │
        ├─ mic1 (SEL→GND)  ── LRCL→PA4, BCLK→PA5, DOUT→PA7
        ├─ mic2 (SEL→3V3)  ── LRCL→PA4, BCLK→PA5, DOUT→PA7   (ten sam SD!)
        ├─ mic3 (SEL→GND)  ── LRCL→PA15, BCLK→PB3, DOUT→PB5
        └─ mic4 (SEL→3V3)  ── LRCL→PA15, BCLK→PB3, DOUT→PB5  (ten sam SD!)

   wszystkie mikrofony: VDD→3V3, GND→GND
```

## Dlaczego tak (zasada synchronizacji)

I2S1 jest **masterem** — generuje BCLK na PA5 i WS na PA4.
I2S3 jest **slave'em** — nie generuje własnych zegarów, tylko podsłuchuje
te same BCLK/WS na PB3 i PA15 (dzięki jumperom).

Skutek: wszystkie 4 mikrofony łapią próbki na **dokładnie tych samych
zboczach BCLK**. Brak dryfu fazy, brak nieznanego offsetu między kanałami,
gotowe pod beamforming / DOA / cancellation.

Sekwencja startu w firmware (`main.c`):
1. `HAL_I2S_Receive_DMA(&hi2s3, …)` — slave armowany, czeka na WS
2. `HAL_I2S_Receive_DMA(&hi2s1, …)` — master startuje BCLK/WS
3. Slave łapie pierwszą krawędź WS i od próbki 0 oba DMA maszerują w lockstepie

## Kolejność kanałów na USB

Host (Audacity / ALSA / Reaper) widzi 4 niezależne kanały, interleavowane
w każdym 1 ms isochronous packet:

```
[mic1, mic2, mic3, mic4, mic1, mic2, mic3, mic4, ...]
   ↑      ↑      ↑      ↑
 PA7    PA7    PB5    PB5
 SEL=0  SEL=1  SEL=0  SEL=1
```

Format strumienia: 16 kHz × 4 kanały × 24-bit (S24_3LE), 192 B/ms.

## Mapowanie DMA / IRQ

| Strumień   | DMA              | IRQ                   |
|------------|------------------|-----------------------|
| I2S1 RX    | DMA2 Stream 0 Ch 3 | `DMA2_Stream0_IRQn` |
| I2S3 RX    | DMA1 Stream 0 Ch 0 | `DMA1_Stream0_IRQn` |

Master i slave na osobnych kontrolerach DMA — nie konkurują o magistralę.

## O kablach

WS to ~16 kHz, BCLK to ~1.024 MHz — bardzo wolne sygnały. Wejścia ICS43434
są CMOS o impedancji rzędu MΩ i pojemności ~5 pF, więc nawet rozdzielanie
jednego pinu STM-a na 3–4 wejścia w „splatce w powietrzu" nie psuje sygnału.

Co ma znaczenie:
- **wspólny GND** wszystkich mikrofonów z masą STM32 (obowiązkowo),
- długość wiązki < ~20 cm dla zdrowego marginesu EMI,
- lokalny kondensator 0.1 µF od VDD do GND tuż przy każdym mikrofonie
  (filtracja zasilania, nie ma związku z topologią sygnału).
