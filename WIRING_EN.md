# ICS43434 → STM32F411CE (Black Pill) — Pin connections

## Microphones

| Microphone | SEL  | WS (LRCL) | BCLK    | DOUT    | VDD   | GND   |
|------------|------|-----------|---------|---------|-------|-------|
| mic1       | GND  | PA4       | PA5     | PA7     | 3V3   | GND   |
| mic2       | 3V3  | PA4       | PA5     | PA7     | 3V3   | GND   |
| mic3       | GND  | PA15      | PB3     | PB5     | 3V3   | GND   |
| mic4       | 3V3  | PA15      | PB3     | PB5     | 3V3   | GND   |

## Synchronisation jumpers (STM32 → STM32)

| From (master) | To (slave) | Signal |
|---------------|------------|--------|
| PA4           | PA15       | WS     |
| PA5           | PB3        | BCLK   |
