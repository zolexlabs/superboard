# Zolex Superboard — open hardware

The Zolex Superboard is an RP2040 development board with the bring-up bench built
in: a CMSIS-DAP debug probe, a 16-channel logic analyser, and a 25-LED pin
dashboard, all presented to your PC over one USB-C cable by an on-board AT32F405
co-processor.

- Product / docs: **https://zolex.co/sb**
- Crowd Supply: https://www.crowdsupply.com/zolex/superboard

This repository holds the **open-source hardware design** for board revision **v1.2**
(98 × 86 mm, AT32F405 + RP2040).

## Contents

```
hardware/
  Superboard_v1.2.sch.json   editable schematic  (EasyEDA source)
  Superboard_v1.2.pcb.json   editable PCB layout (EasyEDA source)
  Superboard_v1.2_schematic.pdf
  Superboard_v1.2_BOM.csv
```

To edit the design, import the `.json` files into [EasyEDA](https://easyeda.com/).
For a quick read, open the schematic PDF.

## Licence

Hardware design files are licensed under the **CERN Open Hardware Licence Version 2 —
Strongly Reciprocal (CERN-OHL-S-2.0)**; see [`LICENSE`](LICENSE). In short: you may
use, study, modify and build the design, but if you distribute a product based on it
you must make your complete source available under the same terms.

"Zolex" and "Superboard" are trademarks of Zolex Labs Ltd and are **not** covered by
that licence — see [`TRADEMARKS.md`](TRADEMARKS.md).

## Firmware

The board firmware (co-processor + RP2040 demos) is released separately under its own
software licence — not in this repository.
