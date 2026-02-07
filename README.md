# SEAtech WisBlock Firmware

Firmware and hardware documentation for SEAtech’s WisBlock-based turtle tracking sensor.

## Overview
This repository contains:
- WisBlock firmware for data logging
- Hardware bill of materials (BOM)
- CSV data format used by the SEAtech web platform
- Deployment and flashing documentation

The sensor records GPS and sensor data to CSV files, which are uploaded to the SEAtech web platform for visualization and analysis.

## Data Interface
The sensor outputs CSV files compatible with the `seatech-web` uploader.

See:
- `docs/csv-format.md`

## Related Repositories
- Web platform: https://github.com/cnguye-n/seatech-web

## Status
Early development – schema and firmware under active iteration.
