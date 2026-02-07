# CSV Data Format – v1

This document defines the CSV format produced by the SEAtech WisBlock sensor and consumed by the SEAtech web platform.

## Required Columns

| Column name        | Type    | Description |
|--------------------|---------|-------------|
| device_id          | string  | Unique device identifier |
| tag_id             | integer | Turtle tag identifier |
| timestamp_utc      | string  | ISO 8601 UTC timestamp |
| latitude           | float   | Decimal degrees |
| longitude          | float   | Decimal degrees |

## Optional Columns

| Column name | Type  | Description |
|------------|-------|-------------|
| hdop       | float | GPS horizontal dilution |
| battery_v | float | Battery voltage |
| accel_x   | float | Accelerometer X |
| accel_y   | float | Accelerometer Y |
| accel_z   | float | Accelerometer Z |

## Notes
- All timestamps MUST be UTC
- Latitude/longitude use WGS84
- Additional columns may be added in future versions

## Versioning
This schema is versioned implicitly as `v1`. Future changes must remain backward compatible where possible.
