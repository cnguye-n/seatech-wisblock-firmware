#pragma once

// Struct must be visible BEFORE Arduino's auto-generated prototypes

struct GnssSnapshot {
  int fixType;
  int siv;
  long lat, lon;
  bool surfaceFix;
};