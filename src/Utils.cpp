/**
 * @file Utils.cpp
 * @brief Implémentation des fonctions utilitaires communes
 * @author Laurent Eyen
 * @date 2024
 */

#include "Utils.h"

//==================== Fonctions utilitaires ====================
float lerp(float a, float b, float t) {
  return a + t * (b - a);
}

float clampF(float value, float min_val, float max_val) {
  if (value < min_val) return min_val;
  if (value > max_val) return max_val;
  return value;
}

long clampL(long value, long min_val, long max_val) {
  if (value < min_val) return min_val;
  if (value > max_val) return max_val;
  return value;
}

float s_minjerk(float t) {
  // Profil minimum-jerk: s(t) = 6t^5 - 15t^4 + 10t^3
  float t2 = t * t;
  float t3 = t2 * t;
  float t4 = t3 * t;
  float t5 = t4 * t;
  return 6.0f * t5 - 15.0f * t4 + 10.0f * t3;
}
