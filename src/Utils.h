/**
 * @file Utils.h
 * @brief Fonctions utilitaires communes
 * @author Laurent Eyen
 * @date 2024
 */

#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>

//==================== Fonctions utilitaires ====================
float lerp(float a, float b, float t);
float clampF(float value, float min_val, float max_val);
long clampL(long value, long min_val, long max_val);
float s_minjerk(float t);

#endif // UTILS_H
