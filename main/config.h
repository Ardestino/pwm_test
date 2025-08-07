#pragma once

#include <array>
#include <cstdint>

// Configuración WiFi
// Modifica estos valores con tus credenciales de red WiFi
#define WIFI_SSID "Totalplay-BBAB"
#define WIFI_PASS "BBAB6E45BGkpHq9G"

// Configuración avanzada WiFi
#define WIFI_MAXIMUM_RETRY 10
#define WIFI_TIMEOUT_MS 10000

// Configuración de motores
constexpr int MOTOR_COUNT = 3;
constexpr uint32_t MCPWM_RES_HZ = 1000000; // 1 MHz resolución

// Configuración de pines
constexpr std::array<int, MOTOR_COUNT> STEP_PINS = {27, 33, 19};
constexpr std::array<int, MOTOR_COUNT> DIR_PINS = {26, 32, 21};
constexpr std::array<int, MOTOR_COUNT> ENABLE_PINS = {14, 25, 18}; // Pines de habilitación para cada motor
constexpr int SPINDLE_PIN = 13; // Pin para controlar el spindle

// Límites de movimiento para cada motor (en pasos)
constexpr std::array<int32_t, MOTOR_COUNT> MIN_LIMITS = {-1000000, -1000000, -1000000};         // Límites mínimos
constexpr std::array<int32_t, MOTOR_COUNT> MAX_LIMITS = {100000, 100000, 100000}; // Límites máximos
