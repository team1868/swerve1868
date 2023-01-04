#pragma once

#include "utils/ROBOT_CONFIG.h"

constexpr int PIGEON_ID = 35;

#if defined COMPBOT

#elif defined PRACTICEBOT

#define ANALOGENCODER
constexpr int MOD_0_ENCODER_ID = 0;
constexpr int MOD_1_ENCODER_ID = 3;
constexpr int MOD_2_ENCODER_ID = 1;
constexpr int MOD_3_ENCODER_ID = 2;

#elif defined SWERVEBASE

#define CANENCODER
/* Front Left Module - Module 0 */
constexpr int MOD_0_DRIVE_MOTOR_ID = 10;
constexpr int MOD_0_STEER_MOTOR_ID = 11;
constexpr int MOD_0_ENCODER_ID     = 30;

/* Front Right Module - Module 1 */
constexpr int MOD_1_DRIVE_MOTOR_ID = 16;
constexpr int MOD_1_STEER_MOTOR_ID = 17;
constexpr int MOD_1_ENCODER_ID     = 33;

/* Back Left Module - Module 2 */
constexpr int MOD_2_DRIVE_MOTOR_ID = 12;
constexpr int MOD_2_STEER_MOTOR_ID = 13;
constexpr int MOD_2_ENCODER_ID     = 31;

/* Back Right Module - Module 3 */
constexpr int MOD_3_DRIVE_MOTOR_ID = 14;
constexpr int MOD_3_STEER_MOTOR_ID = 15;
constexpr int MOD_3_ENCODER_ID     = 32;

#endif

/* Merge swerve drive constants into easy access vectors */
constexpr int DRIVE_MOTOR_IDS[] = {
    MOD_0_DRIVE_MOTOR_ID, MOD_1_DRIVE_MOTOR_ID, MOD_2_DRIVE_MOTOR_ID, MOD_3_DRIVE_MOTOR_ID};
constexpr int ANGLE_MOTOR_IDS[] = {
    MOD_0_STEER_MOTOR_ID, MOD_1_STEER_MOTOR_ID, MOD_2_STEER_MOTOR_ID, MOD_3_STEER_MOTOR_ID};
constexpr int ENCODER_IDS[] = {
    MOD_0_ENCODER_ID, MOD_1_ENCODER_ID, MOD_2_ENCODER_ID, MOD_3_ENCODER_ID};
