package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public final    class Constants {

    public static final class VisionConstants {
        public static final String LIMELIGHT_NAME = "limelight-adacam";

        // NEW: Physical offset of the Limelight from the center of the turret's rotation.
        // X = Forward (direction the turret shoots), Y = Left, Z = Up
        // ⚠️ REPLACE THESE WITH YOUR ACTUAL PHYSICAL MEASUREMENTS
        public static final Transform3d TURRET_TO_CAMERA = new Transform3d(
            new Translation3d(-0.11, 0.0, 0.32), // Example: Camera is 15cm left, 10cm up from turret center
            new Rotation3d(0, Math.toRadians(25), 0) // Example: Camera is pitched up 25 degrees
        );
    }

    public static final class IntakeConstants {

        // ── CAN ───────────────────────────────────────────────────────────────
        public static final int UPPER_INDEX_MOTOR_ID  = 31;
        public static final int BOTTOM_ROLLER_MOTOR_ID = 32;
        public static final int PIVOT_MOTOR_ID         = 33;
        public static final int OUTER_ROLLER_MOTOR_ID  = 34;

        // ── Motor Inverts ─────────────────────────────────────────────────────
        public static final boolean UPPER_INDEX_INVERTED   = false;
        public static final boolean BOTTOM_ROLLER_INVERTED = false;
        public static final boolean PIVOT_INVERTED         = false;
        public static final boolean OUTER_ROLLER_INVERTED  = false;

        // ── Current Limits (A) ────────────────────────────────────────────────
        public static final int ROLLER_SMART_CURRENT_LIMIT_A = 70;
        public static final int PIVOT_SMART_CURRENT_LIMIT_A  = 30;

        // ── Roller Output Setpoints ───────────────────────────────────────────
        public static final double UPPER_INDEX_INTAKE_OUTPUT  = -0.5;
        public static final double BOTTOM_ROLLER_INTAKE_OUTPUT = 0.4;
        public static final double OUTER_ROLLER_INTAKE_OUTPUT  = -0.4;

        public static final double UPPER_INDEX_OUTTAKE_OUTPUT  = 0.5;
        public static final double BOTTOM_ROLLER_OUTTAKE_OUTPUT = -0.4;
        public static final double OUTER_ROLLER_OUTTAKE_OUTPUT  = 0.4;

        // ── Pivot Gearing ─────────────────────────────────────────────────────
        // 15:1 AM Sport planetary + 24:15 timing belt = 24:1 overall
        public static final double PIVOT_GEAR_RATIO = 15.0 * (24.0 / 15.0); // = 24.0

        // ── Pivot Angle Setpoints (mechanism degrees) ─────────────────────────
        // Boot-zero convention: robot starts physically in Closed position.
        public static final double PIVOT_CLOSED_ANGLE_DEG = 0.0;
        public static final double PIVOT_OPEN_ANGLE_DEG   = 350.0; // ⚠️ verify on hardware

        // ── Pivot Soft/Hard Limits (mechanism degrees) ────────────────────────
        public static final double PIVOT_SOFT_MIN_ANGLE_DEG = -1380.0;
        public static final double PIVOT_SOFT_MAX_ANGLE_DEG = 380.0;
        public static final double PIVOT_HARD_MIN_ANGLE_DEG = -1380.0;
        public static final double PIVOT_HARD_MAX_ANGLE_DEG = 400.0;
        public static final double PIVOT_AT_TARGET_TOLERANCE_DEG = 2.0;

        // ── PID Gains ─────────────────────────────────────────────────────────
        public static final double PIVOT_kP = 0.03; // ⚠️ tune on hardware/sim
        public static final double PIVOT_kI = 0.0;
        public static final double PIVOT_kD = 0.0;

        // ── ArmFeedforward ────────────────────────────────────────────────────
        public static final double PIVOT_kS = 0.0; // ⚠️ tune on hardware
        public static final double PIVOT_kG = 0.0; // ⚠️ tune on hardware
        public static final double PIVOT_kV = 0.0; // ⚠️ tune on hardware

        // ── Pivot Motion Constraints ──────────────────────────────────────────
        public static final double PIVOT_MAX_VEL_DEG_PER_SEC    = 375.0;
        public static final double PIVOT_MAX_ACCEL_DEG_PER_SEC2 = 500.0;

        // ── Arm Physical Parameters (simulation) ─────────────────────────────
        public static final double PIVOT_ARM_LENGTH_METERS = 0.35; // ⚠️ measure actual
        public static final double PIVOT_ARM_MASS_KG       = 1.2;  // ⚠️ weigh actual
    }

    public static final class TurretConstants {

        // ── CAN ───────────────────────────────────────────────────────────────
        public static final int    TURN_MOTOR_ID  = 21;
        public static final int    ENCODER_10T_ID = 22;
        public static final int    ENCODER_17T_ID = 23;
        public static final String CANBUS         = "rio";

        // ── Gearing ───────────────────────────────────────────────────────────
        public static final int    TURRET_RING_GEAR_TEETH   = 85;
        public static final int    ENCODER_10T_PINION_TEETH = 10;
        public static final int    ENCODER_17T_PINION_TEETH = 17;

        public static final double MOTOR_TO_MECHANISM_RATIO = (48.0 / 12.0) * (85.0 / 10.0); // = 34.0

        // ── Motor ─────────────────────────────────────────────────────────────
        public static final boolean MOTOR_INVERTED       = false;
        public static final double  SUPPLY_CURRENT_LIMIT = 40.0;
        public static final double  STATOR_CURRENT_LIMIT = 80.0;
        public static final double  MAX_OUTPUT           = 0.05; // ⚠️ increase per speed guide

        // ── PID / Feedforward (Slot 0) ────────────────────────────────────────
        public static final double kP = 5.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;  // ⚠️ add only if oscillation occurs
        public static final double kS = 0.25;
        public static final double kV = 0.12;
        public static final double kA = 0.0;  // ⚠️ leave at 0 until speed stage 3+

        // ── MotionMagic ───────────────────────────────────────────────────────
        public static final double TURRET_CRUISE_DEG_PER_SEC = 2000.0; // ⚠️ increase per speed guide
        public static final double TURRET_ACCEL_DEG_PER_SEC2 = 4000.0; // always 2× cruise
        public static final double TURRET_JERK_DEG_PER_SEC3  = 20000.0; // always 10× cruise

        public static final double MM_CRUISE_VELOCITY =
            (TURRET_CRUISE_DEG_PER_SEC / 360.0) * MOTOR_TO_MECHANISM_RATIO;
        public static final double MM_ACCELERATION =
            (TURRET_ACCEL_DEG_PER_SEC2 / 360.0) * MOTOR_TO_MECHANISM_RATIO;
        public static final double MM_JERK =
            (TURRET_JERK_DEG_PER_SEC3  / 360.0) * MOTOR_TO_MECHANISM_RATIO;

        // ── Mechanism Range ───────────────────────────────────────────────────
        public static final double MIN_ANGLE_ROT = -0.3;
        public static final double MAX_ANGLE_ROT =  0.3;

        // ── CRT ───────────────────────────────────────────────────────────────
        public static final double ENCODER_10T_OFFSET_ROT  = 0.081299;
        public static final double ENCODER_17T_OFFSET_ROT  = -0.269287;
        public static final double CRT_MATCH_TOLERANCE_ROT = 0.06;

        // ── Control ───────────────────────────────────────────────────────────
        public static final double AT_TARGET_TOLERANCE_DEG = 1.0;
    }

    public static final class FeederConstants {
        public static final int FEEDER_MOTOR_ID = 36; // Change this to your actual CAN ID
        public static final double FEEDER_SPEED = 0.5; // Speed for moving the ball up
        public static final boolean MOTOR_INVERTED = false;
        public static final int CURRENT_LIMIT = 60; // Amps
    }

    public static final class ShooterConstants {

        // ── CAN ───────────────────────────────────────────────────────────────
        public static final int    LEFT_FLYWHEEL_MOTOR_ID  = 41;
        public static final int    RIGHT_FLYWHEEL_MOTOR_ID = 42;
        public static final String CANBUS                  = "rio";

        // ── Flywheel Gearing ──────────────────────────────────────────────────
        public static final double FLYWHEEL_GEAR_RATIO = 1.0; // 1:1 direct drive

        // ── Flywheel Speeds ───────────────────────────────────────────────────
        public static final double STANDARD_SHOOT_RPM = 2000.0;
        public static final double WARM_UP_RPM        = 1000.0; // 75% of standard

        // ── Flywheel Motor Inverts ────────────────────────────────────────────
        public static final boolean LEFT_FLYWHEEL_INVERTED  = true;
        public static final boolean RIGHT_FLYWHEEL_INVERTED = false;

        // ── Current Limits (A) ────────────────────────────────────────────────
        public static final double FLYWHEEL_SUPPLY_CURRENT_LIMIT = 80.0;
        public static final double FLYWHEEL_STATOR_CURRENT_LIMIT = 80.0;

        // ── Flywheel PID / Feedforward ────────────────────────────────────────
        public static final double FLYWHEEL_kP = 0.1;  // ⚠️ tune on hardware
        public static final double FLYWHEEL_kI = 0.0;
        public static final double FLYWHEEL_kD = 0.0;
        public static final double FLYWHEEL_kS = 0.1;  // ⚠️ tune on hardware
        public static final double FLYWHEEL_kV = 0.12; // ⚠️ tune on hardware
        public static final double FLYWHEEL_kA = 0.0;

        // ── Control Tolerances ────────────────────────────────────────────────
        public static final double FLYWHEEL_AT_SPEED_TOLERANCE_RPM = 100.0;
    }

    public static final class HoodConstants {

        // ── CAN ───────────────────────────────────────────────────────────────
        public static final int    HOOD_MOTOR_ID = 43;
        public static final String CANBUS        = "rio";

        // ── Motor Invert ──────────────────────────────────────────────────────
        public static final InvertedValue HOOD_INVERTED = InvertedValue.CounterClockwise_Positive;

        // ── Gearing ───────────────────────────────────────────────────────────
        // (20/48) * (15/32) * (10/187)
        public static final double HOOD_GEAR_RATIO = (20.0 / 48.0) * (15.0 / 32.0) * (10.0 / 187.0);
        // Motor rotations per mechanism rotation (for CTRE SensorToMechanismRatio)
        public static final double HOOD_MOTOR_TO_MECHANISM_RATIO = 1.0 / HOOD_GEAR_RATIO; // ≈ 89.76

        // ── Angle Limits (mechanism degrees) ─────────────────────────────────
        public static final double HOOD_STARTING_ANGLE_DEG      = 18.2;
        public static final double HOOD_MAX_TRAVEL_DEG          = 100.0;
        public static final double HOOD_MIN_ANGLE_DEG           = HOOD_STARTING_ANGLE_DEG;
        public static final double HOOD_MAX_ANGLE_DEG           = HOOD_STARTING_ANGLE_DEG + HOOD_MAX_TRAVEL_DEG; // 56.2°
        public static final double HOOD_AT_TARGET_TOLERANCE_DEG = 1.0;

        // ── Current Limits (A) ────────────────────────────────────────────────
        public static final double HOOD_SUPPLY_CURRENT_LIMIT = 40.0;
        public static final double HOOD_STATOR_CURRENT_LIMIT = 60.0;

        // ── PID / Feedforward (Slot 0) ────────────────────────────────────────
        public static final double HOOD_kP = 80.0; // mechanism rotations error → much higher needed
        public static final double HOOD_kI = 0.0;
        public static final double HOOD_kD = 2.0;
        public static final double HOOD_kS = 0.25; // start here, tune up until it just moves
        public static final double HOOD_kG = 0.0;
        public static final double HOOD_kV = 0.0;

        // ── MotionMagic Constraints ───────────────────────────────────────────
        // Stored in mechanism deg/s, converted to motor RPS for CTRE
        public static final double HOOD_MAX_VEL_DEG_PER_SEC    = 180.0; // ⚠️ tune on hardware
        public static final double HOOD_MAX_ACCEL_DEG_PER_SEC2 = 360.0; // ⚠️ tune on hardware

        // Converted to motor RPS for MotionMagic config
        public static final double HOOD_MAX_VEL_RPS =
            (HOOD_MAX_VEL_DEG_PER_SEC / 360.0) * HOOD_MOTOR_TO_MECHANISM_RATIO;
        public static final double HOOD_MAX_ACCEL_RPS2 =
            (HOOD_MAX_ACCEL_DEG_PER_SEC2 / 360.0) * HOOD_MOTOR_TO_MECHANISM_RATIO;

        // ── Physical Parameters (simulation) ─────────────────────────────────
        public static final double HOOD_ARM_LENGTH_METERS = 0.25; // ⚠️ measure actual
        public static final double HOOD_ARM_MASS_KG       = 0.8;  // ⚠️ weigh actual
    }
}