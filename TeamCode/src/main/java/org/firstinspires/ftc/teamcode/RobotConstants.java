package org.firstinspires.ftc.teamcode;

/**
 * Centralized location for robot-wide constants so they can be tuned quickly.
 */
public final class RobotConstants {
    private RobotConstants() {
        // Utility class
    }

    // === TeleOp drive scales ===
    public static final double NORMAL_TURN_SCALE = 0.70;          // 70% turn speed by default
    public static final double SLOW_MODE_TRANSLATION_SCALE = 0.50; // 50% strafe/drive speed when slow mode is held
    public static final double SLOW_MODE_TURN_SCALE = 0.35;        // 35% turn speed when slow mode is held

    // === Slow-mode positional hold ===
    public static final double SLOW_MODE_HOLD_DEADBAND = 0.05;          // Stick magnitude below this counts as "idle"
    public static final double SLOW_MODE_HOLD_TRIGGER_DEADBAND = 0.05;  // Trigger threshold for hold detection
    public static final double SLOW_MODE_HOLD_KP = 0.02;                // Proportional gain for encoder hold
    public static final double SLOW_MODE_HOLD_MAX_POWER = 0.40;         // Clamp for hold PID output

    // === Shooter hardware positions ===
    public static final double OUTTAKE_UP_POSITION = 0.45;
    public static final double OUTTAKE_DOWN_POSITION = 0.245;
    public static final double BLOCKER_UP_POSITION = 0.00;
    public static final double BLOCKER_DOWN_POSITION = 0.55;

    // === Intake presets for autos ===
    public static final double AUTO_INTAKE_POWER = 1.0;
    public static final double AUTO_FRONT_INTAKE_POWER = -0.7;
    public static final double AUTO_INTAKE_ADVANCE_SECONDS = 0.6;
    public static final double AUTO_BLOCKER_RESET_DELAY_SEC = 0.2;

    // === IMU sampling ===
    public static final double IMU_UPDATE_PERIOD_SEC = 0.040;      // Cache heading for 40 ms (~25 Hz)

    // === AprilTag scanning and alignment ===
    public static final int[] START_ZONE_TAG_IDS = {21, 22, 23};   // Tags to capture once at match start
    public static final double AUTO_ALIGN_TURN_KP = 0.90;          // Proportional gain when auto aligning to a tag
    public static final double AUTO_ALIGN_TURN_MAX_POWER = 0.40;   // Clamp for auto-align turning power
    public static final double AUTO_ALIGN_TURN_TOLERANCE_DEG = 2.0;// Acceptable yaw error before considering aligned
    public static final double AUTO_ALIGN_POLL_INTERVAL_SEC = 0.02;// Delay between alignment correction polls

    // === Shooter monitoring ===
    public static final double SHOOTER_TARGET_RPM = 80;         // Default shooter RPM target (adjust as needed)
    public static final double SHOOTER_RPM_GREEN_THRESHOLD = 7.0;  // Within 7 RPM → green indicator
    public static final double SHOOTER_RPM_YELLOW_THRESHOLD = 12.0;// Within 12 RPM → yellow indicator
    public static final double SHOOTER_TICKS_PER_REV = 28.0;       // Update if your shooter motors differ

    // === Shooter long range mode ===
    public static final double LONG_RANGE_IDLE_POWER = 0.70;       // Hold power during spin-up and staging
    public static final double LONG_RANGE_BURST_POWER = 1.00;      // Power applied while launching each ring
    public static final double REGULAR_PREFIRE_DELAY_SEC = 0.7;    // Wait before firing in standard mode
    public static final double LONG_RANGE_PREFIRE_DELAY_SEC = 1.2; // Wait before firing in long range mode
    public static final double OUTTAKE_RELEASE_DURATION_SEC = 0.2; // Time to keep the outtake servo down

    // === Drivetrain geometry ===
    public static final double DRIVE_WHEEL_DIAMETER_IN = 4.0;      // AndyMark 4" BB Mecanum wheels
    public static final double DRIVE_GEAR_REDUCTION = 1.0;         // Change if external gearing is added
    public static final double DRIVE_MOTOR_TICKS_PER_REV = 537.6;  // goBILDA 312 RPM motors
    public static final double DRIVE_TRACK_WIDTH_IN = 14.0;        // Distance between left/right wheel centers (tune)

    public static final double AUTO_DRIVE_MAX_POWER = 0.5;         // Default power for forward/strafe moves
    public static final double AUTO_TURN_MAX_POWER = 0.4;          // Default power for turns

    public static double getDriveCountsPerInch() {
        return (DRIVE_MOTOR_TICKS_PER_REV * DRIVE_GEAR_REDUCTION)
                / (Math.PI * DRIVE_WHEEL_DIAMETER_IN);
    }

    public static double getDriveCountsPerDegree() {
        double turnCircumferenceIn = Math.PI * DRIVE_TRACK_WIDTH_IN;
        double inchesPerDegree = turnCircumferenceIn / 360.0;
        return getDriveCountsPerInch() * inchesPerDegree;
    }
}
