// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/driveTrainChooChoo.java
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Mecanum drivetrain with field-centric drive (using IMU)
 * and optional turn override (for auto-aim).
 */
public class driveTrainChooChoo {

    private final robotHardware RobotHardware;

    // Motors
    private final DcMotor frontLeftMotor;
    private final DcMotor frontRightMotor;
    private final DcMotor backRightMotor;
    private final DcMotor backLeftMotor;

    // IMU
    private final IMU imu;
    private final ElapsedTime imuUpdateTimer = new ElapsedTime();
    private double cachedHeadingRad = 0.0;
    private boolean headingInitialized = false;
    private boolean slowModeLast = false;
    private boolean holdPositionActive = false;
    private int holdFrontLeftTarget = 0;
    private int holdFrontRightTarget = 0;
    private int holdBackLeftTarget = 0;
    private int holdBackRightTarget = 0;

    // === Turn assist PID (tune these gains on-field) ===
    public static double TURN_ASSIST_KP = 1.0;
    public static double TURN_ASSIST_KI = 0.00;
    public static double TURN_ASSIST_KD = 0.0;
    public static double TURN_ASSIST_MAX_OUTPUT = 0.55;
    public static double TURN_ASSIST_I_MAX = 0.35;
    public static double TURN_ASSIST_DEADBAND_RAD = Math.toRadians(1.5);
    public static double TURN_ASSIST_MIN_DT = 0.005;
    public static double TURN_ASSIST_MAX_DT = 0.08;
    public static boolean TURN_ASSIST_INVERT = true;

    // Optional rotation assist (error input → PID output overrides driver turn)
    private Double turnAssistErrorRad = null;
    private final TurnAssistPid turnAssistPid = new TurnAssistPid();
    private final ElapsedTime turnAssistTimer = new ElapsedTime();
    private double lastTurnAssistOutput = 0.0;
    private FieldCentricDebug lastFieldCentricDebug = new FieldCentricDebug();

    public driveTrainChooChoo(robotHardware RobotHardware) {
        this.RobotHardware = RobotHardware;

        frontLeftMotor  = RobotHardware.frontLeftMotor;
        frontRightMotor = RobotHardware.frontRightMotor;
        backRightMotor  = RobotHardware.backRightMotor;
        backLeftMotor   = RobotHardware.backLeftMotor;
        imu             = RobotHardware.imu;

        turnAssistTimer.reset();
        imuUpdateTimer.reset();
    }

    /** Allow external code to assist turn (e.g., AprilTag PID). */
    public void setTurnAssist(Double assist) {
        boolean wasActive = (this.turnAssistErrorRad != null);
        if (assist == null) {
            this.turnAssistErrorRad = null;
            if (wasActive) {
                turnAssistPid.reset();
                lastTurnAssistOutput = 0.0;
            }
            return;
        }

        if (!wasActive) {
            turnAssistTimer.reset();
        }
        this.turnAssistErrorRad = assist;
    }

    /** @deprecated Use {@link #setTurnAssist(Double)}. */
    @Deprecated
    public void setTurnOverride(Double override) {
        setTurnAssist(override);
    }

    /** Call every loop from TeleOp (field-centric by default). */
    public void driveCode(Gamepad gamepad1) {
        drive(gamepad1, true);
    }

    /** Explicit field-centric helper to make intent clear from TeleOp code. */
    public void driveFieldCentric(Gamepad gamepad1) {
        drive(gamepad1, true);
    }

    /** Allow future expansion for robot-centric or other drive modes. */
    public void drive(Gamepad gamepad1, boolean fieldCentric) {
        boolean slowMode = gamepad1.right_bumper;

        double translationScale = slowMode
                ? RobotConstants.SLOW_MODE_TRANSLATION_SCALE
                : 1.0;
        double turnScale = slowMode
                ? RobotConstants.SLOW_MODE_TURN_SCALE
                : RobotConstants.NORMAL_TURN_SCALE;

        double rawYInput = gamepad1.left_stick_y;
        double rawXInput = -gamepad1.left_stick_x;
        double rawTurnInput = -gamepad1.right_stick_x; // FIXED: inverted turn

        double y  = rawYInput * translationScale;
        double x  = rawXInput * translationScale;
        double rxDriver = rawTurnInput * turnScale;
        double rx;

        if (turnAssistErrorRad != null) {
            double dt = Range.clip(turnAssistTimer.seconds(), TURN_ASSIST_MIN_DT, TURN_ASSIST_MAX_DT);
            turnAssistTimer.reset();
            lastTurnAssistOutput = turnAssistPid.update(turnAssistErrorRad, dt);
            rx = lastTurnAssistOutput;
        } else {
            turnAssistPid.reset();
            lastTurnAssistOutput = 0.0;
            rx = rxDriver;
        }

        // Slow turn with triggers
        double slowLeft  =  0.20 * gamepad1.left_trigger;
        double slowRight = -0.20 * gamepad1.right_trigger;
        double slowTurn  = slowLeft + slowRight;

        double rotX;
        double rotY;
        double headingRad = getCachedHeadingRad();
        if (fieldCentric) {
            double sinHeading = Math.sin(headingRad);
            double cosHeading = Math.cos(headingRad);

            // Field-centric transform (rotate driver input by the robot heading)
            rotX = (y * sinHeading) + (x * cosHeading);
            rotY = (y * cosHeading) - (x * sinHeading);
        } else {
            rotX = x;
            rotY = y;
        }

        rotX *= 1.1; // imperfect strafe compensation

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);

        double frontLeftPower  = (rotY + rotX + rx) / denominator;
        double backLeftPower   = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower  = (rotY + rotX - rx) / denominator;

        // Apply slow turn after main mix
        frontLeftPower  += slowTurn;
        backLeftPower   += slowTurn;
        frontRightPower -= slowTurn;
        backRightPower  -= slowTurn;

        if (slowMode != slowModeLast) {
            DcMotor.ZeroPowerBehavior behavior = slowMode
                    ? DcMotor.ZeroPowerBehavior.BRAKE
                    : DcMotor.ZeroPowerBehavior.FLOAT;
            frontLeftMotor.setZeroPowerBehavior(behavior);
            backLeftMotor.setZeroPowerBehavior(behavior);
            frontRightMotor.setZeroPowerBehavior(behavior);
            backRightMotor.setZeroPowerBehavior(behavior);
            slowModeLast = slowMode;
        }

        boolean driverIdle = slowMode
                && Math.abs(rawXInput) <= RobotConstants.SLOW_MODE_HOLD_DEADBAND
                && Math.abs(rawYInput) <= RobotConstants.SLOW_MODE_HOLD_DEADBAND
                && Math.abs(rawTurnInput) <= RobotConstants.SLOW_MODE_HOLD_DEADBAND
                && gamepad1.left_trigger <= RobotConstants.SLOW_MODE_HOLD_TRIGGER_DEADBAND
                && gamepad1.right_trigger <= RobotConstants.SLOW_MODE_HOLD_TRIGGER_DEADBAND;

        if (driverIdle) {
            engageHoldPosition();
        } else {
            holdPositionActive = false;
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }

        FieldCentricDebug debug = new FieldCentricDebug();
        debug.fieldCentricEnabled = fieldCentric;
        debug.rawX = rawXInput;
        debug.rawY = rawYInput;
        debug.rawTurn = rawTurnInput;
        debug.turnAssistActive = (turnAssistErrorRad != null);
        debug.turnAssistOutput = lastTurnAssistOutput;
        debug.appliedTurn = rx;
        debug.slowTurn = slowTurn;
        debug.headingRad = headingRad;
        debug.headingDeg = Math.toDegrees(headingRad);
        debug.translationScale = translationScale;
        debug.turnScale = turnScale;
        debug.slowModeActive = slowMode;
        debug.rotatedX = rotX;
        debug.rotatedY = rotY;
        debug.frontLeftPower = frontLeftPower;
        debug.frontRightPower = frontRightPower;
        debug.backLeftPower = backLeftPower;
        debug.backRightPower = backRightPower;
        lastFieldCentricDebug = debug;

        if (gamepad1.back) {
            RobotHardware.imu.resetYaw();
            headingInitialized = false;
            imuUpdateTimer.reset();
        }
    }

    private void engageHoldPosition() {
        if (!holdPositionActive) {
            holdPositionActive = true;
            holdFrontLeftTarget = frontLeftMotor.getCurrentPosition();
            holdFrontRightTarget = frontRightMotor.getCurrentPosition();
            holdBackLeftTarget = backLeftMotor.getCurrentPosition();
            holdBackRightTarget = backRightMotor.getCurrentPosition();
        }

        double flPower = holdPower(frontLeftMotor, holdFrontLeftTarget);
        double frPower = holdPower(frontRightMotor, holdFrontRightTarget);
        double blPower = holdPower(backLeftMotor, holdBackLeftTarget);
        double brPower = holdPower(backRightMotor, holdBackRightTarget);

        frontLeftMotor.setPower(flPower);
        frontRightMotor.setPower(frPower);
        backLeftMotor.setPower(blPower);
        backRightMotor.setPower(brPower);
    }

    private double holdPower(DcMotor motor, int targetPosition) {
        int error = targetPosition - motor.getCurrentPosition();
        double power = error * RobotConstants.SLOW_MODE_HOLD_KP;
        return Range.clip(power, -RobotConstants.SLOW_MODE_HOLD_MAX_POWER, RobotConstants.SLOW_MODE_HOLD_MAX_POWER);
    }

    private double getCachedHeadingRad() {
        if (!headingInitialized
                || imuUpdateTimer.seconds() >= RobotConstants.IMU_UPDATE_PERIOD_SEC) {
            cachedHeadingRad = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            headingInitialized = true;
            imuUpdateTimer.reset();
        }
        return cachedHeadingRad;
    }

    public double getHeadingDegrees() {
        return Math.toDegrees(getCachedHeadingRad());
    }

    /** Last commanded PID output for telemetry. */
    public double getTurnAssistOutput() {
        return lastTurnAssistOutput;
    }

    /** Latest PID telemetry for debugging auto-aim turn assist. */
    public TurnAssistPidTelemetry getTurnAssistTelemetry() {
        return turnAssistPid.getLastTelemetry();
    }

    /** Whether the PID-based turn assist is active. */
    public boolean isTurnAssistActive() {
        return turnAssistErrorRad != null;
    }

    /** Latest field-centric debug snapshot for telemetry. */
    public FieldCentricDebug getFieldCentricDebug() {
        return lastFieldCentricDebug;
    }

    /** Encapsulates the most recent PID terms for telemetry. */
    public static class TurnAssistPidTelemetry {
        public final double inputYawErrorRad;
        public final double invertedErrorRad;
        public final double pidErrorRad;
        public final double pTerm;
        public final double iTerm;
        public final double dTerm;
        public final double integralState;
        public final double derivative;
        public final double dtSeconds;
        public final double output;

        TurnAssistPidTelemetry(double inputYawErrorRad,
                               double invertedErrorRad,
                               double pidErrorRad,
                               double pTerm,
                               double iTerm,
                               double dTerm,
                               double integralState,
                               double derivative,
                               double dtSeconds,
                               double output) {
            this.inputYawErrorRad = inputYawErrorRad;
            this.invertedErrorRad = invertedErrorRad;
            this.pidErrorRad = pidErrorRad;
            this.pTerm = pTerm;
            this.iTerm = iTerm;
            this.dTerm = dTerm;
            this.integralState = integralState;
            this.derivative = derivative;
            this.dtSeconds = dtSeconds;
            this.output = output;
        }
    }

    /** Simple PID used to convert yaw error into a turn command. */
    private static class TurnAssistPid {
        private double integral = 0.0;
        private double previousError = 0.0;
        private boolean first = true;

        private double lastInputYawErrorRad = 0.0;
        private double lastInvertedErrorRad = 0.0;
        private double lastPidErrorRad = 0.0;
        private double lastPTerm = 0.0;
        private double lastITerm = 0.0;
        private double lastDTerm = 0.0;
        private double lastIntegralState = 0.0;
        private double lastDerivative = 0.0;
        private double lastDtSeconds = 0.0;
        private double lastOutput = 0.0;

        double update(double yawErrorRad, double dt) {
            double processedError = TURN_ASSIST_INVERT ? -yawErrorRad : yawErrorRad;
            double pidError = Math.abs(processedError) < TURN_ASSIST_DEADBAND_RAD ? 0.0 : processedError;

            integral += pidError * dt;
            integral = Range.clip(integral, -TURN_ASSIST_I_MAX, TURN_ASSIST_I_MAX);

            double derivative = 0.0;
            double safeDt = Math.max(dt, 1e-3);
            if (!first) {
                derivative = (pidError - previousError) / safeDt;
            }

            double pTerm = TURN_ASSIST_KP * pidError;
            double iTerm = TURN_ASSIST_KI * integral;
            double dTerm = TURN_ASSIST_KD * derivative;

            double output = pTerm + iTerm + dTerm;
            output = Range.clip(output, -TURN_ASSIST_MAX_OUTPUT, TURN_ASSIST_MAX_OUTPUT);

            previousError = pidError;
            first = false;

            lastInputYawErrorRad = yawErrorRad;
            lastInvertedErrorRad = processedError;
            lastPidErrorRad = pidError;
            lastPTerm = pTerm;
            lastITerm = iTerm;
            lastDTerm = dTerm;
            lastIntegralState = integral;
            lastDerivative = derivative;
            lastDtSeconds = safeDt;
            lastOutput = output;

            return output;
        }

        TurnAssistPidTelemetry getLastTelemetry() {
            return new TurnAssistPidTelemetry(
                    lastInputYawErrorRad,
                    lastInvertedErrorRad,
                    lastPidErrorRad,
                    lastPTerm,
                    lastITerm,
                    lastDTerm,
                    lastIntegralState,
                    lastDerivative,
                    lastDtSeconds,
                    lastOutput
            );
        }

        void reset() {
            integral = 0.0;
            previousError = 0.0;
            first = true;
            clearTelemetry();
        }

        private void clearTelemetry() {
            lastInputYawErrorRad = 0.0;
            lastInvertedErrorRad = 0.0;
            lastPidErrorRad = 0.0;
            lastPTerm = 0.0;
            lastITerm = 0.0;
            lastDTerm = 0.0;
            lastIntegralState = 0.0;
            lastDerivative = 0.0;
            lastDtSeconds = 0.0;
            lastOutput = 0.0;
        }
    }

    /** Captures the driver inputs and heading used for the last field-centric calculation. */
    public static class FieldCentricDebug {
        public boolean fieldCentricEnabled;
        public double rawX;
        public double rawY;
        public double rawTurn;
        public boolean turnAssistActive;
        public double turnAssistOutput;
        public double appliedTurn;
        public double slowTurn;
        public boolean slowModeActive;
        public double translationScale;
        public double turnScale;
        public double headingRad;
        public double headingDeg;
        public double rotatedX;
        public double rotatedY;
        public double frontLeftPower;
        public double frontRightPower;
        public double backLeftPower;
        public double backRightPower;
    }
}
