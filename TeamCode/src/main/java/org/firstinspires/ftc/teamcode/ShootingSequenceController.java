// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ShootingSequenceController.java
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Manages the shooting sequence state machine.
 * Handles A/B/Y/X button clicks and the multi-step shooting logic.
 */
public class ShootingSequenceController {

    // Hardware references
    private final DcMotor leftOuttakeMotor;
    private final DcMotor rightOuttakeMotor;
    private final DcMotor intakeMotor;
    private final DcMotor frontIntakeMotor;
    private final Servo outtakeServo;
    private final Servo blockerServo;

    // Servo positions (FIXED: swapped UP and DOWN for outtake)
    private static final double OUTTAKE_UP_POS   = 0.45;
    private static final double OUTTAKE_DOWN_POS = 0.245;
    private static final double BLOCKER_UP_POS   = 0.00;
    private static final double BLOCKER_DOWN_POS = 0.55;

    // Intake speeds
    private static final double INTAKE_FULL_SPEED = 1.0;
    private static final double FRONT_INTAKE_SPEED = -0.7;
    private static final double INTAKE_HOLD_SPEED = 0.3;
    private static final double INTAKE_ROTATION_SPEED = 0.5;

    // State tracking
    private int aClickCount = 0;
    private int yClickCount = 0;
    private boolean prevA = false;
    private boolean prevY = false;
    private boolean prevX = false;
    private boolean prevB = false;
    private boolean intakeOn = false;

    // Shooting sequence state machine
    private enum ShootingState {
        IDLE,
        SPINUP,
        SHOOTING
    }
    private ShootingState shootingState = ShootingState.IDLE;
    private int shotsFired = 0;
    private int shotsToFire = 0;
    private ElapsedTime stepTimer = new ElapsedTime();
    private int currentShootingStep = 0;

    // Intake rotation targets (tune here for quick adjustments)
    private static final double INTAKE_REVERSE_ROTATIONS = 0.1; // retract a quarter turn for consistent staging
    private static final double INTAKE_FORWARD_ROTATIONS = 0.8;  // advance one turn to feed the next ring
    private static final double ENCODER_COUNTS_PER_ROTATION = 537.6;
    private static final int INTAKE_REVERSE_TICKS = (int) Math.max(1,
            Math.round(ENCODER_COUNTS_PER_ROTATION * INTAKE_REVERSE_ROTATIONS));
    private static final int INTAKE_FORWARD_TICKS = (int) Math.max(1,
            Math.round(ENCODER_COUNTS_PER_ROTATION * INTAKE_FORWARD_ROTATIONS));
    private int intakeStartPosition = 0;
    private boolean waitingForIntakeRotation = false;
    private int intakeMoveTicks = 0;

    // Shooter RPM target and limits
    private static final double SHOOTER_TARGET_RPM = 120.0;
    private static final double SHOOTER_POWER_MAX = 1.0;
    private static final double SHOOTER_RAMP_DURATION_S = 0.75;
    private static final double SHOOTER_RAMP_MAX_POWER = 0.5;
    private static final double SHOOTER_COUNTS_PER_REV = 537.6;
    private static final double SHOOTER_RPM_FILTER_ALPHA = 0.25;
    private static final double SHOOTER_PID_INTEGRAL_LIMIT = 1.0;
    // Left shooter PID gains (tune independently from the right side)
    private static final double LEFT_SHOOTER_PID_KP = 0.015;
    private static final double LEFT_SHOOTER_PID_KI = 0.0;
    private static final double LEFT_SHOOTER_PID_KD = 0.0008;
    // Right shooter PID gains
    private static final double RIGHT_SHOOTER_PID_KP = 0.015;
    private static final double RIGHT_SHOOTER_PID_KI = 0.0;
    private static final double RIGHT_SHOOTER_PID_KD = 0.0008;

    // Shooter velocity control
    private final ShooterPidController leftShooterPid = new ShooterPidController(
            LEFT_SHOOTER_PID_KP, LEFT_SHOOTER_PID_KI, LEFT_SHOOTER_PID_KD, SHOOTER_PID_INTEGRAL_LIMIT);
    private final ShooterPidController rightShooterPid = new ShooterPidController(
            RIGHT_SHOOTER_PID_KP, RIGHT_SHOOTER_PID_KI, RIGHT_SHOOTER_PID_KD, SHOOTER_PID_INTEGRAL_LIMIT);
    private final ElapsedTime shooterTimer = new ElapsedTime();
    private final ElapsedTime shooterRampTimer = new ElapsedTime();
    private boolean shooterPidEnabled = false;
    private double targetShooterRpmBase = 0.0;
    private double targetLeftShooterRpm = 0.0;
    private double targetRightShooterRpm = 0.0;
    private double filteredLeftShooterRpm = 0.0;
    private double filteredRightShooterRpm = 0.0;
    private int lastLeftShooterPosition = 0;
    private int lastRightShooterPosition = 0;
    private double lastLeftShooterPowerCommand = 0.0;
    private double lastRightShooterPowerCommand = 0.0;

    public ShootingSequenceController(robotHardware hardware) {
        this.leftOuttakeMotor = hardware.leftOuttakeMotor;
        this.rightOuttakeMotor = hardware.rightOuttakeMotor;
        this.intakeMotor = hardware.intakeMotor;
        this.frontIntakeMotor = hardware.frontIntakeMotor;
        this.outtakeServo = hardware.outtakeServo;
        this.blockerServo = hardware.ballBlocker;

        this.leftOuttakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightOuttakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.leftOuttakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightOuttakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.leftOuttakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.rightOuttakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        lastLeftShooterPosition = this.leftOuttakeMotor.getCurrentPosition();
        lastRightShooterPosition = this.rightOuttakeMotor.getCurrentPosition();
        shooterTimer.reset();

        this.intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Call every TeleOp loop with gamepad input and AprilTag info.
     */
    public void update(boolean aPressed, boolean bPressed, boolean yPressed, boolean xPressed,
                       AprilTagCalibration.Report tagReport) {

        // X button: cancel everything
        if (xPressed && !prevX) {
            cancelSequence();
            blockerServo.setPosition(BLOCKER_UP_POS);
            outtakeServo.setPosition(OUTTAKE_UP_POS);
        }
        prevX = xPressed;

        // B button: toggle intake
        if (bPressed && !prevB) {
            intakeOn = !intakeOn;
            intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            waitingForIntakeRotation = false;
            if (intakeOn) {
                intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                frontIntakeMotor.setPower(FRONT_INTAKE_SPEED);
                intakeMotor.setPower(INTAKE_FULL_SPEED);
            } else {
                frontIntakeMotor.setPower(0.0);
                intakeMotor.setPower(0.0);
            }
        }
        prevB = bPressed;

        // Y button: increment shot count
        if (yPressed && !prevY) {
            yClickCount++;
        }
        prevY = yPressed;

        // A button state machine
        if (aPressed && !prevA) {
            aClickCount++;
            handleAClick();
        }
        prevA = aPressed;

        // Run shooting sequence if active
        if (shootingState != ShootingState.IDLE) {
            updateShootingSequence();
        }

        updateShooterVelocityControl();
    }

    private void handleAClick() {
        switch (aClickCount) {
            case 1:
                // First click: prepare for shooting
                yClickCount = 0; // Reset shot counter
                blockerServo.setPosition(BLOCKER_DOWN_POS);

                // Only start intake if B button already turned it on
                if (!intakeOn) {
                    intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    intakeMotor.setPower(0.0);
                    frontIntakeMotor.setPower(0.0);
                } else {
                    // B was already on, so reduce regular intake to 25%
                    intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    intakeMotor.setPower(INTAKE_HOLD_SPEED);
                    frontIntakeMotor.setPower(FRONT_INTAKE_SPEED);
                }
                break;

            case 2:
                // Second click: spin up motors
                shootingState = ShootingState.SPINUP;
                spinUpMotors();
                break;

            case 3:
                // Third click: begin shooting sequence
                if (shootingState == ShootingState.SPINUP) {
                    intakeMotor.setPower(0.0);
                    frontIntakeMotor.setPower(0.0);
                    shootingState = ShootingState.SHOOTING;
                    shotsToFire = Math.max(1, yClickCount);
                    shotsFired = 0;
                    currentShootingStep = 0;
                    stepTimer.reset();
                }
                break;
        }
    }

    private void spinUpMotors() {
        enableShooterPid();
    }

    private void updateShootingSequence() {
        if (shootingState == ShootingState.SPINUP) {
            // Continuously adjust RPM during spinup phase
            spinUpMotors();
            return;
        }

        if (shootingState != ShootingState.SHOOTING) {
            return;
        }

        double elapsed = stepTimer.seconds();

        // Each step has a duration; adjust timing as needed
        switch (currentShootingStep) {
            case 0:
                // Step 1: Retract intake by the configured reverse rotation at 50% power
                if (!waitingForIntakeRotation) {
                    intakeStartPosition = intakeMotor.getCurrentPosition();
                    intakeMoveTicks = INTAKE_REVERSE_TICKS;
                    int targetPosition = intakeStartPosition - intakeMoveTicks;
                    intakeMotor.setTargetPosition(targetPosition);
                    intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    intakeMotor.setPower(INTAKE_ROTATION_SPEED);
                    waitingForIntakeRotation = true;
                } else if (!intakeMotor.isBusy()
                        || Math.abs(intakeMotor.getCurrentPosition() - intakeStartPosition) >= intakeMoveTicks) {
                    intakeMotor.setPower(0.0);
                    intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    waitingForIntakeRotation = false;
                    intakeMoveTicks = 0;
                    currentShootingStep++;
                    stepTimer.reset();
                }
                break;

            case 1:
                // Step 2: Adjust motor RPM if distance changed
                if (elapsed < 0.1) {
                    spinUpMotors();
                } else {
                    currentShootingStep++;
                    stepTimer.reset();
                }
                break;

            case 2:
                // Step 3: Raise outtake servo (shoot first ball)
                if (elapsed < 0.2) {
                    outtakeServo.setPosition(OUTTAKE_DOWN_POS);
                } else {
                    currentShootingStep++;
                    stepTimer.reset();
                }
                break;

            case 3:
                // Step 4: Raise blocker servo
                if (elapsed < 0.2) {
                    blockerServo.setPosition(BLOCKER_UP_POS);
                } else {
                    currentShootingStep++;
                    stepTimer.reset();
                }
                break;

            case 4:
                // Step 5: Lower outtake servo
                if (elapsed < 0.2) {
                    outtakeServo.setPosition(OUTTAKE_UP_POS);
                } else {
                    currentShootingStep++;
                    stepTimer.reset();
                }
                break;

            case 5:
                // Step 6: Spin intake motor forward by the configured rotation at 50% power using encoders
                if (!waitingForIntakeRotation) {
                    intakeStartPosition = intakeMotor.getCurrentPosition();
                    intakeMoveTicks = INTAKE_FORWARD_TICKS;
                    int targetPosition = intakeStartPosition + intakeMoveTicks;
                    intakeMotor.setTargetPosition(targetPosition);
                    intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    intakeMotor.setPower(INTAKE_ROTATION_SPEED);
                    waitingForIntakeRotation = true;
                } else if (!intakeMotor.isBusy() ||
                        Math.abs(intakeMotor.getCurrentPosition() - intakeStartPosition) >= intakeMoveTicks) {
                    intakeMotor.setPower(0.0);
                    intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    waitingForIntakeRotation = false;
                    intakeMoveTicks = 0;
                    currentShootingStep++;
                    stepTimer.reset();
                }
                break;

            case 6:
                // Step 7: Lower blocker servo
                if (elapsed < 0.2) {
                    blockerServo.setPosition(BLOCKER_DOWN_POS);
                } else {
                    shotsFired++;
                    if (shotsFired >= shotsToFire) {
                        // All shots done
                        shootingState = ShootingState.IDLE;
                        aClickCount = 0;
                        yClickCount = 0;
                        stopShooterPid();
                    } else {
                        // Repeat steps
                        currentShootingStep = 0;
                    }
                    stepTimer.reset();
                }
                break;
        }
    }

    private void updateShooterVelocityControl() {
        if (!shooterPidEnabled) {
            shooterTimer.reset();
            filteredLeftShooterRpm = 0.0;
            filteredRightShooterRpm = 0.0;
            lastLeftShooterPowerCommand = 0.0;
            lastRightShooterPowerCommand = 0.0;
            return;
        }

        double dt = shooterTimer.seconds();
        shooterTimer.reset();
        if (dt <= 0.0) {
            dt = 0.001;
        } else if (dt > 0.5) {
            dt = 0.5;
        }

        int currentLeftPosition = leftOuttakeMotor.getCurrentPosition();
        int currentRightPosition = rightOuttakeMotor.getCurrentPosition();

        double leftDelta = currentLeftPosition - lastLeftShooterPosition;
        double rightDelta = currentRightPosition - lastRightShooterPosition;
        lastLeftShooterPosition = currentLeftPosition;
        lastRightShooterPosition = currentRightPosition;

        double leftRawRpm = ticksToRpm(leftDelta, dt);
        double rightRawRpm = ticksToRpm(rightDelta, dt);

        filteredLeftShooterRpm = applyFilter(filteredLeftShooterRpm, leftRawRpm, SHOOTER_RPM_FILTER_ALPHA);
        filteredRightShooterRpm = applyFilter(filteredRightShooterRpm, rightRawRpm, SHOOTER_RPM_FILTER_ALPHA);

        double leftPower = leftShooterPid.update(targetLeftShooterRpm, filteredLeftShooterRpm, dt);
        double rightPower = rightShooterPid.update(targetRightShooterRpm, filteredRightShooterRpm, dt);

        double limitedLeft = limitShooterPower(leftPower);
        double limitedRight = limitShooterPower(rightPower);

        leftOuttakeMotor.setPower(limitedLeft);
        rightOuttakeMotor.setPower(limitedRight);

        lastLeftShooterPowerCommand = limitedLeft;
        lastRightShooterPowerCommand = limitedRight;
    }

    private void enableShooterPid() {
        targetShooterRpmBase = SHOOTER_TARGET_RPM;
        targetLeftShooterRpm = SHOOTER_TARGET_RPM;
        targetRightShooterRpm = SHOOTER_TARGET_RPM;

        if (!shooterPidEnabled) {
            shooterPidEnabled = true;
            shooterTimer.reset();
            shooterRampTimer.reset();
            lastLeftShooterPosition = leftOuttakeMotor.getCurrentPosition();
            lastRightShooterPosition = rightOuttakeMotor.getCurrentPosition();
            filteredLeftShooterRpm = 0.0;
            filteredRightShooterRpm = 0.0;
            leftShooterPid.reset();
            rightShooterPid.reset();
        }
    }

    private void stopShooterPid() {
        if (shooterPidEnabled) {
            shooterPidEnabled = false;
            leftOuttakeMotor.setPower(0.0);
            rightOuttakeMotor.setPower(0.0);
            leftShooterPid.reset();
            rightShooterPid.reset();
        }
        targetShooterRpmBase = 0.0;
        targetLeftShooterRpm = 0.0;
        targetRightShooterRpm = 0.0;
        lastLeftShooterPowerCommand = 0.0;
        lastRightShooterPowerCommand = 0.0;
    }

    private double limitShooterPower(double requestedPower) {
        double clipped = Range.clip(requestedPower, 0.0, SHOOTER_POWER_MAX);
        if (shooterRampTimer.seconds() < SHOOTER_RAMP_DURATION_S) {
            clipped = Math.min(clipped, SHOOTER_RAMP_MAX_POWER);
        }
        return clipped;
    }

    private double ticksToRpm(double deltaTicks, double dtSeconds) {
        double revs = deltaTicks / SHOOTER_COUNTS_PER_REV;
        return (revs / dtSeconds) * 60.0;
    }

    private double applyFilter(double previous, double raw, double alpha) {
        if (Double.isNaN(raw) || Double.isInfinite(raw)) {
            return previous;
        }
        return previous + alpha * (raw - previous);
    }

    private void cancelSequence() {
        shootingState = ShootingState.IDLE;
        aClickCount = 0;
        yClickCount = 0;
        shotsFired = 0;
        currentShootingStep = 0;

        outtakeServo.setPosition(OUTTAKE_DOWN_POS);
        blockerServo.setPosition(BLOCKER_UP_POS);
        stopShooterPid();
        intakeMotor.setPower(0.0);
        frontIntakeMotor.setPower(0.0);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitingForIntakeRotation = false;
        intakeOn = false;
    }

    // Getters for telemetry
    public int getAClickCount() { return aClickCount; }
    public int getYClickCount() { return yClickCount; }
    public ShootingState getShootingState() { return shootingState; }
    public int getShotsFired() { return shotsFired; }
    public boolean isIntakeOn() { return intakeOn; }
    public boolean isShooterPidEnabled() { return shooterPidEnabled; }

    public double getTargetShooterRpm() { return targetShooterRpmBase; }
    public double getLeftTargetShooterRpm() { return targetLeftShooterRpm; }
    public double getRightTargetShooterRpm() { return targetRightShooterRpm; }
    public double getLeftShooterRpm() { return filteredLeftShooterRpm; }
    public double getRightShooterRpm() { return filteredRightShooterRpm; }
    public double getLeftShooterPowerCommand() { return lastLeftShooterPowerCommand; }
    public double getRightShooterPowerCommand() { return lastRightShooterPowerCommand; }

    public ShooterPidTelemetry getLeftShooterPidTelemetry() {
        return leftShooterPid.getLastTelemetry();
    }

    public ShooterPidTelemetry getRightShooterPidTelemetry() {
        return rightShooterPid.getLastTelemetry();
    }

    private static class ShooterPidController {
        private final double kP;
        private final double kI;
        private final double kD;
        private final double integralLimit;

        private double integral;
        private double previousError;
        private boolean firstUpdate = true;

        private double lastTargetRpm;
        private double lastMeasuredRpm;
        private double lastErrorRpm;
        private double lastDerivative;
        private double lastPTerm;
        private double lastITerm;
        private double lastDTerm;
        private double lastOutput;

        ShooterPidController(double kP, double kI, double kD, double integralLimit) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.integralLimit = Math.abs(integralLimit);
            clearTelemetry();
        }

        double update(double targetRpm, double measuredRpm, double dtSeconds) {
            if (targetRpm <= 0.0) {
                reset();
                lastMeasuredRpm = measuredRpm;
                return 0.0;
            }

            double safeDt = dtSeconds > 0.0 ? dtSeconds : 1e-3;
            double error = targetRpm - measuredRpm;

            integral += error * safeDt;
            integral = clipIntegral(integral);

            double derivative;
            if (firstUpdate) {
                derivative = 0.0;
                firstUpdate = false;
            } else {
                derivative = (error - previousError) / safeDt;
            }

            previousError = error;

            double pTerm = kP * error;
            double iTerm = kI * integral;
            double dTerm = kD * derivative;

            double output = Range.clip(pTerm + iTerm + dTerm, 0.0, 1.0);

            lastTargetRpm = targetRpm;
            lastMeasuredRpm = measuredRpm;
            lastErrorRpm = error;
            lastDerivative = derivative;
            lastPTerm = pTerm;
            lastITerm = iTerm;
            lastDTerm = dTerm;
            lastOutput = output;

            return output;
        }

        void reset() {
            integral = 0.0;
            previousError = 0.0;
            firstUpdate = true;
            clearTelemetry();
        }

        private double clipIntegral(double value) {
            if (integralLimit <= 0.0) {
                return value;
            }
            return Math.max(-integralLimit, Math.min(integralLimit, value));
        }

        private void clearTelemetry() {
            lastTargetRpm = 0.0;
            lastMeasuredRpm = 0.0;
            lastErrorRpm = 0.0;
            lastDerivative = 0.0;
            lastPTerm = 0.0;
            lastITerm = 0.0;
            lastDTerm = 0.0;
            lastOutput = 0.0;
        }

        ShooterPidTelemetry getLastTelemetry() {
            return new ShooterPidTelemetry(
                    lastTargetRpm,
                    lastMeasuredRpm,
                    lastErrorRpm,
                    lastPTerm,
                    lastITerm,
                    lastDTerm,
                    lastDerivative,
                    lastOutput,
                    integral
            );
        }
    }

    public static class ShooterPidTelemetry {
        public final double targetRpm;
        public final double measuredRpm;
        public final double errorRpm;
        public final double pTerm;
        public final double iTerm;
        public final double dTerm;
        public final double derivativeRpmPerSec;
        public final double output;
        public final double integralState;

        ShooterPidTelemetry(double targetRpm,
                            double measuredRpm,
                            double errorRpm,
                            double pTerm,
                            double iTerm,
                            double dTerm,
                            double derivativeRpmPerSec,
                            double output,
                            double integralState) {
            this.targetRpm = targetRpm;
            this.measuredRpm = measuredRpm;
            this.errorRpm = errorRpm;
            this.pTerm = pTerm;
            this.iTerm = iTerm;
            this.dTerm = dTerm;
            this.derivativeRpmPerSec = derivativeRpmPerSec;
            this.output = output;
            this.integralState = integralState;
        }
    }
}
