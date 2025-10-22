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

    // Intake rotation tracking for step 5
    private static final double ENCODER_COUNTS_PER_ROTATION = 537.6;
    private static final int INTAKE_ROTATION_TICKS = (int) Math.round(ENCODER_COUNTS_PER_ROTATION);
    private static final int INTAKE_HALF_ROTATION_TICKS = (int) Math.round(ENCODER_COUNTS_PER_ROTATION * 0.5);
    private int intakeStartPosition = 0;
    private boolean waitingForIntakeRotation = false;
    private int intakeMoveTicks = 0;

    // Shooter RPM offset (in inches) — tune this if always short/long
    private static final double SHOOTER_DISTANCE_OFFSET_IN = 30.0;
    private static final double SHOOTER_MIN_RPM = 80.0;
    private static final double SHOOTER_MAX_RPM = 200.0;
    /** Positive values lower the requested target RPM to counter overshoot. */
    private static final double LEFT_SHOOTER_TARGET_RPM_OFFSET = 2.0;
    /** Positive values lower the requested target RPM to counter overshoot. */
    private static final double RIGHT_SHOOTER_TARGET_RPM_OFFSET = 42.0;
    private static final double SHOOTER_COUNTS_PER_REV = 537.6;
    private static final double SHOOTER_RPM_FILTER_ALPHA = 0.25;
    // Left shooter PID gains (tune independently from the right side)
    private static final double LEFT_SHOOTER_PID_KP = 0.000000000000000005;
    private static final double LEFT_SHOOTER_PID_KI = 0.000;
    private static final double LEFT_SHOOTER_PID_KD = 0.00;
    // Right shooter PID gains
    private static final double RIGHT_SHOOTER_PID_KP = 0.00000000000000005;
    private static final double RIGHT_SHOOTER_PID_KI = 0.00;
    private static final double RIGHT_SHOOTER_PID_KD = 0.00;

    // Shooter velocity control
    private final ShooterPidController leftShooterPid = new ShooterPidController(
            LEFT_SHOOTER_PID_KP, LEFT_SHOOTER_PID_KI, LEFT_SHOOTER_PID_KD, SHOOTER_MAX_RPM);
    private final ShooterPidController rightShooterPid = new ShooterPidController(
            RIGHT_SHOOTER_PID_KP, RIGHT_SHOOTER_PID_KI, RIGHT_SHOOTER_PID_KD, SHOOTER_MAX_RPM);
    private final ElapsedTime shooterTimer = new ElapsedTime();
    private boolean shooterPidEnabled = false;
    private double targetShooterRpmBase = 0.0;
    private double targetLeftShooterRpm = 0.0;
    private double targetRightShooterRpm = 0.0;
    private double filteredLeftShooterRpm = 0.0;
    private double filteredRightShooterRpm = 0.0;
    private int lastLeftShooterPosition = 0;
    private int lastRightShooterPosition = 0;

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
            handleAClick(tagReport);
        }
        prevA = aPressed;

        // Run shooting sequence if active
        if (shootingState != ShootingState.IDLE) {
            updateShootingSequence(tagReport);
        }

        updateShooterVelocityControl();
    }

    private void handleAClick(AprilTagCalibration.Report tagReport) {
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
                spinUpMotors(tagReport);
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

    private void spinUpMotors(AprilTagCalibration.Report tagReport) {
        double targetRpm;
        if (tagReport.hasTag && tagReport.horizontalRangeIn <= 65.0) {
            double distanceWithOffset = tagReport.horizontalRangeIn + SHOOTER_DISTANCE_OFFSET_IN;
            targetRpm = calculateRpmFromDistance(distanceWithOffset);
        } else {
            targetRpm = SHOOTER_MIN_RPM; // fallback safe value
        }

        enableShooterPid(targetRpm);
    }

    private void updateShootingSequence(AprilTagCalibration.Report tagReport) {
        if (shootingState == ShootingState.SPINUP) {
            // Continuously adjust RPM during spinup phase
            spinUpMotors(tagReport);
            return;
        }

        if (shootingState != ShootingState.SHOOTING) {
            return;
        }

        double elapsed = stepTimer.seconds();

        // Each step has a duration; adjust timing as needed
        switch (currentShootingStep) {
            case 0:
                // Step 1: Retract intake by half a rotation at 50% power (reverse)
                if (!waitingForIntakeRotation) {
                    intakeStartPosition = intakeMotor.getCurrentPosition();
                    intakeMoveTicks = INTAKE_HALF_ROTATION_TICKS;
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
                    spinUpMotors(tagReport);
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
                // Step 6: Spin intake motor forward one rotation at 50% power using encoders
                if (!waitingForIntakeRotation) {
                    intakeStartPosition = intakeMotor.getCurrentPosition();
                    intakeMoveTicks = INTAKE_ROTATION_TICKS;
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

        leftOuttakeMotor.setPower(leftPower);
        rightOuttakeMotor.setPower(rightPower);
    }

    private void enableShooterPid(double targetRpm) {
        double clippedTarget = Range.clip(targetRpm, SHOOTER_MIN_RPM, SHOOTER_MAX_RPM);
        targetShooterRpmBase = clippedTarget;

        targetLeftShooterRpm = Range.clip(
                clippedTarget - LEFT_SHOOTER_TARGET_RPM_OFFSET,
                SHOOTER_MIN_RPM,
                SHOOTER_MAX_RPM);
        targetRightShooterRpm = Range.clip(
                clippedTarget - RIGHT_SHOOTER_TARGET_RPM_OFFSET,
                SHOOTER_MIN_RPM,
                SHOOTER_MAX_RPM);

        if (!shooterPidEnabled) {
            shooterPidEnabled = true;
            shooterTimer.reset();
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

    /**
     * Convert horizontal distance to RPM using the quadratic best-fit from on-field testing.
     * Data points provided: (170 RPM → 72 in), (127 RPM → 64 in), (95 RPM → 42 in).
     */
    private double calculateRpmFromDistance(double distanceIn) {
        double clampedDistance = Math.max(0.0, distanceIn);
        double rpm = 0.13068181818181818 * clampedDistance * clampedDistance
                - 12.397727272727273 * clampedDistance
                + 385.18181818181836;
        return Range.clip(rpm, SHOOTER_MIN_RPM, SHOOTER_MAX_RPM);
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

    private static class ShooterPidController {
        private final double kP;
        private final double kI;
        private final double kD;
        private final double integralLimit;

        private double integral;
        private double previousError;
        private boolean firstUpdate = true;

        ShooterPidController(double kP, double kI, double kD, double integralLimit) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.integralLimit = Math.abs(integralLimit);
        }

        double update(double targetRpm, double measuredRpm, double dtSeconds) {
            if (targetRpm <= 0.0) {
                // No target – let the caller cut power entirely.
                reset();
                return 0.0;
            }

            double error = targetRpm - measuredRpm;
            if (error < 0.0) {
                // If we are overspeed, allow the flywheel to coast by only trimming
                // power through the feedforward term (no braking).
                error = 0.0;
                integral = 0.0;
            } else {
                integral += error * dtSeconds;
                integral = clipIntegral(integral);
            }

            double derivative = 0.0;
            if (!firstUpdate && dtSeconds > 0.0) {
                double errorDelta = error - previousError;
                if (errorDelta > 0.0) {
                    derivative = errorDelta / dtSeconds;
                }
            }

            firstUpdate = false;
            previousError = error;

            double feedforward = targetRpm / SHOOTER_MAX_RPM;
            double correction = kP * error + kI * integral + kD * derivative;
            return Range.clip(feedforward + Math.max(0.0, correction), 0.0, 1.0);
        }

        void reset() {
            integral = 0.0;
            previousError = 0.0;
            firstUpdate = true;
        }

        private double clipIntegral(double value) {
            if (integralLimit <= 0.0) {
                return value;
            }
            return Math.max(-integralLimit, Math.min(integralLimit, value));
        }
    }
}
