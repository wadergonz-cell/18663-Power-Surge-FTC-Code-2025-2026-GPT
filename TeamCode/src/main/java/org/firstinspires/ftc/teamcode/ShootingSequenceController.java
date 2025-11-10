// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ShootingSequenceController.java
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Manages the shooting sequence state machine.
 * Handles A/B/Y/X button clicks and the multi-step shooting logic.
 */
public class ShootingSequenceController {

    // Hardware references
    private final DcMotorEx leftOuttakeMotor;
    private final DcMotorEx rightOuttakeMotor;
    private final DcMotor intakeMotor;
    private final DcMotor frontIntakeMotor;
    private final Servo outtakeServo;
    private final Servo blockerServo;

    // Servo positions (FIXED: swapped UP and DOWN for outtake)
    private static final double SERVO_DEGREES_TO_RANGE = 1.0 / 180.0;
    private static final double OUTTAKE_UP_POS   = 0.45 - (3 * SERVO_DEGREES_TO_RANGE);
    private static final double OUTTAKE_DOWN_POS = 0.245 - (8 * SERVO_DEGREES_TO_RANGE);
    private static final double BLOCKER_UP_POS   = 0.00;
    private static final double BLOCKER_DOWN_POS = 0.55;

    // Intake speeds
    private static final double INTAKE_FULL_SPEED = 1.0;
    private static final double FRONT_INTAKE_SPEED = -0.35;
    private static final double INTAKE_HOLD_SPEED = 0.25;
    private static final double INTAKE_ROTATION_SPEED = 0.4;

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
    private static final double INTAKE_REVERSE_ROTATIONS = 0.0; // retract a quarter turn for consistent staging
    private static final double INTAKE_FORWARD_ROTATIONS = 0.8;  // advance one turn to feed the next ring
    private static final double ENCODER_COUNTS_PER_ROTATION = 537.6;
    private static final int INTAKE_REVERSE_TICKS = (int) Math.max(1,
            Math.round(ENCODER_COUNTS_PER_ROTATION * INTAKE_REVERSE_ROTATIONS));
    private static final int INTAKE_FORWARD_TICKS = (int) Math.max(1,
            Math.round(ENCODER_COUNTS_PER_ROTATION * INTAKE_FORWARD_ROTATIONS));
    private int intakeStartPosition = 0;
    private boolean waitingForIntakeRotation = false;
    private int intakeMoveTicks = 0;

    // Shooter velocity control
    public static final double DEFAULT_SHOOTER_TARGET_RPM = 5500;
    private static final double SHOOTER_KP = 0.000075;
    private static final double SHOOTER_KI = 0.05;
    private static final double SHOOTER_KD = 0.00075;
    private static final double SHOOTER_INTEGRAL_LIMIT = 1.0;

    private static final double INTER_SHOT_PAUSE_SECONDS = 1.0;

    private final double shooterTicksPerRev;
    private final PidfController leftShooterController;
    private final PidfController rightShooterController;
    private final ElapsedTime shooterPidTimer = new ElapsedTime();
    private boolean shooterSpinning = false;
    private double leftShooterTargetRpm = 0.0;
    private double rightShooterTargetRpm = 0.0;
    private double lastLeftShooterPower = 0.0;
    private double lastRightShooterPower = 0.0;
    private double lastLeftMeasuredRpm = 0.0;
    private double lastRightMeasuredRpm = 0.0;

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

        this.intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterTicksPerRev = leftOuttakeMotor.getMotorType().getTicksPerRev();
        double rawMaxRpm = leftOuttakeMotor.getMotorType().getMaxRPM();
        double normalizedMaxRpm = rawMaxRpm > 0.0 ? rawMaxRpm : DEFAULT_SHOOTER_TARGET_RPM;
        double feedForwardGain = 1.0 / Math.max(1.0, normalizedMaxRpm);

        leftShooterController = new PidfController(SHOOTER_KP, SHOOTER_KI, SHOOTER_KD,
                feedForwardGain, SHOOTER_INTEGRAL_LIMIT);
        rightShooterController = new PidfController(SHOOTER_KP, SHOOTER_KI, SHOOTER_KD,
                feedForwardGain, SHOOTER_INTEGRAL_LIMIT);
        shooterPidTimer.reset();
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

        double dt = shooterPidTimer.seconds();
        shooterPidTimer.reset();
        applyShooterPidControl(dt);
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
        if (!shooterSpinning) {
            startShooter();
        } else {
            setShooterTargetRpm(DEFAULT_SHOOTER_TARGET_RPM, DEFAULT_SHOOTER_TARGET_RPM);
        }
    }

    private void updateShootingSequence() {
        if (shootingState == ShootingState.SPINUP) {
            // Keep motors at target power during spinup phase
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
                // Step 2: Hold shooter power before firing
                if (elapsed < 0.1) {
                    spinUpMotors();
                } else {
                    currentShootingStep++;
                    stepTimer.reset();
                }
                break;

            case 2:
                // Step 3: Optional inter-shot pause (skipped for first shot)
                if (shotsFired > 0) {
                    if (elapsed < INTER_SHOT_PAUSE_SECONDS) {
                        spinUpMotors();
                        break;
                    }
                }
                currentShootingStep++;
                stepTimer.reset();
                break;

            case 3:
                // Step 4: Raise outtake servo (fire ring)
                if (elapsed < 0.2) {
                    outtakeServo.setPosition(OUTTAKE_DOWN_POS);
                } else {
                    currentShootingStep++;
                    stepTimer.reset();
                }
                break;

            case 4:
                // Step 5: Raise blocker servo
                if (elapsed < 0.2) {
                    blockerServo.setPosition(BLOCKER_UP_POS);
                } else {
                    currentShootingStep++;
                    stepTimer.reset();
                }
                break;

            case 5:
                // Step 6: Lower outtake servo
                if (elapsed < 0.2) {
                    outtakeServo.setPosition(OUTTAKE_UP_POS);
                } else {
                    currentShootingStep++;
                    stepTimer.reset();
                }
                break;

            case 6:
                // Step 7: Spin intake motor forward by the configured rotation using encoders
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

            case 7:
                // Step 8: Lower blocker servo
                if (elapsed < 0.2) {
                    blockerServo.setPosition(BLOCKER_DOWN_POS);
                } else {
                    shotsFired++;
                    if (shotsFired >= shotsToFire) {
                        // All shots done
                        shootingState = ShootingState.IDLE;
                        aClickCount = 0;
                        yClickCount = 0;
                        stopShooter();
                    } else {
                        // Repeat steps
                        currentShootingStep = 0;
                    }
                    stepTimer.reset();
                }
                break;
        }
    }
    private void startShooter() {
        leftShooterController.reset();
        rightShooterController.reset();
        shooterPidTimer.reset();
        shooterSpinning = true;
        setShooterTargetRpm(DEFAULT_SHOOTER_TARGET_RPM, DEFAULT_SHOOTER_TARGET_RPM);
    }

    private void stopShooter() {
        setShooterTargetRpm(0.0, 0.0);
        leftOuttakeMotor.setPower(0.0);
        rightOuttakeMotor.setPower(0.0);
        leftShooterController.reset();
        rightShooterController.reset();
        lastLeftShooterPower = 0.0;
        lastRightShooterPower = 0.0;
        shooterSpinning = false;
    }

    private void cancelSequence() {
        shootingState = ShootingState.IDLE;
        aClickCount = 0;
        yClickCount = 0;
        shotsFired = 0;
        currentShootingStep = 0;

        outtakeServo.setPosition(OUTTAKE_DOWN_POS);
        blockerServo.setPosition(BLOCKER_UP_POS);
        stopShooter();
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

    public boolean isShooterSpinning() {
        return shooterSpinning;
    }

    public double getLeftShooterTargetRpm() {
        return leftShooterTargetRpm;
    }

    public double getRightShooterTargetRpm() {
        return rightShooterTargetRpm;
    }

    public double getLeftShooterMeasuredRpm() {
        return lastLeftMeasuredRpm;
    }

    public double getRightShooterMeasuredRpm() {
        return lastRightMeasuredRpm;
    }

    public double getLeftShooterPowerCommand() {
        return lastLeftShooterPower;
    }

    public double getRightShooterPowerCommand() {
        return lastRightShooterPower;
    }

    private void setShooterTargetRpm(double leftRpm, double rightRpm) {
        leftShooterTargetRpm = Math.max(0.0, leftRpm);
        rightShooterTargetRpm = Math.max(0.0, rightRpm);
    }

    private void applyShooterPidControl(double dtSeconds) {
        lastLeftMeasuredRpm = ticksPerSecondToRpm(leftOuttakeMotor.getVelocity());
        lastRightMeasuredRpm = ticksPerSecondToRpm(rightOuttakeMotor.getVelocity());

        if (leftShooterTargetRpm <= 0.0 && rightShooterTargetRpm <= 0.0) {
            leftOuttakeMotor.setPower(0.0);
            rightOuttakeMotor.setPower(0.0);
            lastLeftShooterPower = 0.0;
            lastRightShooterPower = 0.0;
            leftShooterController.reset();
            rightShooterController.reset();
            return;
        }

        double leftOutput = leftShooterController.update(leftShooterTargetRpm,
                lastLeftMeasuredRpm, dtSeconds);
        double rightOutput = rightShooterController.update(rightShooterTargetRpm,
                lastRightMeasuredRpm, dtSeconds);

        lastLeftShooterPower = Range.clip(leftOutput, -1.0, 1.0);
        lastRightShooterPower = Range.clip(rightOutput, -1.0, 1.0);

        leftOuttakeMotor.setPower(lastLeftShooterPower);
        rightOuttakeMotor.setPower(lastRightShooterPower);
    }

    private double ticksPerSecondToRpm(double ticksPerSecond) {
        if (shooterTicksPerRev <= 0.0) {
            return 0.0;
        }
        return (ticksPerSecond / shooterTicksPerRev) * 60.0;
    }

    private static class PidfController {
        private final double kP;
        private final double kI;
        private final double kD;
        private final double kF;
        private final double integralLimit;

        private double integral;
        private double lastError;
        private boolean initialized;

        PidfController(double kP, double kI, double kD, double kF, double integralLimit) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
            this.integralLimit = integralLimit;
            reset();
        }

        double update(double targetRpm, double currentRpm, double dtSeconds) {
            double error = targetRpm - currentRpm;

            if (targetRpm <= 0.0) {
                reset();
                return 0.0;
            }

            if (!initialized) {
                lastError = error;
                initialized = true;
            }

            if (dtSeconds > 0.0) {
                integral += error * dtSeconds;
                integral = Range.clip(integral, -integralLimit, integralLimit);
            }

            double derivative = 0.0;
            if (dtSeconds > 0.0) {
                derivative = (error - lastError) / dtSeconds;
            }

            lastError = error;

            double feedForward = targetRpm * kF;
            return feedForward + (kP * error) + (kI * integral) + (kD * derivative);
        }

        void reset() {
            integral = 0.0;
            lastError = 0.0;
            initialized = false;
        }
    }
}