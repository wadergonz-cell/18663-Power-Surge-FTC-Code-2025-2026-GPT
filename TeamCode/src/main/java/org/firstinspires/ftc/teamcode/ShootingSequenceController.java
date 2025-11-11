// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ShootingSequenceController.java
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    private double shooterTargetRpm = RobotConstants.SHOOTER_TARGET_RPM;
    private double leftShooterRpm = 0.0;
    private double rightShooterRpm = 0.0;
    private boolean longRangeMode = false;

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

    // Shooter power (constant)
    public static final double SHOOTER_SPIN_POWER = 1.0;
    private boolean shooterSpinning = false;
    private double currentShooterPowerCommand = 0.0;

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
    }

    /**
     * Call every TeleOp loop with gamepad input and AprilTag info.
     */
    public void update(boolean aPressed, boolean bPressed, boolean yPressed, boolean xPressed,
                       AprilTagCalibration.Report tagReport) {

        updateShooterVelocityEstimate();

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
    }

    public void setLongRangeMode(boolean enabled) {
        if (enabled == longRangeMode) {
            return;
        }
        longRangeMode = enabled;

        if (shooterSpinning) {
            applyCurrentShooterModePower();
        }
    }

    public boolean isLongRangeModeActive() {
        return longRangeMode;
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
        startShooter();
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

            case 1: {
                // Step 2: Allow shooter to spin before releasing the ring
                double requiredDelay = longRangeMode
                        ? RobotConstants.LONG_RANGE_PREFIRE_DELAY_SEC
                        : RobotConstants.REGULAR_PREFIRE_DELAY_SEC;

                if (longRangeMode) {
                    setShooterPower(RobotConstants.LONG_RANGE_BURST_POWER);
                } else {
                    spinUpMotors();
                }

                if (elapsed >= requiredDelay) {
                    outtakeServo.setPosition(OUTTAKE_DOWN_POS);
                    currentShootingStep++;
                    stepTimer.reset();
                } else {
                    outtakeServo.setPosition(OUTTAKE_UP_POS);
                }
                break;
            }

            case 2:
                // Step 3: Keep the outtake servo down long enough for the ring to clear
                outtakeServo.setPosition(OUTTAKE_DOWN_POS);
                if (elapsed >= RobotConstants.OUTTAKE_RELEASE_DURATION_SEC) {
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
                    if (longRangeMode) {
                        setShooterPower(RobotConstants.LONG_RANGE_IDLE_POWER);
                    }
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
        shooterSpinning = true;
        applyCurrentShooterModePower();
        shooterTargetRpm = RobotConstants.SHOOTER_TARGET_RPM;
    }

    private void stopShooter() {
        leftOuttakeMotor.setPower(0.0);
        rightOuttakeMotor.setPower(0.0);
        shooterSpinning = false;
        currentShooterPowerCommand = 0.0;
        shooterTargetRpm = 0.0;
        setLongRangeMode(false);
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

    public double getShooterPowerCommand() {
        return currentShooterPowerCommand;
    }

    public double getShooterTargetRpm() {
        return shooterTargetRpm;
    }

    public double getLeftShooterRpm() {
        return leftShooterRpm;
    }

    public double getRightShooterRpm() {
        return rightShooterRpm;
    }

    public void setShooterTargetRpm(double targetRpm) {
        shooterTargetRpm = targetRpm;
    }

    private void applyCurrentShooterModePower() {
        double power = longRangeMode ? RobotConstants.LONG_RANGE_IDLE_POWER : SHOOTER_SPIN_POWER;
        setShooterPower(power);
    }

    private void setShooterPower(double power) {
        leftOuttakeMotor.setPower(power);
        rightOuttakeMotor.setPower(power);
        currentShooterPowerCommand = power;
    }

    private void updateShooterVelocityEstimate() {
        leftShooterRpm = ticksPerSecondToRpm(leftOuttakeMotor.getVelocity());
        rightShooterRpm = ticksPerSecondToRpm(rightOuttakeMotor.getVelocity());
    }

    private double ticksPerSecondToRpm(double ticksPerSecond) {
        return (ticksPerSecond * 60.0) / RobotConstants.SHOOTER_TICKS_PER_REV;
    }
}
