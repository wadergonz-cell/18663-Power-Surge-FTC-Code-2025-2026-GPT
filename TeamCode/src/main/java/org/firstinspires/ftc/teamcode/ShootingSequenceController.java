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
    private static final double INTAKE_HOLD_SPEED = 0.35;
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
    private int intakeStartPosition = 0;
    private boolean waitingForIntakeRotation = false;

    // Shooter RPM offset (in inches) — tune this if always short/long
    private static final double SHOOTER_DISTANCE_OFFSET_IN = 0.0;
    private static final double SHOOTER_MIN_RPM = 80.0;
    private static final double SHOOTER_MAX_RPM = 200.0;

    public ShootingSequenceController(robotHardware hardware) {
        this.leftOuttakeMotor = hardware.leftOuttakeMotor;
        this.rightOuttakeMotor = hardware.rightOuttakeMotor;
        this.intakeMotor = hardware.intakeMotor;
        this.frontIntakeMotor = hardware.frontIntakeMotor;
        this.outtakeServo = hardware.outtakeServo;
        this.blockerServo = hardware.ballBlocker;

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

        double motorPower = rpmToPower(targetRpm);
        leftOuttakeMotor.setPower(motorPower);
        rightOuttakeMotor.setPower(motorPower);
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
                // Step 1: Adjust motor RPM if distance changed
                if (elapsed < 0.1) {
                    spinUpMotors(tagReport);
                } else {
                    currentShootingStep++;
                    stepTimer.reset();
                }
                break;

            case 1:
                // Step 2: Raise outtake servo (shoot first ball)
                if (elapsed < 0.2) {
                    outtakeServo.setPosition(OUTTAKE_DOWN_POS);
                } else {
                    currentShootingStep++;
                    stepTimer.reset();
                }
                break;

            case 2:
                // Step 3: Raise blocker servo
                if (elapsed < 0.2) {
                    blockerServo.setPosition(BLOCKER_UP_POS);
                } else {
                    currentShootingStep++;
                    stepTimer.reset();
                }
                break;

            case 3:
                // Step 4: Lower outtake servo
                if (elapsed < 0.2) {
                    outtakeServo.setPosition(OUTTAKE_UP_POS);
                } else {
                    currentShootingStep++;
                    stepTimer.reset();
                }
                break;

            case 4:
                // Step 5: Spin intake motor exactly one rotation at 50% power using encoders
                if (!waitingForIntakeRotation) {
                    intakeStartPosition = intakeMotor.getCurrentPosition();
                    int targetPosition = intakeStartPosition + INTAKE_ROTATION_TICKS;
                    intakeMotor.setTargetPosition(targetPosition);
                    intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    intakeMotor.setPower(INTAKE_ROTATION_SPEED);
                    waitingForIntakeRotation = true;
                } else if (!intakeMotor.isBusy() ||
                        Math.abs(intakeMotor.getCurrentPosition() - intakeStartPosition) >= INTAKE_ROTATION_TICKS) {
                    intakeMotor.setPower(0.0);
                    intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    waitingForIntakeRotation = false;
                    currentShootingStep++;
                    stepTimer.reset();
                }
                break;

            case 5:
                // Step 6: Lower blocker servo
                if (elapsed < 0.2) {
                    blockerServo.setPosition(BLOCKER_DOWN_POS);
                } else {
                    shotsFired++;
                    if (shotsFired >= shotsToFire) {
                        // All shots done
                        shootingState = ShootingState.IDLE;
                        aClickCount = 0;
                        yClickCount = 0;
                    } else {
                        // Repeat steps
                        currentShootingStep = 0;
                    }
                    stepTimer.reset();
                }
                break;
        }
    }

    private void cancelSequence() {
        shootingState = ShootingState.IDLE;
        aClickCount = 0;
        yClickCount = 0;
        shotsFired = 0;
        currentShootingStep = 0;

        outtakeServo.setPosition(OUTTAKE_DOWN_POS);
        blockerServo.setPosition(BLOCKER_UP_POS);
        leftOuttakeMotor.setPower(0.0);
        rightOuttakeMotor.setPower(0.0);
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

    /**
     * Convert RPM to motor power (0.0 to 1.0).
     * Adjust based on your motor's max RPM.
     */
    private double rpmToPower(double rpm) {
        double maxRpm = 1000.0; // Adjust for your motor
        return Math.max(0.0, Math.min(1.0, rpm / maxRpm));
    }

    // Getters for telemetry
    public int getAClickCount() { return aClickCount; }
    public int getYClickCount() { return yClickCount; }
    public ShootingState getShootingState() { return shootingState; }
    public int getShotsFired() { return shotsFired; }
    public boolean isIntakeOn() { return intakeOn; }
}