package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;

/**
 * TeleOp for gathering shooter RPM vs. distance calibration data.
 * <p>
 * Controls:
 * A - Toggle shooter PID on/off
 * B - Toggle flipper (outtake servo)
 * Right bumper - Next preset RPM
 * Left bumper - Previous preset RPM
 * D-Pad up/down - Fine adjust target RPM (+/-5)
 * X - Log current data snapshot to telemetry log
 */
@TeleOp(name = "Shooter Speed Tuning", group = "Tuning")
public class ShooterSpeedTuningTeleOp extends LinearOpMode {

    private static final double SHOOTER_COUNTS_PER_REV = 537.6; // Update if your gearing differs
    private static final double RPM_MIN = 0.0;
    private static final double RPM_MAX = 200.0; // Matches current distance->RPM model range
    private static final double RPM_STEP_FINE = 5.0;
    private static final double[] PRESET_RPMS = {60.0, 80.0, 100.0, 120.0, 140.0, 160.0, 180.0, 200.0};

    private static final double SERVO_UP_POS = 0.45;
    private static final double SERVO_DOWN_POS = 0.245;
    private static final double BLOCKER_UP_POS = 0.0;

    private static final double RPM_FILTER_ALPHA = 0.25; // 0..1, higher = less smoothing

    private static final double PID_KP = 0.010;
    private static final double PID_KI = 0.002;
    private static final double PID_KD = 0.0005;

    private robotHardware RobotHardware;
    private DcMotor leftShooter;
    private DcMotor rightShooter;
    private Servo outtakeServo;
    private Servo blockerServo;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware = new robotHardware(this);

        leftShooter = RobotHardware.leftOuttakeMotor;
        rightShooter = RobotHardware.rightOuttakeMotor;
        outtakeServo = RobotHardware.outtakeServo;
        blockerServo = RobotHardware.ballBlocker;

        configureShooterMotor(leftShooter);
        configureShooterMotor(rightShooter);

        outtakeServo.setPosition(SERVO_UP_POS);
        blockerServo.setPosition(BLOCKER_UP_POS);

        telemetry.addLine("Shooter Speed Tuning Ready");
        telemetry.addLine("Press A to arm PID, use bumpers/D-pad to change RPM");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        ShooterPidController leftPid = new ShooterPidController(PID_KP, PID_KI, PID_KD, RPM_MAX);
        ShooterPidController rightPid = new ShooterPidController(PID_KP, PID_KI, PID_KD, RPM_MAX);

        boolean motorsEnabled = false;
        boolean flipperExtended = false;

        double targetRpm = PRESET_RPMS[0];
        int presetIndex = 0;

        boolean prevA = false;
        boolean prevB = false;
        boolean prevX = false;
        boolean prevDpadUp = false;
        boolean prevDpadDown = false;
        boolean prevRightBumper = false;
        boolean prevLeftBumper = false;

        int lastLeftPosition = leftShooter.getCurrentPosition();
        int lastRightPosition = rightShooter.getCurrentPosition();
        double filteredLeftRpm = 0.0;
        double filteredRightRpm = 0.0;
        double leftPowerCmd = 0.0;
        double rightPowerCmd = 0.0;

        ElapsedTime loopTimer = new ElapsedTime();
        loopTimer.reset();

        while (opModeIsActive()) {
            double dt = loopTimer.seconds();
            loopTimer.reset();
            if (dt <= 0.0) {
                dt = 0.001;
            } else if (dt > 0.5) {
                dt = 0.5; // guard against long pauses causing large derivative spikes
            }

            boolean aPressed = gamepad1.a;
            boolean bPressed = gamepad1.b;
            boolean xPressed = gamepad1.x;
            boolean dpadUp = gamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down;
            boolean rightBumper = gamepad1.right_bumper;
            boolean leftBumper = gamepad1.left_bumper;

            if (aPressed && !prevA) {
                motorsEnabled = !motorsEnabled;
                if (!motorsEnabled) {
                    leftShooter.setPower(0.0);
                    rightShooter.setPower(0.0);
                    leftPid.reset();
                    rightPid.reset();
                } else {
                    leftPid.reset();
                    rightPid.reset();
                }
            }
            prevA = aPressed;

            if (bPressed && !prevB) {
                flipperExtended = !flipperExtended;
                outtakeServo.setPosition(flipperExtended ? SERVO_DOWN_POS : SERVO_UP_POS);
            }
            prevB = bPressed;

            if (rightBumper && !prevRightBumper) {
                presetIndex = (presetIndex + 1) % PRESET_RPMS.length;
                targetRpm = PRESET_RPMS[presetIndex];
            }
            prevRightBumper = rightBumper;

            if (leftBumper && !prevLeftBumper) {
                presetIndex = (presetIndex - 1 + PRESET_RPMS.length) % PRESET_RPMS.length;
                targetRpm = PRESET_RPMS[presetIndex];
            }
            prevLeftBumper = leftBumper;

            if (dpadUp && !prevDpadUp) {
                targetRpm = clip(targetRpm + RPM_STEP_FINE, RPM_MIN, RPM_MAX);
                presetIndex = -1;
            }
            prevDpadUp = dpadUp;

            if (dpadDown && !prevDpadDown) {
                targetRpm = clip(targetRpm - RPM_STEP_FINE, RPM_MIN, RPM_MAX);
                presetIndex = -1;
            }
            prevDpadDown = dpadDown;

            int currentLeftPosition = leftShooter.getCurrentPosition();
            int currentRightPosition = rightShooter.getCurrentPosition();

            double leftDelta = currentLeftPosition - lastLeftPosition;
            double rightDelta = currentRightPosition - lastRightPosition;
            lastLeftPosition = currentLeftPosition;
            lastRightPosition = currentRightPosition;

            double leftRawRpm = ticksToRpm(leftDelta, dt);
            double rightRawRpm = ticksToRpm(rightDelta, dt);

            filteredLeftRpm = applyFilter(filteredLeftRpm, leftRawRpm, RPM_FILTER_ALPHA);
            filteredRightRpm = applyFilter(filteredRightRpm, rightRawRpm, RPM_FILTER_ALPHA);

            if (motorsEnabled) {
                leftPowerCmd = leftPid.update(targetRpm, filteredLeftRpm, dt);
                rightPowerCmd = rightPid.update(targetRpm, filteredRightRpm, dt);
                leftShooter.setPower(leftPowerCmd);
                rightShooter.setPower(rightPowerCmd);
            } else {
                leftShooter.setPower(0.0);
                rightShooter.setPower(0.0);
            }

            if (xPressed && !prevX) {
                telemetry.log().add(String.format(Locale.US,
                        "Sample -> Target: %.1f rpm | L: %.1f rpm | R: %.1f rpm | Power L/R: %.2f / %.2f",
                        targetRpm, filteredLeftRpm, filteredRightRpm, leftPowerCmd, rightPowerCmd));
            }
            prevX = xPressed;

            telemetry.addLine("Controls: A=Toggle PID  B=Toggle Flipper  LB/RB=Preset  Dpad Up/Down=Fine +/-5  X=Log Sample");
            telemetry.addData("PID Enabled", motorsEnabled);
            telemetry.addData("Target RPM", "%.1f", targetRpm);
            telemetry.addData("Preset", presetIndex >= 0 ? String.format(Locale.US, "#%d (%.0f rpm)", presetIndex + 1, PRESET_RPMS[presetIndex]) : "Manual");
            telemetry.addData("Left RPM", "%.1f (raw %.1f)", filteredLeftRpm, leftRawRpm);
            telemetry.addData("Right RPM", "%.1f (raw %.1f)", filteredRightRpm, rightRawRpm);
            telemetry.addData("Left Power", "%.3f", leftPowerCmd);
            telemetry.addData("Right Power", "%.3f", rightPowerCmd);
            telemetry.addData("Flipper", flipperExtended ? "DOWN" : "UP");
            telemetry.addData("Loop dt", "%.2f ms", dt * 1000.0);
            telemetry.update();

            idle();
        }

        leftShooter.setPower(0.0);
        rightShooter.setPower(0.0);
        outtakeServo.setPosition(SERVO_UP_POS);
    }

    private void configureShooterMotor(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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

    private double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

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
            double error = targetRpm - measuredRpm;

            integral += error * dtSeconds;
            integral = clipIntegral(integral);

            double derivative = 0.0;
            if (!firstUpdate && dtSeconds > 0.0) {
                derivative = (error - previousError) / dtSeconds;
            }
            firstUpdate = false;

            previousError = error;

            double feedforward = targetRpm / RPM_MAX;
            double correction = kP * error + kI * integral + kD * derivative;
            double output = feedforward + correction;
            return Math.max(0.0, Math.min(1.0, output));
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
