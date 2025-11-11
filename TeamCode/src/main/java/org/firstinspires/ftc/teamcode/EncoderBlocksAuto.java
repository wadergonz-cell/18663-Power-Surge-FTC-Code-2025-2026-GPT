package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto - Encoder Blocks", group = "Auto")
public class EncoderBlocksAuto extends LinearOpMode {

    private robotHardware hardware;
    private DriveBlocks drive;
    private AprilTagHelper vision;

    @Override
    public void runOpMode() {
        hardware = new robotHardware(this);
        drive = new DriveBlocks(this, hardware);
        vision = new AprilTagHelper(hardwareMap);
        vision.init();
        drive.initialize();

        telemetry.addLine("Encoder auto ready");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            vision.update();
            Integer startTag = AprilTagHelper.getStartZoneTagId();
            if (startTag != null) {
                telemetry.addLine("Start tag 🎯 - " + startTag);
            } else {
                telemetry.addLine("Start tag 🎯 - scanning...");
            }
            telemetry.update();
            sleep(50);
            telemetry.clearAll();
        }

        waitForStart();
        if (isStopRequested()) {
            vision.stop();
            return;
        }

        drive.driveForward(-3.0, 0.3);
        ShootConfig.rpmShots(2, 2).withDropBlockerOnStart(true);
        Integer startTag = drive.scanForStartZoneTag(vision, 2.0);

        if (startTag != null) {
            if (startTag == 21) {
                drive.driveForward(3.0, 0.3);
            } else if (startTag == 22) {
                drive.driveForward(1.0, 0.3);
            } else if (startTag == 23) {
                drive.driveForward(-2.0, 0.3);
            }
        } else {
            drive.turnDegrees(-45.0, 0.32);
        }

        drive.pause(2);

        boolean aligned = drive.autoAlignToTag(vision, 20, 3);

        drive.stopAll();
        drive.stopMechanisms();
        vision.stop();
    }

    /**
     * Helper that exposes simple "block-style" movement methods for autonomous paths.
     */
    public static class DriveBlocks {
        private final LinearOpMode opMode;
        private final DcMotor frontLeft;
        private final DcMotor frontRight;
        private final DcMotor backLeft;
        private final DcMotor backRight;
        private final DcMotorEx leftShooter;
        private final DcMotorEx rightShooter;
        private final DcMotor intake;
        private final DcMotor frontIntake;
        private final Servo outtakeServo;
        private final Servo blockerServo;

        public DriveBlocks(LinearOpMode opMode, robotHardware hardware) {
            this.opMode = opMode;
            this.frontLeft = hardware.frontLeftMotor;
            this.frontRight = hardware.frontRightMotor;
            this.backLeft = hardware.backLeftMotor;
            this.backRight = hardware.backRightMotor;
            this.leftShooter = hardware.leftOuttakeMotor;
            this.rightShooter = hardware.rightOuttakeMotor;
            this.intake = hardware.intakeMotor;
            this.frontIntake = hardware.frontIntakeMotor;
            this.outtakeServo = hardware.outtakeServo;
            this.blockerServo = hardware.ballBlocker;
        }

        public void initialize() {
            setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

            applyBrake();
        }

        public void stopAll() {
            setMotorPower(0.0);
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public void stopMechanisms() {
            setIntakePower(0.0, 0.0);
            stopShooter();
            outtakeServo.setPosition(RobotConstants.OUTTAKE_UP_POSITION);
            blockerServo.setPosition(RobotConstants.BLOCKER_UP_POSITION);
        }

        // === Block-style movement helpers ===
        public void driveForward(double inches) {
            driveForward(inches, RobotConstants.AUTO_DRIVE_MAX_POWER);
        }

        public void driveForward(double inches, double power) {
            runMecanum(inchesToTicks(inches), 0, power);
        }

        public void strafe(double inches) {
            strafe(inches, RobotConstants.AUTO_DRIVE_MAX_POWER);
        }

        public void strafe(double inches, double power) {
            runMecanum(0, inchesToTicks(inches), power);
        }

        public void driveAndStrafe(double forwardInches, double strafeInches) {
            driveAndStrafe(forwardInches, strafeInches, RobotConstants.AUTO_DRIVE_MAX_POWER);
        }

        public void driveAndStrafe(double forwardInches, double strafeInches, double power) {
            runMecanum(inchesToTicks(forwardInches), inchesToTicks(strafeInches), power);
        }

        public void turnDegrees(double degrees) {
            turnDegrees(degrees, RobotConstants.AUTO_TURN_MAX_POWER);
        }

        public void turnDegrees(double degrees, double power) {
            int turnTicks = degreesToTicks(degrees);
            runToPosition(turnTicks, -turnTicks, turnTicks, -turnTicks, power);
        }

        // === Mechanism helpers ===
        public void setIntakePower(double mainPower, double frontPower) {
            intake.setPower(mainPower);
            frontIntake.setPower(frontPower);
        }

        public void runIntake(double mainPower, double frontPower, double seconds) {
            setIntakePower(mainPower, frontPower);
            waitSeconds(seconds);
            setIntakePower(0.0, 0.0);
        }

        public void intakeForDefault(double seconds) {
            runIntake(RobotConstants.AUTO_INTAKE_POWER,
                    RobotConstants.AUTO_FRONT_INTAKE_POWER,
                    seconds);
        }

        public void pause(double seconds) {
            waitSeconds(seconds);
        }

        public void shooterPrimeBlockerDown() {
            blockerServo.setPosition(RobotConstants.BLOCKER_DOWN_POSITION);
        }

        public Integer scanForStartZoneTag(AprilTagHelper vision, double timeoutSec) {
            if (vision == null) {
                return null;
            }

            ElapsedTime timer = new ElapsedTime();
            timer.reset();
            while (opMode.opModeIsActive()
                    && (timeoutSec <= 0 || timer.seconds() < timeoutSec)
                    && !AprilTagHelper.hasStartZoneTag()) {
                vision.update();
                opMode.idle();
                opMode.sleep((int) Math.round(RobotConstants.AUTO_ALIGN_POLL_INTERVAL_SEC * 1000));
            }
            return AprilTagHelper.getStartZoneTagId();
        }

        public boolean autoAlignToTag(AprilTagHelper vision, int tagId, double timeoutSec) {
            if (vision == null) {
                return false;
            }

            ElapsedTime timer = new ElapsedTime();
            timer.reset();
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

            while (opMode.opModeIsActive() && (timeoutSec <= 0 || timer.seconds() < timeoutSec)) {
                vision.update();
                AprilTagHelper.DetectionInfo info = vision.getDetectionInfo(tagId);
                if (info == null) {
                    setMotorPower(0.0);
                    opMode.idle();
                    opMode.sleep((int) Math.round(RobotConstants.AUTO_ALIGN_POLL_INTERVAL_SEC * 1000));
                    continue;
                }

                double yawErrorDeg = info.getYawErrorDegrees();
                if (Math.abs(yawErrorDeg) <= RobotConstants.AUTO_ALIGN_TURN_TOLERANCE_DEG) {
                    setMotorPower(0.0);
                    return true;
                }

                double turnPower = Range.clip(
                        info.yawErrorRad * RobotConstants.AUTO_ALIGN_TURN_KP,
                        -RobotConstants.AUTO_ALIGN_TURN_MAX_POWER,
                        RobotConstants.AUTO_ALIGN_TURN_MAX_POWER
                );

                frontLeft.setPower(turnPower);
                backLeft.setPower(turnPower);
                frontRight.setPower(-turnPower);
                backRight.setPower(-turnPower);

                opMode.idle();
                opMode.sleep((int) Math.round(RobotConstants.AUTO_ALIGN_POLL_INTERVAL_SEC * 1000));
            }

            setMotorPower(0.0);
            return false;
        }

        public void shoot(ShootConfig config) {
            if (config == null || config.shots <= 0) {
                return;
            }

            ShooterMode mode = config.mode;
            int shotsRemaining = config.shots;

            if (config.dropBlockerOnStart) {
                shooterPrimeBlockerDown();
                waitSeconds(RobotConstants.AUTO_BLOCKER_RESET_DELAY_SEC);
            }

            outtakeServo.setPosition(RobotConstants.OUTTAKE_UP_POSITION);

            if (mode == ShooterMode.HIGH_SPEED) {
                setShooterPower(RobotConstants.LONG_RANGE_IDLE_POWER);
            } else {
                setShooterVelocity(config.targetRpm);
            }

            while (opMode.opModeIsActive() && shotsRemaining > 0) {
                if (mode == ShooterMode.HIGH_SPEED) {
                    setShooterPower(RobotConstants.LONG_RANGE_BURST_POWER);
                    waitSeconds(RobotConstants.LONG_RANGE_PREFIRE_DELAY_SEC);
                } else {
                    if (!waitForShooterReady(config.targetRpm, config.readyTimeoutSec)) {
                        break;
                    }
                    waitSeconds(RobotConstants.REGULAR_PREFIRE_DELAY_SEC);
                }

                outtakeServo.setPosition(RobotConstants.OUTTAKE_DOWN_POSITION);
                waitSeconds(RobotConstants.OUTTAKE_RELEASE_DURATION_SEC);
                outtakeServo.setPosition(RobotConstants.OUTTAKE_UP_POSITION);

                blockerServo.setPosition(RobotConstants.BLOCKER_UP_POSITION);
                waitSeconds(RobotConstants.AUTO_BLOCKER_RESET_DELAY_SEC);

                if (config.intakeAdvanceSeconds > 0.0) {
                    runIntake(config.intakeAdvancePower,
                            config.frontIntakeAdvancePower,
                            config.intakeAdvanceSeconds);
                }

                blockerServo.setPosition(RobotConstants.BLOCKER_DOWN_POSITION);

                if (mode == ShooterMode.HIGH_SPEED) {
                    setShooterPower(RobotConstants.LONG_RANGE_IDLE_POWER);
                } else {
                    // keep velocity target applied
                    setShooterVelocity(config.targetRpm);
                }

                shotsRemaining--;
            }

            stopShooter();
            blockerServo.setPosition(RobotConstants.BLOCKER_UP_POSITION);
        }

        private void setShooterPower(double power) {
            leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftShooter.setPower(power);
            rightShooter.setPower(power);
        }

        private void setShooterVelocity(double targetRpm) {
            double ticksPerSecond = rpmToTicksPerSecond(targetRpm);
            leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftShooter.setVelocity(ticksPerSecond);
            rightShooter.setVelocity(ticksPerSecond);
        }

        private void stopShooter() {
            leftShooter.setPower(0.0);
            rightShooter.setPower(0.0);
        }

        private boolean waitForShooterReady(double targetRpm, double timeoutSec) {
            ElapsedTime timer = new ElapsedTime();
            timer.reset();
            while (opMode.opModeIsActive()) {
                if (isShooterReady(targetRpm)) {
                    return true;
                }
                if (timeoutSec > 0 && timer.seconds() >= timeoutSec) {
                    return false;
                }
                opMode.idle();
                opMode.sleep(10);
            }
            return false;
        }

        private boolean isShooterReady(double targetRpm) {
            double leftRpm = ticksPerSecondToRpm(leftShooter.getVelocity());
            double rightRpm = ticksPerSecondToRpm(rightShooter.getVelocity());

            boolean leftGreen = Math.abs(targetRpm - leftRpm) <= RobotConstants.SHOOTER_RPM_GREEN_THRESHOLD;
            boolean rightGreen = Math.abs(targetRpm - rightRpm) <= RobotConstants.SHOOTER_RPM_GREEN_THRESHOLD;

            boolean leftYellow = Math.abs(targetRpm - leftRpm) <= RobotConstants.SHOOTER_RPM_YELLOW_THRESHOLD;
            boolean rightYellow = Math.abs(targetRpm - rightRpm) <= RobotConstants.SHOOTER_RPM_YELLOW_THRESHOLD;

            return (leftGreen && rightGreen)
                    || (leftGreen && rightYellow)
                    || (rightGreen && leftYellow);
        }

        private void waitSeconds(double seconds) {
            if (seconds <= 0) {
                return;
            }
            ElapsedTime timer = new ElapsedTime();
            timer.reset();
            while (opMode.opModeIsActive() && timer.seconds() < seconds) {
                opMode.idle();
                opMode.sleep(10);
            }
        }

        private double rpmToTicksPerSecond(double rpm) {
            return (rpm * RobotConstants.SHOOTER_TICKS_PER_REV) / 60.0;
        }

        private double ticksPerSecondToRpm(double ticksPerSecond) {
            return (ticksPerSecond * 60.0) / RobotConstants.SHOOTER_TICKS_PER_REV;
        }

        private void runMecanum(int forwardTicks, int strafeTicks, double power) {
            int fl = forwardTicks + strafeTicks;
            int fr = forwardTicks - strafeTicks;
            int bl = forwardTicks - strafeTicks;
            int br = forwardTicks + strafeTicks;
            runToPosition(fl, fr, bl, br, power);
        }

        private void runToPosition(int flDelta, int frDelta, int blDelta, int brDelta, double power) {
            if (flDelta == 0 && frDelta == 0 && blDelta == 0 && brDelta == 0) {
                return;
            }

            double clippedPower = Range.clip(Math.abs(power), 0.0, 1.0);

            int flTarget = frontLeft.getCurrentPosition() + flDelta;
            int frTarget = frontRight.getCurrentPosition() + frDelta;
            int blTarget = backLeft.getCurrentPosition() + blDelta;
            int brTarget = backRight.getCurrentPosition() + brDelta;

            frontLeft.setTargetPosition(flTarget);
            frontRight.setTargetPosition(frTarget);
            backLeft.setTargetPosition(blTarget);
            backRight.setTargetPosition(brTarget);

            setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
            setMotorPower(clippedPower);

            while (opMode.opModeIsActive()
                    && (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {
                opMode.idle();
            }

            setMotorPower(0.0);
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
            applyBrake();
        }

        private void setRunMode(DcMotor.RunMode mode) {
            frontLeft.setMode(mode);
            frontRight.setMode(mode);
            backLeft.setMode(mode);
            backRight.setMode(mode);
        }

        private void setMotorPower(double power) {
            frontLeft.setPower(power);
            frontRight.setPower(power);
            backLeft.setPower(power);
            backRight.setPower(power);
        }

        private void applyBrake() {
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        private int inchesToTicks(double inches) {
            return (int) Math.round(inches * RobotConstants.getDriveCountsPerInch());
        }

        private int degreesToTicks(double degrees) {
            return (int) Math.round(degrees * RobotConstants.getDriveCountsPerDegree());
        }
    }

    public static class ShootConfig {
        private ShooterMode mode = ShooterMode.RPM;
        private double targetRpm = RobotConstants.SHOOTER_TARGET_RPM;
        private int shots = 1;
        private boolean dropBlockerOnStart = true;
        private double intakeAdvancePower = RobotConstants.AUTO_INTAKE_POWER;
        private double frontIntakeAdvancePower = RobotConstants.AUTO_FRONT_INTAKE_POWER;
        private double intakeAdvanceSeconds = RobotConstants.AUTO_INTAKE_ADVANCE_SECONDS;
        private double readyTimeoutSec = 3.0;

        public static ShootConfig rpmShots(int shots, double targetRpm) {
            ShootConfig config = new ShootConfig();
            config.mode = ShooterMode.RPM;
            config.shots = Math.max(1, shots);
            config.targetRpm = targetRpm;
            return config;
        }

        public static ShootConfig highSpeedShots(int shots) {
            ShootConfig config = new ShootConfig();
            config.mode = ShooterMode.HIGH_SPEED;
            config.shots = Math.max(1, shots);
            return config;
        }

        public ShootConfig withDropBlockerOnStart(boolean drop) {
            this.dropBlockerOnStart = drop;
            return this;
        }

        public ShootConfig withIntakeAdvance(double mainPower, double frontPower, double seconds) {
            this.intakeAdvancePower = mainPower;
            this.frontIntakeAdvancePower = frontPower;
            this.intakeAdvanceSeconds = Math.max(0.0, seconds);
            return this;
        }

        public ShootConfig withReadyTimeout(double seconds) {
            this.readyTimeoutSec = Math.max(0.0, seconds);
            return this;
        }

        private ShootConfig() {
        }
    }

    public enum ShooterMode {
        RPM,
        HIGH_SPEED
    }
}
