// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ZoomiesMode.java
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Basic TeleOp with shooting sequence state machine.
 * - Drive: field-centric with left stick
 * - A/Y/X buttons: Shooting sequence (no vision auto-aim)
 */
@TeleOp(name = "ZoomiesMode", group = "TeleOp")
public class ZoomiesMode extends LinearOpMode {

    private robotHardware RobotHardware;
    private driveTrainChooChoo driveTrainChooChoo;
    private theForbiddenButtons theForbiddenButtons;
    private ShootingSequenceController shootingSequence;
    private AprilTagHelper vision;
    private boolean cameraTrackingEnabled = false;
    private boolean leftBumperLatch = false;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean fieldCentricEnabled = true;
    private boolean prevY = false;
    private boolean shooterHighSpeedMode = false;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware = new robotHardware(this);
        driveTrainChooChoo = new driveTrainChooChoo(RobotHardware);
        theForbiddenButtons = new theForbiddenButtons(RobotHardware);
        shootingSequence = new ShootingSequenceController(RobotHardware);
        shootingSequence.setShooterTargetRpm(ShootingSequenceController.SHOOTER_LOW_SPEED_RPM);

        vision = new AprilTagHelper(hardwareMap);
        vision.init();

        telemetry.addLine("ZoomiesMode Ready");
        telemetry.addLine("A: Shoot sequence  |  Y: Drive mode  |  X: Cancel");
        telemetry.addLine("D-pad: Shooter speed (⬆ high / ⬇ low)");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            vision.update();
            Integer startTag = AprilTagHelper.getStartZoneTagId();
            telemetry.addLine(startTag != null
                    ? "Start tag 🎯 - " + startTag
                    : "Start tag 🎯 - scanning...");
            telemetry.update();
            sleep(50);
            telemetry.clearAll();
        }

        waitForStart();
        if (isStopRequested()) {
            vision.stop();
            return;
        }

        while (opModeIsActive()) {
            // Update vision
            vision.update();

            // Compute AprilTag report
            AprilTagCalibration.Report tagReport = AprilTagCalibration.computeReport(
                    vision.hasTarget(),
                    vision.getTargetId(),
                    vision.getX_m(),
                    vision.getY_m(),
                    vision.getZ_m(),
                    vision.getYawErrorRad()
            );

            boolean leftBumper = gamepad1.left_bumper;
            if (leftBumper && !leftBumperLatch) {
                cameraTrackingEnabled = !cameraTrackingEnabled;
            }
            leftBumperLatch = leftBumper;

            boolean dpadUp = gamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down;
            if (dpadUp && !prevDpadUp) {
                shooterHighSpeedMode = true;
                shootingSequence.setShooterTargetRpm(ShootingSequenceController.SHOOTER_HIGH_SPEED_RPM);
            }
            if (dpadDown && !prevDpadDown) {
                shooterHighSpeedMode = false;
                shootingSequence.setShooterTargetRpm(ShootingSequenceController.SHOOTER_LOW_SPEED_RPM);
            }
            prevDpadUp = dpadUp;
            prevDpadDown = dpadDown;

            boolean yPressed = gamepad1.y;
            if (yPressed && !prevY) {
                fieldCentricEnabled = !fieldCentricEnabled;
            }
            prevY = yPressed;

            Double turnAssist = null;
            if (cameraTrackingEnabled && tagReport.hasTag) {
                turnAssist = tagReport.yawCorrectedRad;
            }
            driveTrainChooChoo.setTurnAssist(turnAssist);

            // Drive
            driveTrainChooChoo.drive(gamepad1, fieldCentricEnabled);
            theForbiddenButtons.driveOnlyInputs(gamepad1);

            // Shooting sequence controller
            shootingSequence.update(
                    gamepad1.a,
                    gamepad1.b,
                    gamepad1.x,
                    tagReport
            );

            telemetry.addLine(String.format("Camera tracking 📸 - %s %s",
                    boolEmoji(cameraTrackingEnabled),
                    boolEmoji(tagReport.hasTag)));
            telemetry.addLine(startZoneStatusLine());

            telemetry.addLine(String.format("Drive mode 🚗 - %s",
                    fieldCentricEnabled ? "Field-centric" : "Robot-centric"));
            telemetry.addLine(String.format("Shooter mode 🎯 - %s (target %.0f RPM)",
                    shooterHighSpeedMode ? "High speed" : "Low speed",
                    shootingSequence.getShooterTargetRpm()));

            String spinLine = String.format("Spin-up state 😵‍💫 - L%s R%s",
                    rpmEmoji(shootingSequence.getShooterTargetRpm(), shootingSequence.getLeftShooterRpm()),
                    rpmEmoji(shootingSequence.getShooterTargetRpm(), shootingSequence.getRightShooterRpm()));
            telemetry.addLine(spinLine);

            telemetry.addLine(String.format("Shooter RPM 📈 - L %.1f | R %.1f",
                    shootingSequence.getLeftShooterRpm(),
                    shootingSequence.getRightShooterRpm()));

            Integer startZoneTag = AprilTagHelper.getStartZoneTagId();
            telemetry.addLine(startZoneTag != null
                    ? "Start tag 🎯 - " + startZoneTag
                    : "Start tag 🎯 - scanning...");

            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addLine("");

            if (tagReport.hasTag) {
                telemetry.addLine(String.format("April tag distance ↔️ - %d inches",
                        Math.round(tagReport.horizontalRangeIn)));
            } else {
                telemetry.addLine("April tag distance ↔️ - 🔴 (not found)");
            }

            telemetry.addLine(String.format("IMU ⬆ %.1f degrees", driveTrainChooChoo.getHeadingDegrees()));

            telemetry.update();
        }

        vision.stop();
    }

    private String boolEmoji(boolean condition) {
        return condition ? "🟢" : "🔴";
    }

    private String rpmEmoji(double targetRpm, double currentRpm) {
        if (targetRpm <= 0.0) {
            return "🔴";
        }

        double error = Math.abs(targetRpm - currentRpm);
        if (error <= ShootingSequenceController.SHOOTER_RPM_GREEN_THRESHOLD) {
            return "🟢";
        }
        if (error <= ShootingSequenceController.SHOOTER_RPM_YELLOW_THRESHOLD) {
            return "🟡";
        }
        return "🔴";
    }

    private String startZoneStatusLine() {
        Integer startTag = AprilTagHelper.getStartZoneTagId();
        if (startTag == null) {
            return "⚫ ⚫ ⚫";
        }
        switch (startTag) {
            case 21:
                return "🟢 🟣 🟣";
            case 22:
                return "🟣 🟢 🟣";
            case 23:
                return "🟣 🟣 🟢";
            default:
                return "🟣 🟣 🟢";
        }
    }
}
