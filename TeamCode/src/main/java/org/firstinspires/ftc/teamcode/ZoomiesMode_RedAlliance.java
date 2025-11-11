// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ZoomiesMode_RedAlliance.java
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Red Alliance TeleOp with shooting sequence and auto-aim.
 * - Drive: field-centric with left stick
 * - A/Y/X buttons: Shooting sequence (uses AprilTag distance for RPM)
 * - B button: Toggle intake
 * - Left bumper: Optional auto-aim towards Tag ID 24
 */
// Sync marker: updated on 2025-10-21 22:57 UTC
@TeleOp(name = "zoomies - red alliance", group = "TeleOp")
public class ZoomiesMode_RedAlliance extends LinearOpMode {

    private robotHardware RobotHardware;
    private driveTrainChooChoo driveTrainChooChoo;
    private theForbiddenButtons theForbiddenButtons;
    private ShootingSequenceController shootingSequence;
    private AprilTagHelper vision;
    private final ElapsedTime aimTimer = new ElapsedTime();

    private boolean autoAimEnabled = false;
    private boolean leftBumperLatch = false;
    private boolean autoAimTracking = false;
    private double filteredAimErrorRad = 0.0;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;

    // Red alliance Tag ID
    private static final int TARGET_ID = 24;

    private static final double AIM_FILTER_ALPHA = 0.45;
    private static final double AIM_MIN_DT = 0.005;
    private static final double AIM_MAX_DT = 0.08;
    private static final double AIM_DRIVER_OVERRIDE = 0.18;

    @Override
    public void runOpMode() {
        RobotHardware = new robotHardware(this);
        driveTrainChooChoo = new driveTrainChooChoo(RobotHardware);
        theForbiddenButtons = new theForbiddenButtons(RobotHardware);
        shootingSequence = new ShootingSequenceController(RobotHardware);

        vision = new AprilTagHelper(hardwareMap);
        vision.init();

        telemetry.addLine("Red Auto-Aim TeleOp Ready");
        telemetry.addLine("A: Shoot sequence  |  B: Intake toggle  |  Y: Add shot  |  X: Cancel");
        telemetry.addLine("LB: Auto-aim (Tag ID 19)");
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

        aimTimer.reset();
        filteredAimErrorRad = 0.0;
        autoAimEnabled = false;
        leftBumperLatch = false;
        autoAimTracking = false;

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

            // Handle auto-aim (left bumper toggle)
            boolean leftBumper = gamepad1.left_bumper;
            if (leftBumper && !leftBumperLatch) {
                autoAimEnabled = !autoAimEnabled;
                filteredAimErrorRad = 0.0;
                aimTimer.reset();
            }
            leftBumperLatch = leftBumper;

            boolean dpadUp = gamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down;
            if (dpadUp && !prevDpadUp) {
                shootingSequence.setLongRangeMode(true);
            }
            if (dpadDown && !prevDpadDown) {
                shootingSequence.setLongRangeMode(false);
            }
            prevDpadUp = dpadUp;
            prevDpadDown = dpadDown;

            Double turnAssist = null;
            autoAimTracking = false;
            double driverTurnInput = -gamepad1.right_stick_x;
            boolean driverOverride = Math.abs(driverTurnInput) > AIM_DRIVER_OVERRIDE;
            if (autoAimEnabled && !driverOverride) {
                double dt = Range.clip(aimTimer.seconds(), AIM_MIN_DT, AIM_MAX_DT);
                aimTimer.reset();

                if (tagReport.hasTag && (tagReport.tagId == TARGET_ID)) {
                    double rawError = tagReport.yawCorrectedRad;
                    filteredAimErrorRad = filteredAimErrorRad * AIM_FILTER_ALPHA
                            + rawError * (1.0 - AIM_FILTER_ALPHA);
                    turnAssist = filteredAimErrorRad;
                    autoAimTracking = true;
                } else {
                    filteredAimErrorRad = 0.0;
                }
            } else {
                if (!autoAimEnabled || driverOverride) {
                    filteredAimErrorRad = 0.0;
                }
            }
            driveTrainChooChoo.setTurnAssist(turnAssist);
            autoAimTracking = autoAimTracking && driveTrainChooChoo.isTurnAssistActive();

            // Drive
            driveTrainChooChoo.driveFieldCentric(gamepad1);
            theForbiddenButtons.driveOnlyInputs(gamepad1);

            // Shooting sequence controller (uses AprilTag distance)
            shootingSequence.update(
                    gamepad1.a,
                    gamepad1.b,
                    gamepad1.y,
                    gamepad1.x,
                    tagReport
            );

            telemetry.addLine(String.format("Camera tracking 📸 - %s %s",
                    boolEmoji(autoAimEnabled),
                    boolEmoji(tagReport.hasTag)));
            telemetry.addLine(startZoneStatusLine());

            String spinLine = String.format("Spin-up state 😵‍💫 - L%s R%s",
                    rpmEmoji(shootingSequence.getShooterTargetRpm(), shootingSequence.getLeftShooterRpm()),
                    rpmEmoji(shootingSequence.getShooterTargetRpm(), shootingSequence.getRightShooterRpm()));
            if (shootingSequence.isLongRangeModeActive()) {
                spinLine += " LR🟣";
            }
            telemetry.addLine(spinLine);

            telemetry.addLine(yCountLine(shootingSequence.getYClickCount()));

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
        if (error <= RobotConstants.SHOOTER_RPM_GREEN_THRESHOLD) {
            return "🟢";
        }
        if (error <= RobotConstants.SHOOTER_RPM_YELLOW_THRESHOLD) {
            return "🟡";
        }
        return "🔴";
    }

    private String yCountLine(int yCount) {
        if (yCount <= 0) {
            return "Y count ⚪ - none";
        }
        StringBuilder builder = new StringBuilder("Y count ⚪ - ");
        for (int i = 0; i < yCount; i++) {
            if (i > 0) {
                builder.append(' ');
            }
            builder.append("⚪");
        }
        return builder.toString();
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
