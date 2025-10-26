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

            // Telemetry
            telemetry.addLine("=== DRIVE ===");
            telemetry.addData("IMU Heading", "%.1f°",
                    RobotHardware.imu.getRobotYawPitchRollAngles().getYaw(
                            org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES));
            telemetry.addData("Auto-Aim Enabled", autoAimEnabled);
            telemetry.addData("Auto-Aim Tracking", autoAimTracking);
            telemetry.addData("Auto-Aim Error (deg)", "%.2f", Math.toDegrees(filteredAimErrorRad));
            driveTrainChooChoo.TurnAssistPidTelemetry turnPid = driveTrainChooChoo.getTurnAssistTelemetry();
            telemetry.addData("Auto-Aim Turn PID",
                    "out=%.2f | raw=%.2f° err=%.2f° | P=%.3f I=%.3f D=%.3f | I=%.3f",
                    turnPid.output,
                    Math.toDegrees(turnPid.invertedErrorRad),
                    Math.toDegrees(turnPid.pidErrorRad),
                    turnPid.pTerm,
                    turnPid.iTerm,
                    turnPid.dTerm,
                    turnPid.integralState);
            driveTrainChooChoo.FieldCentricDebug debug = driveTrainChooChoo.getFieldCentricDebug();
            telemetry.addData("Field Centric", debug.fieldCentricEnabled);
            telemetry.addData("Raw XY/Turn", "%.2f %.2f | %.2f", debug.rawX, debug.rawY, debug.rawTurn);
            telemetry.addData("Rot XY", "%.2f %.2f", debug.rotatedX, debug.rotatedY);
            telemetry.addData("Heading", "%.1f°", debug.headingDeg);
            telemetry.addData("Turn Assist", "%s | out=%.2f | applied=%.2f | slow=%.2f",
                    debug.turnAssistActive ? "ON" : "OFF",
                    debug.turnAssistOutput,
                    debug.appliedTurn,
                    debug.slowTurn);
            telemetry.addData("Motor Power", "FL %.2f | FR %.2f | BL %.2f | BR %.2f",
                    debug.frontLeftPower,
                    debug.frontRightPower,
                    debug.backLeftPower,
                    debug.backRightPower);

            telemetry.addLine("\n=== SHOOTING ===");
            telemetry.addData("A Clicks", shootingSequence.getAClickCount());
            telemetry.addData("Y Count (shots)", shootingSequence.getYClickCount());
            telemetry.addData("State", shootingSequence.getShootingState());
            telemetry.addData("Shots Fired", shootingSequence.getShotsFired());
            telemetry.addData("Shooter Spinning", shootingSequence.isShooterSpinning());
            telemetry.addData("Shooter Target Power", "%.2f", ShootingSequenceController.SHOOTER_SPIN_POWER);
            telemetry.addData("Commanded Power", "%.2f", shootingSequence.getShooterPowerCommand());
            telemetry.addData("Motor Power L/R", "%.2f / %.2f",
                    RobotHardware.leftOuttakeMotor.getPower(),
                    RobotHardware.rightOuttakeMotor.getPower());

            telemetry.addLine("\n=== VISION (Red / ID 19) ===");
            telemetry.addData("LB Auto-Aim Enabled", autoAimEnabled);
            telemetry.addData("LB Tracking", autoAimTracking);
            telemetry.addData("Has Tag", tagReport.hasTag);
            telemetry.addData("Tag ID", tagReport.tagId);
            telemetry.addData("Distance (in)", "%.1f", tagReport.horizontalRangeIn);
            telemetry.addData("Forward (in)", "%.1f", tagReport.forwardIn);
            telemetry.addData("Lateral (in)", "%.1f", tagReport.lateralIn);
            telemetry.addData("Yaw Corrected (deg)", "%.1f", Math.toDegrees(tagReport.yawCorrectedRad));

            telemetry.update();
        }

        vision.stop();
    }
}
