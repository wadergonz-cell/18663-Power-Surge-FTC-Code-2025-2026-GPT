package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Red Alliance TeleOp with shooting sequence and auto-aim.
 * - Drive: field-centric with left stick
 * - A/Y/X buttons: Shooting sequence (uses AprilTag distance for RPM)
 * - B button: Toggle intake
 * - Left bumper: Optional auto-aim towards Tag ID 19
 */
@TeleOp(name = "zoomies - red alliance", group = "TeleOp")
public class ZoomiesMode_RedAlliance extends LinearOpMode {

    private robotHardware RobotHardware;
    private driveTrainChooChoo driveTrainChooChoo;
    private theForbiddenButtons theForbiddenButtons;
    private ShootingSequenceController shootingSequence;
    private AprilTagHelper vision;
    private AprilTagAim aimPID;

    // Red alliance Tag ID
    private static final int TARGET_ID = 19;

    // PID gains (tune on-field)
    private static final double KP = 4.0;
    private static final double KI = 0.10;
    private static final double KD = 4.0;

    private static final double OUTPUT_MAX = 0.6;
    private static final double DEADBAND_DEG = 2.0;

    @Override
    public void runOpMode() {
        RobotHardware = new robotHardware(this);
        driveTrainChooChoo = new driveTrainChooChoo(RobotHardware);
        theForbiddenButtons = new theForbiddenButtons(RobotHardware);
        shootingSequence = new ShootingSequenceController(RobotHardware);

        vision = new AprilTagHelper(hardwareMap);
        vision.init();

        aimPID = new AprilTagAim(KP, KI, KD)
                .setOutputMax(OUTPUT_MAX)
                .setIntegralMax(0.4)
                .setDeadbandRad(Math.toRadians(DEADBAND_DEG))
                .setInvert(true);

        telemetry.addLine("Red Auto-Aim TeleOp Ready");
        telemetry.addLine("A: Shoot sequence  |  B: Intake toggle  |  Y: Add shot  |  X: Cancel");
        telemetry.addLine("LB: Auto-aim (Tag ID 19)");
        telemetry.update();

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

            // Handle auto-aim (left bumper)
            Double turnOverride = null;
            boolean aimRequested = gamepad1.left_bumper;
            if (aimRequested && tagReport.hasTag && (tagReport.tagId == TARGET_ID)) {
                double dt = 0.016; // Approximate 60 Hz
                double turnCmd = aimPID.update(tagReport.yawCorrectedRad, dt);
                turnOverride = turnCmd;
            } else {
                aimPID.reset();
            }
            driveTrainChooChoo.setTurnOverride(turnOverride);

            // Drive
            driveTrainChooChoo.driveCode(gamepad1);
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
            telemetry.addData("Auto-Aim", aimRequested);

            telemetry.addLine("\n=== SHOOTING ===");
            telemetry.addData("A Clicks", shootingSequence.getAClickCount());
            telemetry.addData("Y Count (shots)", shootingSequence.getYClickCount());
            telemetry.addData("State", shootingSequence.getShootingState());
            telemetry.addData("Shots Fired", shootingSequence.getShotsFired());

            telemetry.addLine("\n=== VISION (Red / ID 19) ===");
            telemetry.addData("LB Auto-Aim", aimRequested);
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