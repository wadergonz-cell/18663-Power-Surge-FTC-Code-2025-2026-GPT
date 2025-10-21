package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Blue Alliance TeleOp with shooting sequence and auto-aim.
 * - Drive: field-centric with left stick
 * - A/Y/X buttons: Shooting sequence (uses AprilTag distance for RPM)
 * - B button: Toggle intake
 * - Left bumper: Optional auto-aim towards Tag ID 20
 */
@TeleOp(name = "zoomies - blue alliance", group = "TeleOp")
public class ZoomiesMode_BlueAlliance extends LinearOpMode {

    private robotHardware RobotHardware;
    private driveTrainChooChoo driveTrainChooChoo;
    private theForbiddenButtons theForbiddenButtons;
    private ShootingSequenceController shootingSequence;
    private AprilTagHelper vision;
    private AprilTagAim aimPID;

    private final ElapsedTime aimTimer = new ElapsedTime();

    private boolean autoAimEnabled = false;
    private boolean leftBumperLatch = false;
    private boolean autoAimTracking = false;
    private double aimTurnCommand = 0.0;

    // Blue alliance Tag ID
    private static final int TARGET_ID = 20;

    // PID gains (tune on-field)
    private static final double KP = 4.0;
    private static final double KI = 0.10;
    private static final double KD = 4.0;

    private static final double OUTPUT_MAX = 0.6;
    private static final double DEADBAND_DEG = 2.0;
    private static final double AIM_FILTER_ALPHA = 0.45; // higher = smoother, lower = quicker
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

        aimPID = new AprilTagAim(KP, KI, KD)
                .setOutputMax(OUTPUT_MAX)
                .setIntegralMax(0.4)
                .setDeadbandRad(Math.toRadians(DEADBAND_DEG))
                .setInvert(true);

        telemetry.addLine("Blue Auto-Aim TeleOp Ready");
        telemetry.addLine("A: Shoot sequence  |  B: Intake toggle  |  Y: Add shot  |  X: Cancel");
        telemetry.addLine("LB: Auto-aim (Tag ID 20)");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            vision.stop();
            return;
        }

        aimTimer.reset();
        aimTurnCommand = 0.0;
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
                aimPID.reset();
                aimTurnCommand = 0.0;
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
                    double turnCmd = aimPID.update(tagReport.yawCorrectedRad, dt);
                    aimTurnCommand = Range.clip(
                            aimTurnCommand * AIM_FILTER_ALPHA + turnCmd * (1.0 - AIM_FILTER_ALPHA),
                            -OUTPUT_MAX,
                            OUTPUT_MAX);
                    turnAssist = aimTurnCommand;
                    autoAimTracking = true;
                } else {
                    aimPID.reset();
                    aimTurnCommand = 0.0;
                }
            } else {
                if (!autoAimEnabled || driverOverride) {
                    aimPID.reset();
                    aimTurnCommand = 0.0;
                }
            }
            driveTrainChooChoo.setTurnAssist(turnAssist);

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
            telemetry.addData("Auto-Aim Enabled", autoAimEnabled);
            telemetry.addData("Auto-Aim Tracking", autoAimTracking);
            telemetry.addData("Auto-Aim Turn", "%.2f", aimTurnCommand);

            telemetry.addLine("\n=== SHOOTING ===");
            telemetry.addData("A Clicks", shootingSequence.getAClickCount());
            telemetry.addData("Y Count (shots)", shootingSequence.getYClickCount());
            telemetry.addData("State", shootingSequence.getShootingState());
            telemetry.addData("Shots Fired", shootingSequence.getShotsFired());

            telemetry.addLine("\n=== VISION (Blue / ID 20) ===");
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