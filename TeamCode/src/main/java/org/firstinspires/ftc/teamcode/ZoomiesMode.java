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

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware = new robotHardware(this);
        driveTrainChooChoo = new driveTrainChooChoo(RobotHardware);
        theForbiddenButtons = new theForbiddenButtons(RobotHardware);
        shootingSequence = new ShootingSequenceController(RobotHardware);

        vision = new AprilTagHelper(hardwareMap);
        vision.init();

        telemetry.addLine("ZoomiesMode Ready");
        telemetry.addLine("A: Shoot sequence  |  Y: Add shot  |  X: Cancel");
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

            // Drive
            driveTrainChooChoo.driveCode(gamepad1);
            theForbiddenButtons.driveOnlyInputs(gamepad1);

            // Shooting sequence controller
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

            telemetry.addLine("\n=== SHOOTING ===");
            telemetry.addData("A Clicks", shootingSequence.getAClickCount());
            telemetry.addData("Y Count (shots)", shootingSequence.getYClickCount());
            telemetry.addData("State", shootingSequence.getShootingState());
            telemetry.addData("Shots Fired", shootingSequence.getShotsFired());
            ShootingSequenceController.ShooterPidTelemetry leftPid = shootingSequence.getLeftShooterPidTelemetry();
            ShootingSequenceController.ShooterPidTelemetry rightPid = shootingSequence.getRightShooterPidTelemetry();
            telemetry.addData("Shooter PID Enabled", shootingSequence.isShooterPidEnabled());
            telemetry.addData("Shooter Target RPM", "%.1f", shootingSequence.getTargetShooterRpm());
            telemetry.addData("Shooter RPM L/R", "%.1f / %.1f",
                    shootingSequence.getLeftShooterRpm(),
                    shootingSequence.getRightShooterRpm());
            telemetry.addData("Shooter Motor Power L/R", "%.2f / %.2f",
                    shootingSequence.getLeftShooterPowerCommand(),
                    shootingSequence.getRightShooterPowerCommand());
            telemetry.addData("PID L", "err=%.1f | P=%.3f I=%.3f D=%.3f | out=%.2f",
                    leftPid.errorRpm,
                    leftPid.pTerm,
                    leftPid.iTerm,
                    leftPid.dTerm,
                    leftPid.output);
            telemetry.addData("PID R", "err=%.1f | P=%.3f I=%.3f D=%.3f | out=%.2f",
                    rightPid.errorRpm,
                    rightPid.pTerm,
                    rightPid.iTerm,
                    rightPid.dTerm,
                    rightPid.output);

            telemetry.addLine("\n=== VISION ===");
            telemetry.addData("Has Tag", tagReport.hasTag);
            telemetry.addData("Tag ID", tagReport.tagId);
            telemetry.addData("Distance (in)", "%.1f", tagReport.horizontalRangeIn);

            telemetry.update();
        }

        vision.stop();
    }
}
