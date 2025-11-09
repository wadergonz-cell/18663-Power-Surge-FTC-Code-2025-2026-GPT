// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ZoomiesMode.java
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Basic TeleOp with shooting sequence state machine.
 * - Drive: robot-centric with left stick
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
            driveTrainChooChoo.driveRobotCentric(gamepad1);
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
            driveTrainChooChoo.FieldCentricDebug debug = driveTrainChooChoo.getFieldCentricDebug();
            telemetry.addData("Drive Mode", "Robot-Centric");
            telemetry.addData("Raw XY/Turn", "%.2f %.2f | %.2f", debug.rawX, debug.rawY, debug.rawTurn);
            telemetry.addData("Rot XY", "%.2f %.2f", debug.rotatedX, debug.rotatedY);
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

            telemetry.addLine("\n=== VISION ===");
            telemetry.addData("Has Tag", tagReport.hasTag);
            telemetry.addData("Tag ID", tagReport.tagId);
            telemetry.addData("Distance (in)", "%.1f", tagReport.horizontalRangeIn);

            telemetry.update();
        }

        vision.stop();
    }
}
