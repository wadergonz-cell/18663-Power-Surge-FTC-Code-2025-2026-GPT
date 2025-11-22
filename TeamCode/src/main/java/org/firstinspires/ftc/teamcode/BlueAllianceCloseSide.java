package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "🔵BlueAllianceCloseSide🟢", group = "Auto")
public class BlueAllianceCloseSide extends LinearOpMode {

    private robotHardware hardware;
    private BlueAllianceFarSide.DriveBlocks drive;
    private AprilTagHelper vision;

    @Override
    public void runOpMode() {
        hardware = new robotHardware(this);
        drive = new BlueAllianceFarSide.DriveBlocks(this, hardware);
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

        drive.blockerDown();

        drive.driveForward(50.0, 1);
        drive.shoot(BlueAllianceFarSide.ShootConfig.rpmShots(3, 2900));

        drive.driveForward(7, 1);
        drive.turnDegrees(60, 1);
        drive.startIntakeContinuous(0.8, -0.8);
        drive.driveForward(-27, 1);
        drive.blockerDown();
        drive.driveForward(-15, 1);
        drive.stopIntakeContinuous();
        drive.driveForward(38, 1);
        drive.turnDegrees(-72, 0.8);
        drive.driveForward(-8, 1);
        drive.shoot(BlueAllianceFarSide.ShootConfig.rpmShots(2, 2900));
        drive.turnDegrees(-60, 1);
        drive.driveForward(50, 1);
        drive.turnDegrees(-125, 1);
        drive.pause(5);



        drive.stopIntakeContinuous();
        drive.stopAll();
        drive.stopMechanisms();
        vision.stop();
    }
}
