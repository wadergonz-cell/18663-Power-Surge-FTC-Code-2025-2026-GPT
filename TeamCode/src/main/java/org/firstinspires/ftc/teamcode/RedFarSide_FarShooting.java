package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "🔴RedAllianceCloseSide⚫", group = "Auto")
public class RedFarSide_FarShooting extends LinearOpMode {

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

        drive.shoot(BlueAllianceFarSide.ShootConfig.rpmShots(3, 6600));
        drive.pause(18);
        drive.driveForward(-5, 1);
        drive.turnDegrees(145, 0.7);

        drive.stopIntakeContinuous();
        drive.stopAll();
        drive.stopMechanisms();
        vision.stop();
    }
}
