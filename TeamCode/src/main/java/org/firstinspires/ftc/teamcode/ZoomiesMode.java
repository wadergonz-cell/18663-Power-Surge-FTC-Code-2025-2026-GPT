package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ZoomiesMode extends LinearOpMode {
    robotHardware RobotHardware;
    driveTrainChooChoo driveTrainChooChoo;
    theForbiddenButtons theForbiddenButtons;



   // @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware = new robotHardware(this);
        driveTrainChooChoo = new driveTrainChooChoo(RobotHardware);
        theForbiddenButtons = new theForbiddenButtons(RobotHardware);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            driveTrainChooChoo.driveCode(gamepad1);
            theForbiddenButtons.chaoticInputs(gamepad1);
        }
    }
}