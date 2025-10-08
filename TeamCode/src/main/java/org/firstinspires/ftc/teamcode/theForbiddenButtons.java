package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class theForbiddenButtons {
    private robotHardware RobotHardware;
    final private DcMotor leftOuttakeMotor;
    final private DcMotor rightOuttakeMotor;
    final private DcMotor intakeMotor;
    final private DcMotor frontIntakeMotor;
    public theForbiddenButtons(robotHardware RobotHardware) {
        this.RobotHardware = RobotHardware;
        rightOuttakeMotor = RobotHardware.rightOuttakeMotor;
        leftOuttakeMotor = RobotHardware.leftOuttakeMotor;
        intakeMotor = RobotHardware.intakeMotor;
        frontIntakeMotor = RobotHardware.frontIntakeMotor;
    }
    public void chaoticInputs(Gamepad gamepad1){

// Find if A is pressed on the gamepad. If so, spin each of the outtake motors at max speed

        boolean spinOuttakeMotors = gamepad1.a;
        double outtakeMotorSpeed;



//get motor speed from gamepad boolean
        if (spinOuttakeMotors) {
            outtakeMotorSpeed = 1;
        }
        else {
            outtakeMotorSpeed = 0;
        }

        rightOuttakeMotor.setPower(outtakeMotorSpeed);
        leftOuttakeMotor.setPower(outtakeMotorSpeed);







// Find if B is pressed on the gamepad. If so, spin the intake motor at max speed
        boolean spinIntakeMotor = gamepad1.b;
        double intakeMotorSpeed;

//get motor speed from gamepad boolean
        if (spinIntakeMotor) {
            intakeMotorSpeed = -1;
        }
        else {
            intakeMotorSpeed = 0;
        }


        intakeMotor.setPower(intakeMotorSpeed) ;
        frontIntakeMotor.setPower(intakeMotorSpeed);
    }
}






