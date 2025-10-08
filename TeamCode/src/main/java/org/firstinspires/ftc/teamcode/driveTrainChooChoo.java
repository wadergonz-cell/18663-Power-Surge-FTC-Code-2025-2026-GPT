package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class driveTrainChooChoo {
    private robotHardware RobotHardware;

    final private DcMotor frontLeftMotor;
    final private DcMotor frontRightMotor;
    final private DcMotor backRightMotor;
    final private DcMotor backLeftMotor;
    final private IMU imu;


    public driveTrainChooChoo(robotHardware RobotHardware){
        this.RobotHardware = RobotHardware;

        frontLeftMotor = RobotHardware.frontLeftMotor;
        frontRightMotor = RobotHardware.frontRightMotor;
        backRightMotor = RobotHardware.backRightMotor;
        backLeftMotor = RobotHardware.backLeftMotor;
        imu = RobotHardware.imu;
    }

    public void driveCode(Gamepad gamepad1){
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;


        // These track the amount the triggers are pressed, contributing to a slow and more precise turn
        double slowLeft = (-0.20) * gamepad1.left_trigger;
        double slowRight = 0.20 * gamepad1.right_trigger;
        double slowTurn = slowRight + slowLeft;

        // Resets the heading of the field centric drive.
        //Not using options as this button didnt work on our controller


        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        /// removed negatives from (botHeading)
        double rotX = x * Math.cos(botHeading) + y * Math.sin(-botHeading);
        double rotY = x * Math.sin(botHeading) - y * Math.cos(-botHeading);

        // Create seperate variables for the trigger turning of the robot


        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        double frontLeftPower = ((rotY + rotX + rx) / denominator) +  slowTurn;
        double backLeftPower = ((rotY - rotX + rx) / denominator) + slowTurn;
        double frontRightPower = ((rotY - rotX - rx) / denominator) - slowTurn;
        double backRightPower = ((rotY + rotX - rx) / denominator) - slowTurn;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        if (gamepad1.back) {
            RobotHardware.imu.resetYaw();
        }
    }
}
