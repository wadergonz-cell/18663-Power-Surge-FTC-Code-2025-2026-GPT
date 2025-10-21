package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Mecanum drivetrain with field-centric drive (using IMU)
 * and optional turn override (for auto-aim).
 */
public class driveTrainChooChoo {

    private final robotHardware RobotHardware;

    // Motors
    private final DcMotor frontLeftMotor;
    private final DcMotor frontRightMotor;
    private final DcMotor backRightMotor;
    private final DcMotor backLeftMotor;

    // IMU
    private final IMU imu;

    // Optional rotation override
    private Double turnOverride = null;

    public driveTrainChooChoo(robotHardware RobotHardware) {
        this.RobotHardware = RobotHardware;

        frontLeftMotor  = RobotHardware.frontLeftMotor;
        frontRightMotor = RobotHardware.frontRightMotor;
        backRightMotor  = RobotHardware.backRightMotor;
        backLeftMotor   = RobotHardware.backLeftMotor;
        imu             = RobotHardware.imu;
    }

    /** Allow external code to override turn (e.g., AprilTag PID). */
    public void setTurnOverride(Double override) {
        this.turnOverride = override;
    }

    /** Call every loop from TeleOp. */
    public void driveCode(Gamepad gamepad1) {
        double y  = -gamepad1.left_stick_y; // up is negative on stick
        double x  =  gamepad1.left_stick_x;
        double rxDriver = -gamepad1.right_stick_x; // FIXED: inverted turn
        double rx = (turnOverride != null) ? turnOverride : rxDriver;

        // Slow turn with triggers
        double slowLeft  =  0.20 * gamepad1.left_trigger;
        double slowRight = -0.20 * gamepad1.right_trigger;
        double slowTurn  = slowLeft + slowRight;

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Field-centric transform
        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        rotX *= 1.1; // imperfect strafe compensation

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);

        double frontLeftPower  = (rotY + rotX + rx) / denominator;
        double backLeftPower   = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower  = (rotY + rotX - rx) / denominator;

        // Apply slow turn after main mix
        frontLeftPower  += slowTurn;
        backLeftPower   += slowTurn;
        frontRightPower -= slowTurn;
        backRightPower  -= slowTurn;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        if (gamepad1.back) {
            RobotHardware.imu.resetYaw();
        }
    }
}