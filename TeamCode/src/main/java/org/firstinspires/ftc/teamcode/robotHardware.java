package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class robotHardware {
    private HardwareMap hardwareMap;

    public DcMotor frontLeftMotor;
    //Control Hub 3
    public DcMotor backLeftMotor;
    //Control Hub 0
    public DcMotor frontRightMotor;
    //Control Hub 1
    public DcMotor backRightMotor;
    //Expansion Hub 1
    public DcMotor leftOuttakeMotor;
    //Expansion Hub 2
    public DcMotor rightOuttakeMotor;
    //Expansion Hub 0
    public DcMotor intakeMotor;
    //Expansion Hub 3
    public DcMotor frontIntakeMotor;

    public IMU imu;

    public Servo outtakeServo;

    public robotHardware(OpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;


        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN));

        imu.initialize(parameters);

        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");


        //Control Hub 3
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");

        //Control Hub 0
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //Control Hub 1
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //Expansion Hub 1
         leftOuttakeMotor = hardwareMap.dcMotor.get("leftOuttakeMotor");

        //Expansion Hub 2
        rightOuttakeMotor = hardwareMap.dcMotor.get("rightOuttakeMotor");
        rightOuttakeMotor.setDirection((DcMotorSimple.Direction.REVERSE));
        //Expansion Hub 0
         intakeMotor = hardwareMap.dcMotor.get("intakeMotor");

        //Expansion Hub 3
        frontIntakeMotor = hardwareMap.dcMotor.get("frontIntakeMotor");
        frontIntakeMotor.setDirection((DcMotorSimple.Direction.REVERSE));

        //Control Hub 0
        outtakeServo = hardwareMap.servo.get("outtakeServo");
    }


}
