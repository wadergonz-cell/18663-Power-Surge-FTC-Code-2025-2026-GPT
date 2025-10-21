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

    // === Drive motors ===
    public DcMotor frontLeftMotor;   // Control Hub 3
    public DcMotor backLeftMotor;    // Control Hub 0
    public DcMotor frontRightMotor;  // Control Hub 1
    public DcMotor backRightMotor;   // Control Hub 2

    // === Mechanisms (motors) ===
    public DcMotor leftOuttakeMotor;   // Expansion Hub 1
    public DcMotor rightOuttakeMotor;  // Expansion Hub 2
    public DcMotor intakeMotor;        // Expansion Hub 0
    public DcMotor frontIntakeMotor;   // Expansion Hub 3

    // === Servos ===
    public Servo outtakeServo;         // Control Hub servo 0 (config: "outtakeServo")
    public Servo ballBlocker;          // Control Hub servo 1 (config: "BallBlocker")

    // === IMU ===
    public IMU imu;

    public robotHardware(OpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;

        // IMU (match your mounting)
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN));
        imu.initialize(parameters);

        // Drive motors
        frontLeftMotor  = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor   = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor  = hardwareMap.dcMotor.get("backRightMotor");

        // Your directions
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Mechanism motors
        leftOuttakeMotor  = hardwareMap.dcMotor.get("leftOuttakeMotor");
        rightOuttakeMotor = hardwareMap.dcMotor.get("rightOuttakeMotor");
        rightOuttakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMotor       = hardwareMap.dcMotor.get("intakeMotor");
        frontIntakeMotor  = hardwareMap.dcMotor.get("frontIntakeMotor");

        // FIXED: Reverse both intake motors
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Servos
        outtakeServo = hardwareMap.servo.get("outtakeServo");

        ballBlocker = hardwareMap.servo.get("BallBlocker");
        // Per your note: reverse the BallBlocker direction and use 0.0 (up) / ~0.40 (down)
        ballBlocker.setDirection(Servo.Direction.REVERSE);
    }
}