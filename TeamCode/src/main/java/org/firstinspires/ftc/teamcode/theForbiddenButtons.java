package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Button handler for drive motors (servos, slow turn).
 * Shooting sequence logic is now handled by ShootingSequenceController.
 */
public class theForbiddenButtons {
    private final robotHardware RobotHardware;

    // Servos
    private final Servo outtakeServo;
    private final Servo blockerServo;

    // Servo positions
    private static final double OUTTAKE_UP_POS   = 0.45;
    private static final double OUTTAKE_DOWN_POS = 0.245;
    private static final double BLOCKER_UP_POS   = 0.00;
    private static final double BLOCKER_DOWN_POS = 0.40;

    public theForbiddenButtons(robotHardware RobotHardware) {
        this.RobotHardware = RobotHardware;

        // Map hardware
        outtakeServo = RobotHardware.outtakeServo;
        blockerServo = RobotHardware.ballBlocker;

        // Zero servos at init
        safeSetServo(outtakeServo, OUTTAKE_UP_POS);
        safeSetServo(blockerServo, BLOCKER_UP_POS);
    }

    /**
     * Call this every TeleOp loop for non-shooting buttons (if needed).
     * Most button logic is now in ShootingSequenceController.
     */
    public void driveOnlyInputs(Gamepad gamepad1) {
        // Slow turn with triggers (if you want to keep this)
        double slowLeft  =  0.20 * gamepad1.left_trigger;
        double slowRight = -0.20 * gamepad1.right_trigger;
        // Apply these in driveTrainChooChoo if desired
    }

    private void safeSetServo(Servo s, double pos) {
        double clipped = Math.max(0.0, Math.min(1.0, pos));
        s.setPosition(clipped);
    }
}