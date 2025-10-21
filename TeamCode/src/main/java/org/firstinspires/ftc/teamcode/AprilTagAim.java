package org.firstinspires.ftc.teamcode;

/** Small PID used to auto-aim robot towards an AprilTag (input = yaw error in radians). */
public class AprilTagAim {
    private double kP, kI, kD;

    private double deadbandRad = Math.toRadians(2.0);
    private double outMax = 0.6;
    private double iMax = 0.4;

    private double integral = 0.0;
    private double lastError = 0.0;
    private boolean first = true;

    private boolean invert = true;

    public AprilTagAim(double kP, double kI, double kD) {
        this.kP = kP; this.kI = kI; this.kD = kD;
    }

    public AprilTagAim setDeadbandRad(double db) { this.deadbandRad = Math.max(0, db); return this; }
    public AprilTagAim setOutputMax(double max)  { this.outMax = Math.max(0, max); return this; }
    public AprilTagAim setIntegralMax(double im) { this.iMax = Math.max(0, im); return this; }
    public AprilTagAim setInvert(boolean inv)    { this.invert = inv; return this; }

    public void reset() {
        integral = 0.0;
        lastError = 0.0;
        first = true;
    }

    public double update(double yawErrorRad, double dtSec) {
        double err = (Math.abs(yawErrorRad) < deadbandRad) ? 0.0 : yawErrorRad;
        if (invert) err = -err;

        double p = kP * err;

        integral += err * dtSec;
        if (integral >  iMax) integral =  iMax;
        if (integral < -iMax) integral = -iMax;
        double i = kI * integral;

        double deriv = first ? 0.0 : (err - lastError) / Math.max(dtSec, 1e-3);
        double d = kD * deriv;

        first = false;
        lastError = err;

        double out = p + i + d;
        if (out >  outMax) out =  outMax;
        if (out < -outMax) out = -outMax;
        return out;
    }
}
