package org.firstinspires.ftc.teamcode;

/**
 * Central place for AprilTag calibration and distance math.
 * All sizes are INCHES, angles are DEGREES.
 */
public class AprilTagCalibration {

    // === USER TUNABLES ===
    public static double TAG_SIZE_IN = 3.35;             // black-square size 8.35
    public static double APRIL_TAG_CENTER_HEIGHT_IN = 29.5;
    public static double CAMERA_HEIGHT_IN           = 14.25;
    public static double CAMERA_PITCH_DEG           = -22.0;   // +down
    public static double CAMERA_YAW_OFFSET_DEG      = 1.5;    // +left

    // Optional intrinsics (at stream size)
    public static double FX = 600, FY = 600, CX = 320, CY = 240;
    public static int STREAM_WIDTH = 640, STREAM_HEIGHT = 480;

    // === CONVENIENCE ===
    public static final double INCH_PER_M = 39.37007874;
    public static final double M_PER_INCH = 1.0 / INCH_PER_M;

    public static double inchesToMeters(double in) { return in * M_PER_INCH; }
    public static double metersToInches(double m)  { return m * INCH_PER_M; }

    public static double getTagSizeMeters() { return inchesToMeters(TAG_SIZE_IN); }

    // === REPORT STRUCT ===
    public static class Report {
        public final boolean hasTag;
        public final int tagId;
        public final double forwardIn, lateralIn, verticalIn;
        public final double horizontalRangeIn;
        public final double deltaHeightIn;
        public final double yawCorrectedRad;

        public Report(boolean hasTag, int tagId,
                      double forwardIn, double lateralIn, double verticalIn,
                      double horizontalRangeIn, double deltaHeightIn, double yawCorrectedRad) {
            this.hasTag = hasTag;
            this.tagId = tagId;
            this.forwardIn = forwardIn;
            this.lateralIn = lateralIn;
            this.verticalIn = verticalIn;
            this.horizontalRangeIn = horizontalRangeIn;
            this.deltaHeightIn = deltaHeightIn;
            this.yawCorrectedRad = yawCorrectedRad;
        }
    }

    // === CORE MATH ===
    public static double[] leveledPose(double x_m, double y_m, double z_m) {
        double pitch = Math.toRadians(CAMERA_PITCH_DEG);
        double xL = x_m;
        double yL =  y_m * Math.cos(pitch) - z_m * Math.sin(pitch);
        double zL =  y_m * Math.sin(pitch) + z_m * Math.cos(pitch);
        return new double[]{ xL, yL, zL };
    }

    public static Report computeReport(boolean hasTag, int id,
                                       double x_m, double y_m, double z_m,
                                       double rawYawRad) {
        if (!hasTag) {
            return new Report(false, -1, 0,0,0, 0,0, 0);
        }

        double[] lvl = leveledPose(x_m, y_m, z_m);
        double xL_m = lvl[0], yL_m = lvl[1], zL_m = lvl[2];

        double forwardIn  = metersToInches(zL_m);
        double lateralIn  = metersToInches(xL_m);
        double verticalIn = metersToInches(yL_m);

        double horizontalRangeIn = Math.hypot(forwardIn, lateralIn);
        double deltaHeightIn = APRIL_TAG_CENTER_HEIGHT_IN - CAMERA_HEIGHT_IN;
        double yawCorrectedRad = rawYawRad + Math.toRadians(CAMERA_YAW_OFFSET_DEG);

        return new Report(true, id, forwardIn, lateralIn, verticalIn,
                horizontalRangeIn, deltaHeightIn, yawCorrectedRad);
    }
}
