package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

/**
 * Opens "Webcam 1", runs OpenFTC AprilTag (tag36h11), exposes latest pose.
 * Getters: hasTarget, id, X/Y/Z (meters, camera frame), yaw error (rad).
 */
public class AprilTagHelper {

    // Tag size (meters)
    public static double TAG_SIZE_M = AprilTagCalibration.getTagSizeMeters();

    // Approximated intrinsics for 640x480 (replace with calibrated when ready)
    public static double FX = AprilTagCalibration.FX;
    public static double FY = AprilTagCalibration.FY;
    public static double CX = AprilTagCalibration.CX;
    public static double CY = AprilTagCalibration.CY;

    public static int STREAM_WIDTH  = AprilTagCalibration.STREAM_WIDTH;
    public static int STREAM_HEIGHT = AprilTagCalibration.STREAM_HEIGHT;
    public static OpenCvCameraRotation ROTATION = OpenCvCameraRotation.UPRIGHT;

    // Outputs
    private volatile boolean hasTarget = false;
    private volatile int     targetId  = -1;
    private volatile double  x_m = 0;   // +right
    private volatile double  y_m = 0;   // +down
    private volatile double  z_m = 0;   // +forward
    private volatile double  yawErrorRad = 0; // atan2(x,z)

    // Internals
    private final HardwareMap hardwareMap;
    private OpenCvWebcam webcam;
    private AprilTagPipeline pipeline;

    public AprilTagHelper(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        int viewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), viewId);

        pipeline = new AprilTagPipeline(TAG_SIZE_M, FX, FY, CX, CY);
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() {
                webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, ROTATION);
            }
            @Override public void onError(int errorCode) { /* telemetry optional */ }
        });
    }

    public void stop() {
        try { if (webcam != null) webcam.stopStreaming(); } catch (Exception ignored) {}
    }

    /** Pull latest detection once per loop. */
    public void update() {
        ArrayList<AprilTagDetection> dets = pipeline.getLatestDetections();
        if (dets == null || dets.isEmpty()) {
            hasTarget = false;
            targetId = -1; x_m = y_m = z_m = yawErrorRad = 0;
            return;
        }

        // choose closest by z
        AprilTagDetection best = dets.get(0);
        for (int i = 1; i < dets.size(); i++) {
            if (dets.get(i).pose.z < best.pose.z) best = dets.get(i);
        }

        hasTarget   = true;
        targetId    = best.id;
        x_m         = best.pose.x;
        y_m         = best.pose.y;
        z_m         = best.pose.z;
        yawErrorRad = Math.atan2(x_m, z_m);
    }

    // getters
    public boolean hasTarget()      { return hasTarget; }
    public int     getTargetId()    { return targetId; }
    public double  getX_m()         { return x_m; }
    public double  getY_m()         { return y_m; }
    public double  getZ_m()         { return z_m; }
    public double  getYawErrorRad() { return yawErrorRad; }

    // ---- Minimal AprilTag pipeline using OpenFTC native detector ----
    private static class AprilTagPipeline extends OpenCvPipeline {
        private final double tagSize, fx, fy, cx, cy;
        private long nativeTagPtr;
        private final Mat grey = new Mat();
        private volatile ArrayList<AprilTagDetection> latestDetections = new ArrayList<>();

        AprilTagPipeline(double tagSizeMeters, double fx, double fy, double cx, double cy) {
            this.tagSize = tagSizeMeters;
            this.fx = fx; this.fy = fy; this.cx = cx; this.cy = cy;
            nativeTagPtr = AprilTagDetectorJNI.createApriltagDetector(
                    AprilTagDetectorJNI.TagFamily.TAG_36h11.string,
                    3,  // decimate
                    3   // threads
            );
        }

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY);
            ArrayList<AprilTagDetection> detections =
                    AprilTagDetectorJNI.runAprilTagDetectorSimple(
                            nativeTagPtr, grey, tagSize, fx, fy, cx, cy);
            latestDetections = detections;
            return input; // show camera preview
        }

        ArrayList<AprilTagDetection> getLatestDetections() {
            return latestDetections;
        }

        @Override
        protected void finalize() throws Throwable {
            try {
                if (nativeTagPtr != 0) {
                    AprilTagDetectorJNI.releaseApriltagDetector(nativeTagPtr);
                    nativeTagPtr = 0;
                }
                grey.release();
            } finally {
                super.finalize();
            }
        }
    }
}
