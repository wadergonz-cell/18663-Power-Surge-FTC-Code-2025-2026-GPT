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
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

/**
 * Opens "Webcam 1", runs OpenFTC AprilTag (tag36h11), exposes latest pose.
 * Getters: hasTarget, id, X/Y/Z (meters, camera frame), yaw error (rad).
 */
public class AprilTagHelper {

    // Tag size (meters)
    public static double TAG_SIZE_M = AprilTagCalibration.getTagSizeMeters();

    // Approximated intrinsics for 640x480 (re  place with calibrated when ready)
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
    private volatile boolean streamingRequested = false;
    private volatile boolean streamingActive = false;
    private final Map<Integer, DetectionInfo> latestDetectionsById = new HashMap<>();
    private final Set<Integer> startZoneTagSet = new HashSet<>();
    private static volatile Integer startZoneTagId = null;

    public AprilTagHelper(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        for (int tagId : RobotConstants.START_ZONE_TAG_IDS) {
            startZoneTagSet.add(tagId);
        }
        resetStartZoneTag();
    }

    public void init() {
        int viewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), viewId);

        pipeline = new AprilTagPipeline(TAG_SIZE_M, FX, FY, CX, CY);
        webcam.setPipeline(pipeline);

        streamingRequested = true;

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() {
                webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, ROTATION);
                streamingActive = true;
            }
            @Override public void onError(int errorCode) { /* telemetry optional */ }
        });
    }

    public void stop() {
        streamingRequested = false;
        streamingActive = false;
        try { if (webcam != null) webcam.stopStreaming(); } catch (Exception ignored) {}
    }

    public void setCameraEnabled(boolean enabled) {
        streamingRequested = enabled;
        if (webcam == null) {
            return;
        }

        if (enabled && !streamingActive) {
            try {
                webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, ROTATION);
                streamingActive = true;
            } catch (Exception ignored) {
            }
        } else if (!enabled && streamingActive) {
            try {
                webcam.stopStreaming();
            } catch (Exception ignored) {
            }
            streamingActive = false;
        }
    }

    public boolean isCameraEnabled() {
        return streamingRequested;
    }

    public boolean isCameraStreaming() {
        return streamingActive;
    }

    /** Pull latest detection once per loop. */
    public void update() {
        ArrayList<AprilTagDetection> dets = pipeline.getLatestDetections();
        latestDetectionsById.clear();
        if (dets == null || dets.isEmpty()) {
            hasTarget = false;
            targetId = -1; x_m = y_m = z_m = yawErrorRad = 0;
            return;
        }

        // choose closest by z
        AprilTagDetection best = dets.get(0);
        for (int i = 0; i < dets.size(); i++) {
            AprilTagDetection detection = dets.get(i);
            DetectionInfo info = DetectionInfo.fromDetection(detection);
            latestDetectionsById.put(detection.id, info);

            if (startZoneTagId == null && startZoneTagSet.contains(detection.id)) {
                startZoneTagId = detection.id;
            }

            if (detection.pose.z < best.pose.z) {
                best = detection;
            }
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

    /** Returns the cached ID for the first start-zone tag (21/22/23) detected this match. */
    public static Integer getStartZoneTagId() {
        return startZoneTagId;
    }

    public static boolean hasStartZoneTag() {
        return startZoneTagId != null;
    }

    public static void resetStartZoneTag() {
        startZoneTagId = null;
    }

    /**
     * Returns detection info for a specific AprilTag ID if it was seen on the latest frame.
     */
    public DetectionInfo getDetectionInfo(int tagId) {
        return latestDetectionsById.get(tagId);
    }

    /** Returns all detections from the most recent frame. */
    public Collection<DetectionInfo> getAllDetections() {
        return latestDetectionsById.values();
    }

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
                    2,  // decimate (higher FPS for smoother tracking)
                    4   // threads
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

    /** Lightweight DTO describing the pose for a detected tag. */
    public static class DetectionInfo {
        public final int id;
        public final double x_m;
        public final double y_m;
        public final double z_m;
        public final double yawErrorRad;

        private DetectionInfo(int id, double x_m, double y_m, double z_m, double yawErrorRad) {
            this.id = id;
            this.x_m = x_m;
            this.y_m = y_m;
            this.z_m = z_m;
            this.yawErrorRad = yawErrorRad;
        }

        static DetectionInfo fromDetection(AprilTagDetection detection) {
            double yaw = Math.atan2(detection.pose.x, detection.pose.z);
            return new DetectionInfo(
                    detection.id,
                    detection.pose.x,
                    detection.pose.y,
                    detection.pose.z,
                    yaw
            );
        }

        public double getHorizontalRangeInches() {
            return detectionMetersToInches(z_m);
        }

        public double getYawErrorDegrees() {
            return Math.toDegrees(yawErrorRad);
        }

        private static double detectionMetersToInches(double meters) {
            return meters * 39.3701;
        }
    }
}
