package org.firstinspires.ftc.teamcode.apriltag;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.ArrayList;
import java.util.List;

public class AprilTagDetector {

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private Telemetry telemetry;
    private double cameraPitchAngleDeg = AprilTagConstants.CAMERA_PITCH_ANGLE_DEG;

    public void init(HardwareMap hwMap, Telemetry telemetry) {
        init(hwMap, telemetry, AprilTagConstants.CAMERA_PITCH_ANGLE_DEG);
    }

    public void init(HardwareMap hwMap, Telemetry telemetry, double cameraPitchAngleDeg) {
        this.telemetry = telemetry;
        this.cameraPitchAngleDeg = cameraPitchAngleDeg;

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, AprilTagConstants.CAMERA_HARDWARE_NAME));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();
    }

    public void setCameraPitchAngleDeg(double angleDeg) {
        this.cameraPitchAngleDeg = angleDeg;
    }

    public AprilTagData getTagDetails(int id) {
        if (aprilTagProcessor == null) return null;
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection single = (AprilTagSingleDetection) detection;
                if (single.id == id) {
                    return processDetection(single);
                }
            }
        }
        return null;
    }

    public AprilTagData getClosestTag() {
        if (aprilTagProcessor == null) return null;
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        AprilTagSingleDetection closest = null;
        double minDistance = Double.MAX_VALUE;

        for (AprilTagDetection detection : detections) {
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection single = (AprilTagSingleDetection) detection;
                if (single.ftcPose != null && single.ftcPose.range < minDistance) {
                    minDistance = single.ftcPose.range;
                    closest = single;
                }
            }
        }

        if (closest != null) {
            return processDetection(closest);
        }
        return null;
    }

    public List<AprilTagData> getAllDetectedTags() {
        List<AprilTagData> resultList = new ArrayList<>();
        if (aprilTagProcessor == null) return resultList;
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection instanceof AprilTagSingleDetection) {
                resultList.add(processDetection((AprilTagSingleDetection) detection));
            }
        }
        return resultList;
    }

    private AprilTagData processDetection(AprilTagSingleDetection detection) {
        int id = detection.id;
        String tagName = (detection.metadata != null) ? detection.metadata.name : "Tag_" + id;

        // Determine plane based on tag ID
        AprilTagData.TagPlane plane = determineTagPlane(id);

        double straightDistance = 0;
        double horizontalDistance = 0;
        double heightInches = 0;
        double x = 0;
        double y = 0;
        double z = 0;
        double bearing = 0;
        double yaw = 0;

        if (detection.ftcPose != null) {
            x = detection.ftcPose.x;
            y = detection.ftcPose.y;
            z = detection.ftcPose.z;
            straightDistance = detection.ftcPose.range;
            bearing = detection.ftcPose.bearing;
            yaw = detection.ftcPose.yaw;

            // Pitch angle compensation for straight-line vs horizontal distance
            double totalElevationAngleRad = Math.toRadians(detection.ftcPose.elevation + cameraPitchAngleDeg);
            horizontalDistance = straightDistance * Math.cos(totalElevationAngleRad);

            // Calculate height off ground: Known tag height + Z elevation offset from camera
            double knownBaseHeight = getKnownTagHeight(plane);
            heightInches = knownBaseHeight + z;
        }

        return new AprilTagData(
                id, tagName, plane,
                straightDistance, horizontalDistance, heightInches,
                x, y, z, bearing, yaw
        );
    }

    public AprilTagData.TagPlane determineTagPlane(int id) {
        if (id == AprilTagConstants.BLUE_GOAL_TAG_ID || id == AprilTagConstants.RED_GOAL_TAG_ID) {
            return AprilTagData.TagPlane.GOAL_PLANE;
        } else if (id == AprilTagConstants.BLUE_BACKDROP_TAG_ID || id == AprilTagConstants.RED_BACKDROP_TAG_ID || (id >= 1 && id <= 6)) {
            return AprilTagData.TagPlane.BACKDROP_PLANE;
        } else if (id >= 7 && id <= 10) {
            return AprilTagData.TagPlane.AUDIENCE_WALL_PLANE;
        } else if (id >= 11 && id <= 16) {
            return AprilTagData.TagPlane.SIDE_WALL_PLANE;
        }
        return AprilTagData.TagPlane.UNKNOWN_PLANE;
    }

    private double getKnownTagHeight(AprilTagData.TagPlane plane) {
        switch (plane) {
            case GOAL_PLANE:
                return AprilTagConstants.GOAL_TAG_HEIGHT_INCHES;
            case BACKDROP_PLANE:
                return AprilTagConstants.BACKDROP_TAG_HEIGHT_INCHES;
            default:
                return AprilTagConstants.WALL_TAG_HEIGHT_INCHES;
        }
    }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
