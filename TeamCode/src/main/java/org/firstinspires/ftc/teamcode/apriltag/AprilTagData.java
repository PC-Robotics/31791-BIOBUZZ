package org.firstinspires.ftc.teamcode.apriltag;

import java.util.Locale;

public class AprilTagData {

    public enum TagPlane {
        BACKDROP_PLANE,
        GOAL_PLANE,
        AUDIENCE_WALL_PLANE,
        SIDE_WALL_PLANE,
        UNKNOWN_PLANE
    }

    private final int tagId;
    private final String tagName;
    private final TagPlane plane;
    private final double straightDistanceInches;
    private final double horizontalDistanceInches;
    private final double heightInches;
    private final double xInches;
    private final double yInches;
    private final double zInches;
    private final double bearingDegrees;
    private final double yawDegrees;

    public AprilTagData(int tagId, String tagName, TagPlane plane,
                        double straightDistanceInches, double horizontalDistanceInches,
                        double heightInches, double xInches, double yInches, double zInches,
                        double bearingDegrees, double yawDegrees) {
        this.tagId = tagId;
        this.tagName = tagName;
        this.plane = plane;
        this.straightDistanceInches = straightDistanceInches;
        this.horizontalDistanceInches = horizontalDistanceInches;
        this.heightInches = heightInches;
        this.xInches = xInches;
        this.yInches = yInches;
        this.zInches = zInches;
        this.bearingDegrees = bearingDegrees;
        this.yawDegrees = yawDegrees;
    }

    public int getTagId() { return tagId; }
    public String getTagName() { return tagName; }
    public TagPlane getPlane() { return plane; }
    public double getStraightDistanceInches() { return straightDistanceInches; }
    public double getHorizontalDistanceInches() { return horizontalDistanceInches; }
    public double getHeightInches() { return heightInches; }
    public double getXInches() { return xInches; }
    public double getYInches() { return yInches; }
    public double getZInches() { return zInches; }
    public double getBearingDegrees() { return bearingDegrees; }
    public double getYawDegrees() { return yawDegrees; }

    @Override
    public String toString() {
        return String.format(Locale.US, "Tag %d [%s] | Plane: %s | Dist: %.2fin | HorizDist: %.2fin | Height: %.2fin",
                tagId, tagName != null ? tagName : "Unknown", plane, straightDistanceInches, horizontalDistanceInches, heightInches);
    }
}
