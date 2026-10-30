package org.firstinspires.ftc.teamcode.apriltag;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class AprilTagAligner {

    public static class ShotTargetPose {
        public final double targetX;
        public final double targetY;
        public final double targetHeadingDeg;
        public final double distanceErrorInches;
        public final boolean isValid;

        public ShotTargetPose(double targetX, double targetY, double targetHeadingDeg, double distanceErrorInches, boolean isValid) {
            this.targetX = targetX;
            this.targetY = targetY;
            this.targetHeadingDeg = targetHeadingDeg;
            this.distanceErrorInches = distanceErrorInches;
            this.isValid = isValid;
        }
    }

    /**
     * Calculates the exact robot field target position and heading required
     * so that the camera is exactly desiredDistanceInches (e.g. 10 inches) away from the goal tag.
     */
    public static ShotTargetPose calculateShotTarget(AprilTagData tagData, Pose2D currentRobotPose, double desiredDistanceInches) {
        if (tagData == null) {
            return new ShotTargetPose(
                    currentRobotPose.getX(DistanceUnit.INCH),
                    currentRobotPose.getY(DistanceUnit.INCH),
                    currentRobotPose.getHeading(AngleUnit.DEGREES),
                    0,
                    false
            );
        }

        double robotX = currentRobotPose.getX(DistanceUnit.INCH);
        double robotY = currentRobotPose.getY(DistanceUnit.INCH);
        double currentHeadingDeg = currentRobotPose.getHeading(AngleUnit.DEGREES);

        // Relative offset of tag from camera/robot
        double tagRelY = tagData.getYInches(); // Forward distance to tag
        double tagRelX = tagData.getXInches(); // Lateral distance to tag

        // Calculate current horizontal distance to tag
        double currentHorizDist = tagData.getHorizontalDistanceInches();
        double distanceError = currentHorizDist - desiredDistanceInches;

        // Convert tag location into field coordinates
        double headingRad = Math.toRadians(currentHeadingDeg);
        double fieldTagX = robotX + (tagRelY * Math.cos(headingRad) - tagRelX * Math.sin(headingRad));
        double fieldTagY = robotY + (tagRelY * Math.sin(headingRad) + tagRelX * Math.cos(headingRad));

        // Calculate angle from goal tag back to robot
        double angleFromTagToBotRad = Math.atan2(robotY - fieldTagY, robotX - fieldTagX);

        // Robot target position exactly desiredDistanceInches away from tag along the line of sight
        double targetX = fieldTagX + desiredDistanceInches * Math.cos(angleFromTagToBotRad);
        double targetY = fieldTagY + desiredDistanceInches * Math.sin(angleFromTagToBotRad);

        // Target heading facing directly at the goal tag
        double targetHeadingDeg = Math.toDegrees(Math.atan2(fieldTagY - targetY, fieldTagX - targetX));

        return new ShotTargetPose(targetX, targetY, targetHeadingDeg, distanceError, true);
    }
}
