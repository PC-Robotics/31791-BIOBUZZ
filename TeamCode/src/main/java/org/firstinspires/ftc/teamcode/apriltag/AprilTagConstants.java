package org.firstinspires.ftc.teamcode.apriltag;

public class AprilTagConstants {

    // Camera configuration
    public static final double CAMERA_PITCH_ANGLE_DEG = 15.0; // Pitch angle of camera in degrees
    public static final double CAMERA_HEIGHT_INCHES = 8.5;    // Camera height from ground
    public static final double CAMERA_X_OFFSET_INCHES = 0.0;   // Forward offset from center of robot
    public static final double CAMERA_Y_OFFSET_INCHES = 0.0;   // Lateral offset from center of robot

    // Shot alignment configuration
    public static final double DESIRED_SHOT_DISTANCE_INCHES = 13.0; // Camera distance from goal for shot (13 inches)

    // Known AprilTag IDs for goals/targets
    public static final int BLUE_GOAL_TAG_ID = 1;
    public static final int RED_GOAL_TAG_ID = 2;
    public static final int BLUE_BACKDROP_TAG_ID = 3;
    public static final int RED_BACKDROP_TAG_ID = 4;

    // Known heights off ground for tags (in inches)
    public static final double GOAL_TAG_HEIGHT_INCHES = 12.0;
    public static final double BACKDROP_TAG_HEIGHT_INCHES = 14.5;
    public static final double WALL_TAG_HEIGHT_INCHES = 6.0;

    // Web camera device name in FTC hardware map
    public static final String CAMERA_HARDWARE_NAME = "Webcam 1";
}
