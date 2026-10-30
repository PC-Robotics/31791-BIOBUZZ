package org.firstinspires.ftc.teamcode.driver;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.apriltag.AprilTagAligner;
import org.firstinspires.ftc.teamcode.apriltag.AprilTagConstants;
import org.firstinspires.ftc.teamcode.apriltag.AprilTagData;
import org.firstinspires.ftc.teamcode.apriltag.AprilTagDetector;
import org.firstinspires.ftc.teamcode.drive.DriveBasePID;

/**
 * Full-Featured TeleOp OpMode combining:
 * 1) DriveBasePID state machine (DRIVE, POSITION_LOCK, TAG_ALIGN)
 * 2) AprilTag vision alignment maintaining 13-inch camera shot distance to goal
 * 3) Launcher and intake systems from BioBuzzStarterTeleop3
 */
@TeleOp(name = "BioBuzz PID TeleOp with AprilTag & Launcher", group = "Driver")
public class BioBuzzPIDTeleOp extends LinearOpMode {

    // ===== CONFIGURATION & APRILTAG CONSTANTS =====
    public static final double CAMERA_ANGLE_DEG = 15.0;            // Pitch angle of camera in degrees
    public static final double DESIRED_SHOT_DISTANCE_INCHES = 13.0; // Camera distance from goal for shot (13 inches)
    public static final int BLUE_GOAL_TAG_ID = AprilTagConstants.BLUE_GOAL_TAG_ID; // Tag 1
    public static final int RED_GOAL_TAG_ID = AprilTagConstants.RED_GOAL_TAG_ID;   // Tag 2
    public static final int DEFAULT_TARGET_TAG_ID = BLUE_GOAL_TAG_ID;

    // Launcher speed constants
    public static final int LAUNCHER_DEFAULT_TARGET_VELOCITY = 1265;
    public static final int LAUNCHER_MIN_VELOCITY = 50;
    public static final int LAUNCHER_TIMEOUT = 5000;

    public enum DriveState {
        DRIVE,
        POSITION_LOCK,
        TAG_ALIGN
    }

    // Hardware
    private DriveBasePID robot;
    private AprilTagDetector aprilTagDetector;
    private DcMotorEx launcher;
    private DcMotor intake;
    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;
    private CRServo windmillServo;

    // State Variables
    private DriveState driveState = DriveState.DRIVE;
    private double lockX = 0;
    private double lockY = 0;
    private double goalX = 0;
    private double goalY = 0;

    // Launcher & Intake state
    public int launcher_target_velocity = LAUNCHER_DEFAULT_TARGET_VELOCITY;
    private boolean reachedTarget = false;
    private boolean subtracted = false;
    private int launches = 0;
    private double intakePower = 0.0;
    private final ElapsedTime lastLaunchMillis = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Drivetrain
        robot = new DriveBasePID(this, true);
        robot.init();

        // Initialize AprilTag Vision Detector
        aprilTagDetector = new AprilTagDetector();
        aprilTagDetector.init(hardwareMap, telemetry, CAMERA_ANGLE_DEG);

        // Initialize Launcher & Intake Actuators
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");

        intake.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));

        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized with AprilTag Detector & Launcher");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            handleDrive();
            handleIntake();
            launch();
            handleControllerButtons();

            robot.updatePositionAndTelemetry();

            // Telemetry Output
            telemetry.addData("Drive State", driveState);
            telemetry.addData("Lock X (in)", "%.2f", lockX);
            telemetry.addData("Lock Y (in)", "%.2f", lockY);
            telemetry.addData("Launcher Target Vel", launcher_target_velocity);
            telemetry.addData("Launcher Real Vel", launcher.getVelocity());
            telemetry.addData("Queued Launches", launches);

            AprilTagData tagData = aprilTagDetector.getTagDetails(DEFAULT_TARGET_TAG_ID);
            if (tagData != null) {
                telemetry.addData("AprilTag " + DEFAULT_TARGET_TAG_ID,
                        "Dist: %.2fin | Horiz: %.2fin | Plane: %s",
                        tagData.getStraightDistanceInches(),
                        tagData.getHorizontalDistanceInches(),
                        tagData.getPlane());
            } else {
                telemetry.addData("AprilTag " + DEFAULT_TARGET_TAG_ID, "Not Visible");
            }

            telemetry.update();
        }

        aprilTagDetector.stop();
    }

    private void handleDrive() {

        double axial = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yawInput = gamepad1.right_stick_x;
        double yaw = yawInput;

        // ===== STATE SWITCH BUTTONS =====

        if(gamepad1.triangleWasPressed()){

            Pose2D pose = robot.getRobotPosition();

            lockX = pose.getX(DistanceUnit.INCH);
            lockY = pose.getY(DistanceUnit.INCH);

            robot.setLockX(lockX);
            robot.setLockY(lockY);
            robot.setLockHeading(
                    pose.getHeading(AngleUnit.DEGREES));

            driveState = DriveState.POSITION_LOCK;
        }

        if(gamepad1.left_trigger > 0.5){
            driveState = DriveState.TAG_ALIGN;
        }

        if(gamepad1.crossWasPressed()){
            driveState = DriveState.DRIVE;
        }

        // ===== STICK DETECTION =====

        boolean sticksActive =
                Math.abs(axial) > 0.01 ||
                        Math.abs(lateral) > 0.01 ||
                        Math.abs(yawInput) > 0.01;

        if(sticksActive && driveState != DriveState.TAG_ALIGN){
            driveState = DriveState.DRIVE;
        }
        else if(!sticksActive && driveState == DriveState.DRIVE){

            Pose2D pose = robot.getRobotPosition();

            lockX = pose.getX(DistanceUnit.INCH);
            lockY = pose.getY(DistanceUnit.INCH);

            robot.setLockX(lockX);
            robot.setLockY(lockY);
            robot.setLockHeading(
                    pose.getHeading(AngleUnit.DEGREES));

            driveState = DriveState.POSITION_LOCK;
        }

        // ===== STATE MACHINE =====

        switch(driveState){

            case POSITION_LOCK:

                robot.goToPositionNonBlocking(
                        lockY,
                        lockX,
                        robot.getHeading(AngleUnit.DEGREES),
                        0.7);
                break;

            case TAG_ALIGN:

                if(gamepad1.left_trigger <= 0.5 ||
                        Math.abs(yawInput) > 0.15){

                    driveState = DriveState.DRIVE;
                    break;
                }

                Pose2D pose = robot.getRobotPosition();

                // Check AprilTag detection for goal alignment at 13 inches
                AprilTagData tagData = aprilTagDetector.getTagDetails(DEFAULT_TARGET_TAG_ID);
                if (tagData == null) {
                    // Fallback to closest tag if default target tag is not visible
                    tagData = aprilTagDetector.getClosestTag();
                }

                if (tagData != null) {
                    // Calculate exact 13-inch camera shot position from AprilTag
                    AprilTagAligner.ShotTargetPose shotPose = AprilTagAligner.calculateShotTarget(
                            tagData, pose, DESIRED_SHOT_DISTANCE_INCHES
                    );

                    if (shotPose.isValid) {
                        // Drive non-blocking to lock position 13 inches away facing goal tag
                        robot.goToPositionNonBlocking(
                                shotPose.targetY,
                                shotPose.targetX,
                                shotPose.targetHeadingDeg,
                                0.85
                        );
                        break;
                    }
                }

                // Fallback to calculated goal facing yaw if tag not currently visible
                double robotX = pose.getX(DistanceUnit.INCH);
                double robotY = pose.getY(DistanceUnit.INCH);

                double currentHeading =
                        robot.getHeading(AngleUnit.DEGREES);

                yaw = calculateGoalFacingYaw(
                        currentHeading,
                        robotX,
                        robotY);

                robot.drive(axial, lateral, yaw, 1);
                break;

            case DRIVE:

                robot.drive(axial, lateral, yawInput, 1);
                break;
        }
    }

    // Update intake power
    public void handleIntake() {
        intakePower = (gamepad1.right_trigger + gamepad2.right_trigger) - (gamepad1.left_trigger + gamepad2.left_trigger);

        intake.setPower(intakePower);
        leftIntakeServo.setPower(intakePower);
        rightIntakeServo.setPower(intakePower);
    }

    // Call from loop to handle launcher functions
    void launch() {
        // Add another ball to the launch queue
        if (gamepad1.rightBumperWasPressed() || gamepad2.rightBumperWasPressed()) {
            launches += 1;
            lastLaunchMillis.reset();
        }

        // Set launcher motor speed
        if (launches > 0) {
            launcher.setVelocity(launcher_target_velocity);
        } else {
            launcher.setVelocity(0);
            reachedTarget = false;
        }

        // Timeout if no balls are shot within the time limit
        if (lastLaunchMillis.time() > LAUNCHER_TIMEOUT && launches > 0) {
            launches = 0;
        }

        // The launcher reached target once passed min velocity
        if (launcher.getVelocity() > (launcher_target_velocity - LAUNCHER_MIN_VELOCITY)) {
            reachedTarget = true;
            subtracted = false;
        }

        // If the launcher slows down below minimum, a ball has been shot
        if (launcher.getVelocity() < (launcher_target_velocity - LAUNCHER_MIN_VELOCITY) && reachedTarget) {
            launches -= 1;
            subtracted = true;
            lastLaunchMillis.reset();
        }

        // Once the launcher gets up to speed, feed balls and slowly run intake to dislodge
        // stuck balls
        if (launches > 0 && launcher.getVelocity() > (launcher_target_velocity - LAUNCHER_MIN_VELOCITY)) {
            windmillServo.setPower(1);
            intakePower += 0.5;
        } else if (!gamepad1.left_bumper) {
            windmillServo.setPower(0);
        }
    }

    private void handleControllerButtons() {
        if (gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed()) {
            launcher_target_velocity += 5;
        } else if (gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed()) {
            launcher_target_velocity -= 5;
        }

        if (gamepad1.dpad_left || gamepad2.dpad_left) {
            windmillServo.setPower(-1);
        } else if (gamepad1.dpad_right || gamepad2.dpad_right) {
            windmillServo.setPower(1);
        }
    }

    /**
     * Calculates the target heading (in degrees) required for the robot to face the target goal.
     */
    private double calculateGoalFacingYaw(double currentHeading, double robotX, double robotY) {
        double dx = goalX - robotX;
        double dy = goalY - robotY;
        if (Math.hypot(dx, dy) < 1e-3) {
            return currentHeading;
        }
        return Math.toDegrees(Math.atan2(dy, dx));
    }
}
