package org.firstinspires.ftc.teamcode.driver;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import static org.firstinspires.ftc.teamcode.support.ConstantsPID.DRIVE_DEADBAND;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.DRIVE_KD;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.DRIVE_KI;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.DRIVE_KP;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.DRIVE_MAX_AUTO;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.DRIVE_TOLERANCE;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.STRAFE_DEADBAND;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.STRAFE_KD;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.STRAFE_KI;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.STRAFE_KP;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.STRAFE_MAX_AUTO;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.STRAFE_TOLERANCE;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.YAW_DEADBAND;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.YAW_KD;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.YAW_KI;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.YAW_KP;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.YAW_MAX_AUTO;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.YAW_TOLERANCE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.support.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.support.PIDController;

import java.util.Locale;

@TeleOp(name = "Hold Position Mirrored", group = "StarterBot")
//@Disabled

public class holdPositionMirrored extends OpMode {
    // CLASS VARIABLES =============================================================================
    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor leftRearDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightRearDrive = null;
    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;


    // Set target and min velocity of flywheel for launching (ticks per second)
    public int launcher_target_velocity = 1265;
    public final int LAUNCHER_MIN_VELOCITY = 50;
    public final int LAUNCHER_TIMEOUT = 5000;

    FtcDashboard Dashboard = FtcDashboard.getInstance();
    Pose2D robotPosition = null;
    Pose2D targetPos = null;
    Telemetry dBoardTelemetry = Dashboard.getTelemetry();
    ElapsedTime lastLaunchMillis = new ElapsedTime(); // Tracks the last launch time for timeout

    // Define drive motor power in entire class for telemetry
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;
    double intakePower;
    boolean field_centric_drive = true;
    boolean reachedTarget = false;
    boolean subtracted = false;
    int launches = 0; // Tracks how many balls need to be launched

    private final PIDController strafeController = new PIDController(STRAFE_KP, STRAFE_KI, STRAFE_KD, STRAFE_MAX_AUTO, STRAFE_TOLERANCE, STRAFE_DEADBAND, false);
    private final PIDController yawController = new PIDController(YAW_KP, YAW_KI, YAW_KD, YAW_MAX_AUTO, YAW_TOLERANCE, YAW_DEADBAND, true);
    private final PIDController driveController = new PIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_MAX_AUTO, DRIVE_TOLERANCE, DRIVE_DEADBAND, false);

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    // --- Simple Helpers ---
    public double getXPosition(DistanceUnit unit) { return robotPosition != null ? robotPosition.getX(unit) : 0; }
    public double getYPosition(DistanceUnit unit) { return robotPosition != null ? robotPosition.getY(unit) : 0; }
    public double getHeading(AngleUnit unit) { return robotPosition != null ? robotPosition.getHeading(unit) : 0; }


    // =============================================================================================
    // SYSTEM FUNCTION DEFINITIONS - Please don't put actual code in functions except init()
    // =============================================================================================

    // init() runs once after INIT is pressed
    @Override
    public void init() {
        // SET UP HARDWARE MAPS ====================================================================
        // Sensors
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        // Actuators
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftRearDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");


        // INITIALIZE ACTUATORS ====================================================================
        // Reverse left motors because robots
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);

        // Brake when not powered
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftRearDrive.setZeroPowerBehavior(BRAKE);
        rightRearDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        // Feedback for launcher
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // PID coefficients
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));

        // Start servos in off position
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);

        // Reverse right side because ball should go left, not right to be fed
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);


        // ODOMETRY ================================================================================
        odo.setOffsets(0, -4.0, DistanceUnit.MM);

        // Odo initialization
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        odo.update();


        // STATUS ==================================================================================
        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Heading Scalar", odo.getYawScalar());
        telemetry.update();
    }

    // Run repeatedly in INIT mode
    @Override
    public void init_loop() {
    }

    // Run once in START mode
    @Override
    public void start() {
        odo.resetPosAndIMU();
        targetPos = null;
    }

    // Run repeatedly in START mode
    @Override
    public void loop() {
        // Please do ***NOT*** put any code other than function calls here.
        launch();
        handleIntake();
        controllerButtons();

        updatePosition();
        if (targetPos == null) {
            targetPos = robotPosition;
        }
        updateTarget((gamepad1.left_stick_x + gamepad2.left_stick_x), -(gamepad1.left_stick_y + gamepad2.left_stick_y), (gamepad1.right_stick_x + gamepad2.right_stick_x));

        displayTelemetry();
    }

    public void updateTarget(double targetX, double targetY, double rotate) {
            targetPos = new Pose2D(
                    DistanceUnit.MM,
                    ((targetX >= 0.1 || targetX <= -0.1) ? robotPosition.getX(DistanceUnit.MM) + 10 * targetX : targetPos.getX(DistanceUnit.MM)),
                    ((targetY >= 0.1 || targetY <= -0.1) ? robotPosition.getY(DistanceUnit.MM) + 10 * targetY : targetPos.getY(DistanceUnit.MM)),
                    AngleUnit.DEGREES,
                    ((rotate >= 0.1 || rotate <= -0.1) ? robotPosition.getHeading(AngleUnit.DEGREES) + 5 * rotate : targetPos.getHeading(AngleUnit.DEGREES))
            );
            goTo(targetPos.getY(DistanceUnit.MM), targetPos.getX(DistanceUnit.MM), targetPos.getHeading(AngleUnit.DEGREES));
    }

    // Run one time when STOP is pressed
    @Override
    public void stop() {
    }
    // =============================================================================================
    // End system function definitions
    // =============================================================================================

    // =============================================================================================
    // Helper function definitions
    // =============================================================================================

    public void drive(double axial, double lateral, double yaw, double power)
    {
        // Convert to field-centric if enabled
        if (false) {
            double botHeading = -getHeading(AngleUnit.RADIANS);

            double rotatedX = lateral * Math.cos(botHeading) - axial * Math.sin(botHeading);
            double rotatedY = lateral * Math.sin(botHeading) + axial * Math.cos(botHeading);

            lateral = rotatedX;
            axial = rotatedY;
        }

        // Normalize powers
        double denominator = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1);
        double leftFrontPower  = (axial + lateral + yaw) / denominator;
        double rightFrontPower = (axial - lateral - yaw) / denominator;
        double leftRearPower   = (axial - lateral + yaw) / denominator;
        double rightRearPower  = (axial + lateral - yaw) / denominator;

        // Apply scaled power
        leftFrontDrive.setPower(leftFrontPower * power);
        leftRearDrive.setPower(leftRearPower * power);
        rightFrontDrive.setPower(rightFrontPower * power);
        rightRearDrive.setPower(rightRearPower * power);
    }

    public void goTo(double yLocation, double xLocation, double headingDegree) {
        double xDistance = xLocation - getXPosition(DistanceUnit.INCH);
        double yDistance = yLocation - getYPosition(DistanceUnit.INCH);

        double negativeRadianHeading = -getHeading(AngleUnit.RADIANS);

        double rotatedX = xDistance * Math.cos(negativeRadianHeading) - yDistance * Math.sin(negativeRadianHeading);
        double rotatedY = xDistance * Math.sin(negativeRadianHeading) + yDistance * Math.cos(negativeRadianHeading);

        double axialPower = driveController.getOutputFromError(rotatedX);
        double lateralPower = strafeController.getOutputFromError(rotatedY);
        double yawPower = yawController.getOutput(headingDegree);

        drive(axialPower, -lateralPower, -yawPower, 1);
    }

    // Update intake power
    public void handleIntake() {
        intakePower = (gamepad1.right_trigger + gamepad2.right_trigger) - (gamepad1.left_trigger + gamepad2.left_trigger);

        intake.setPower(intakePower);
        leftIntakeServo.setPower(intakePower);
        rightIntakeServo.setPower(intakePower);
    }

    // Display status and data on driver station
    public void displayTelemetry() {
        // Odometry
        Pose2D pos = odo.getPosition();
        String data = String.format(Locale.US, "X: %.3f, Y: %.3f, H: %.3f", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position: ", data);

        String velocity = String.format(Locale.US,"X Vel: %.3f, Y Vel: %.3f, Heading Vel: %.3f", odo.getVelX(DistanceUnit.MM), odo.getVelY(DistanceUnit.MM), odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
        telemetry.addData("Velocities: ", velocity);

        // Drive status
        telemetry.addData("Status", odo.getDeviceStatus());

        telemetry.addData("Target Launch Speed", launcher_target_velocity);
        telemetry.addData("Minimum Launcher Speed", (launcher_target_velocity - LAUNCHER_MIN_VELOCITY));
        telemetry.addData("Actual launcher speed", launcher.getVelocity());

        telemetry.addData("Motors", "Left Front (%.2f), Right Front (%.2f), Left Rear (%.2f), Right Rear (%.2f)", leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
        telemetry.addData("Triggers", "left (%.2f, right (%.2f)",gamepad1.left_trigger, gamepad1.right_trigger);

    }

    // Handle top controller buttons
    public void controllerButtons() {
        // Handle field centric enable/disable buttons
        if (gamepad1.squareWasPressed() || gamepad2.squareWasPressed()) { // Square looks like field
            field_centric_drive = true;
        } else if (gamepad1.triangleWasPressed() || gamepad2.triangleWasPressed()) { // Triangle is messy like bot
            field_centric_drive = false;
        }

        // Reset position or recalibrate
        if (gamepad1.circleWasPressed() || gamepad2.circleWasPressed()) {
            odo.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
            targetPos = odo.getPosition();
        }
        if (gamepad1.crossWasPressed() || gamepad2.crossWasPressed()) {
            launches = 0;
        }

        if (gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed()) {
            launcher_target_velocity += 5;
        } else if (gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed()) {
            launcher_target_velocity -= 5;
        }

        if (gamepad1.dpad_left || gamepad2.dpad_left) {
            windmillServo.setPower(-1);
        }
        if (gamepad1.dpad_right || gamepad2.dpad_right) {
            windmillServo.setPower(1);
        }
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
        if (lastLaunchMillis.milliseconds() > LAUNCHER_TIMEOUT && launches > 0) {
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

    public void updatePosition() {
        odo.update();
        dBoardTelemetry.update();
        robotPosition = odo.getPosition();
    }

    // =============================================================================================
    // End helper function definitions
    // =============================================================================================
}