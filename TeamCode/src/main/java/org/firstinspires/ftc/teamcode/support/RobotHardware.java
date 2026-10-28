package org.firstinspires.ftc.teamcode.support;

import static org.firstinspires.ftc.teamcode.support.ConstantsPID.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class RobotHardware {
    // Keep a reference to the active OpMode for loop safety and telemetry
    private final LinearOpMode opMode;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    // Drivetrain
    private DcMotor leftFrontDrive = null;
    private DcMotor leftRearDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightRearDrive = null;

    // DC Motors & Servos
    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;

    // Odometry Driver
    public GoBildaPinpointDriver odo;
    private Pose2D robotPosition = null;

    // PID Controllers
    private final PIDController driveController = new PIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_MAX_AUTO, DRIVE_TOLERANCE, DRIVE_DEADBAND, false);
    private final PIDController strafeController = new PIDController(STRAFE_KP, STRAFE_KI, STRAFE_KD, STRAFE_MAX_AUTO, STRAFE_TOLERANCE, STRAFE_DEADBAND, false);
    private final PIDController yawController = new PIDController(YAW_KP, YAW_KI, YAW_KD, YAW_MAX_AUTO, YAW_TOLERANCE, YAW_DEADBAND, true);

    // Constructor: Takes the active OpMode to access its hardwareMap and telemetry
    public RobotHardware(LinearOpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
    }

    // Initialize all hardware components
    public void init() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Odo initialization
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();
        odo.update();

        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftRearDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // DO NOT FORGET THIS!!!!!!!!!!
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);

        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");

        updatePositionAndTelemetry();
    }

    // --- Simple Helpers ---
    public double getXPosition(DistanceUnit unit) { return robotPosition != null ? robotPosition.getX(unit) : 0; }
    public double getYPosition(DistanceUnit unit) { return robotPosition != null ? robotPosition.getY(unit) : 0; }
    public double getHeading(AngleUnit unit) { return robotPosition != null ? robotPosition.getHeading(unit) : 0; }

    public void intake(double power) {
        intake.setPower(power);
        leftIntakeServo.setPower(power);
        rightIntakeServo.setPower(power);
    }

    public void testDrivetrain() throws InterruptedException {
        leftFrontDrive.setPower(1);
        Thread.sleep(1000);
        leftFrontDrive.setPower(0);

        rightFrontDrive.setPower(1);
        Thread.sleep(1000);
        rightFrontDrive.setPower(0);

        leftRearDrive.setPower(1);
        Thread.sleep(1000);
        leftRearDrive.setPower(0);

        rightRearDrive.setPower(1);
        Thread.sleep(1000);
        rightRearDrive.setPower(0);
    }

    // --- Core Action Methods ---
    public void drive(double axial, double lateral, double yaw) {

        // Make field-centric
        Pose2D pos = odo.getPosition();
        double heading = pos.getHeading(AngleUnit.RADIANS);

        // Get new speeds
        double strafe = lateral * Math.cos(-heading) - axial * Math.sin(-heading);
        double forward =  lateral * Math.sin(-heading) + axial * Math.cos(-heading);

        // Counteract imperfect strafing
        forward = forward * 1.1;

        // Fit range
        double denominator = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1);
        double leftFrontPower  = (forward + strafe + yaw) / denominator;
        double rightFrontPower = (forward - strafe - yaw) / denominator;
        double leftRearPower   = (forward - strafe + yaw) / denominator;
        double rightRearPower  = (forward + strafe - yaw) / denominator;

        // Do something
        leftFrontDrive.setPower(leftFrontPower);
        leftRearDrive.setPower(leftRearPower);
        rightFrontDrive.setPower(rightFrontPower);
        rightRearDrive.setPower(rightRearPower);
    }

    public int launch(int launches, double timeoutMillis) {
        boolean reachedTarget = false;
        boolean subtracted = false;
        ElapsedTime lastLaunchMillis = new ElapsedTime();
        lastLaunchMillis.reset();

        if (launches > 0) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else {
            launcher.setVelocity(0);
        }

        while (opMode.opModeIsActive() && launches > 0) {
            if (lastLaunchMillis.time() > timeoutMillis) {
                windmillServo.setPower(0);
                intake(0);
                launcher.setVelocity(0);
                return launches;
            }

            if (launcher.getVelocity() > (LAUNCHER_TARGET_VELOCITY - LAUNCHER_MIN_VELOCITY)) {
                reachedTarget = true;
                subtracted = false;
            }

            if (launcher.getVelocity() < (LAUNCHER_TARGET_VELOCITY - LAUNCHER_MIN_VELOCITY) && reachedTarget && !subtracted) {
                launches -= 1;
                subtracted = true;
                lastLaunchMillis.reset();
            }

            if (launches > 0 && launcher.getVelocity() > (LAUNCHER_TARGET_VELOCITY - LAUNCHER_MIN_VELOCITY)) {
                windmillServo.setPower(1);
                intake(0.5);
            }
        }
        windmillServo.setPower(0);
        launcher.setVelocity(0);
        intake(0);
        return 0;
    }

    public void goTo(double yLocation, double xLocation, double headingDegree, double power, double holdTime, double pathTime) {
        ElapsedTime pathTimer = new ElapsedTime();
        ElapsedTime holdTimer = new ElapsedTime();

        driveController.reset(yLocation, power);
        strafeController.reset(xLocation, power);
        yawController.reset(headingDegree, power);
        pathTimer.reset();

        boolean exit = false;

        while (opMode.opModeIsActive() && !exit) {
            updatePositionAndTelemetry();

            double xDistance = xLocation - getXPosition(DistanceUnit.INCH);
            double yDistance = yLocation - getYPosition(DistanceUnit.INCH);

            // Rotate odometry input left 90 degrees
            double tempXDistance = xDistance;
            xDistance = yDistance;
            yDistance = tempXDistance;

            double axialPower = driveController.getOutputFromError(yDistance);
            double lateralPower = strafeController.getOutputFromError(xDistance);
            double yawPower = yawController.getOutput(getHeading(AngleUnit.DEGREES));

            drive(axialPower, -lateralPower, -yawPower);

            if (pathTimer.seconds() > pathTime) {
                exit = true;
            }

            if (driveController.isInPosition() && strafeController.isInPosition() && yawController.isInPosition()) {
                if (holdTimer.time() > holdTime) exit = true;
            } else {
                holdTimer.reset();
            }
        }
        drive(0, 0, 0);
    }

    public void updatePositionAndTelemetry() {
        odo.update();
        robotPosition = odo.getPosition();

        telemetry.addData("Y-Position", odo.getPosY(DistanceUnit.MM));
        telemetry.addData("X-Position", odo.getPosX(DistanceUnit.MM));
        telemetry.addData("Heading Deg", odo.getHeading(AngleUnit.DEGREES));
        telemetry.update();
    }
}
