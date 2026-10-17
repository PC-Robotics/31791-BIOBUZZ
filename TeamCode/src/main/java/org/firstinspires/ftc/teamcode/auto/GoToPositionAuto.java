package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.driver.support.ConstantsPID.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.driver.support.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.driver.support.PIDController;

@Autonomous(name="Go to Position Auto v1", group = "StarterBot")
public class GoToPositionAuto extends LinearOpMode { // Changed to LinearOpMode
    // Drivetrain
    private DcMotor leftFrontDrive = null;
    private DcMotor leftRearDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightRearDrive = null;

    // DC Motors
    private DcMotorEx launcher = null;
    private DcMotor intake = null;

    // Servos
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;

    // PID Controllers
    private final PIDController driveController = new PIDController(DRIVE_KP,DRIVE_KI,DRIVE_KD,DRIVE_MAX_AUTO,DRIVE_TOLERANCE,DRIVE_DEADBAND,false);
    private final PIDController strafeController = new PIDController(STRAFE_KP,STRAFE_KI,STRAFE_KD,STRAFE_MAX_AUTO,STRAFE_TOLERANCE,STRAFE_DEADBAND, false);
    private final PIDController yawController = new PIDController(YAW_KP,YAW_KI,YAW_KD,YAW_MAX_AUTO,YAW_TOLERANCE,YAW_DEADBAND,true);

    // Global Variables
    private Pose2D robotPosition = null;
    private final boolean fieldCentric = true;
    public int launcher_target_velocity = 1265;
    public final int LAUNCHER_MIN_VELOCITY = 50;
    GoBildaPinpointDriver odo;

    private double getHeading(AngleUnit unit) { return robotPosition != null ? robotPosition.getHeading(unit) : 0; }
    private void intake(double power) { intake.setPower(power); leftIntakeServo.setPower(power); rightIntakeServo.setPower(power); }

    @Override
    public void runOpMode() throws InterruptedException {
        // Connect devices from Hardware Map
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftRearDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");

        updatePositionAndTelemetry();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            goToPosition(20, 20, 170, 1, 0.5, 10);

            int fails = launch(2, 2000);

            telemetry.addData("Fails", fails);
            telemetry.update();

            goToPosition(0, 0, 0, 0.5, 1, 5);

            // LinearOpMode handles stopping automatically here without a while(true) freeze
        }
    }

    private void updateTelemetry() {
        odo.update();
        telemetry.addData("Y-Position", odo.getPosY(DistanceUnit.MM));
        telemetry.addData("X-Position", odo.getPosX(DistanceUnit.MM));
        telemetry.addData("Heading Deg", odo.getHeading(AngleUnit.DEGREES));
    }

    private void drive(double axial, double lateral, double yaw) {
        if (fieldCentric) {
            double botHeading = -getHeading(AngleUnit.RADIANS);
            double rotatedX = lateral * Math.cos(botHeading) - axial * Math.sin(botHeading);
            double rotatedY = lateral * Math.sin(botHeading) + axial * Math.cos(botHeading);
            lateral = rotatedX;
            axial = rotatedY;
        }

        double denominator = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1);
        double leftFrontPower  = (axial + lateral + yaw) / denominator;
        double rightFrontPower = (axial - lateral - yaw) / denominator;
        double leftRearPower   = (axial - lateral + yaw) / denominator;
        double rightRearPower  = (axial + lateral - yaw) / denominator;

        leftFrontDrive.setPower(leftFrontPower);
        leftRearDrive.setPower(leftRearPower);
        rightFrontDrive.setPower(rightFrontPower);
        rightRearDrive.setPower(rightRearPower);
    }

    private int launch(int launches, double timeoutMillis) {
        boolean reachedTarget = false;
        boolean subtracted = false;
        ElapsedTime lastLaunchMillis = new ElapsedTime();

        launcher.setVelocity((launches > 0) ? launcher_target_velocity : 0);

        while (opModeIsActive() && launches > 0) {
            if (lastLaunchMillis.time() > timeoutMillis) {
                windmillServo.setPower(0);
                intake(0);
                launcher.setVelocity(0);
                return launches;
            }

            if (launcher.getVelocity() > (launcher_target_velocity - LAUNCHER_MIN_VELOCITY)) {
                reachedTarget = true;
                subtracted = false;
            }

            if (launcher.getVelocity() < (launcher_target_velocity - LAUNCHER_MIN_VELOCITY) && reachedTarget && !subtracted) {
                launches -= 1;
                subtracted = true;
                lastLaunchMillis.reset();
            }

            if (launches > 0 && launcher.getVelocity() > (launcher_target_velocity - LAUNCHER_MIN_VELOCITY)) {
                windmillServo.setPower(1);
                intake(0.5);
            } else if (!gamepad1.left_bumper) {
                windmillServo.setPower(0);
            }
        }
        windmillServo.setPower(0);
        launcher.setVelocity(0);
        intake(0);
        return 0;
    }

    private void goToPosition(double yLocation, double xLocation, double headingDegree, double power, double holdTime, double pathTime) {
        ElapsedTime pathTimer = new ElapsedTime();
        ElapsedTime holdTimer = new ElapsedTime();

        driveController.reset(yLocation, power);
        strafeController.reset(xLocation, power);
        yawController.reset(headingDegree, power);
        pathTimer.reset();

        boolean exit = false;

        // Added opModeIsActive() safety check
        while (opModeIsActive() && !exit) {
            updatePositionAndTelemetry();

            // Calculate simple linear distance errors
            double xDistance = xLocation - robotPosition.getX(DistanceUnit.INCH);
            double yDistance = yLocation - robotPosition.getY(DistanceUnit.INCH);

            // Pass un-rotated raw errors into your controllers since the drive() method converts them to field-centric
            double axialPower = driveController.getOutputFromError(yDistance);
            double lateralPower = strafeController.getOutputFromError(xDistance);
            double yawPower = yawController.getOutput(getHeading(AngleUnit.DEGREES));

            drive(axialPower, -lateralPower, -yawPower);

            if (pathTimer.seconds() > pathTime) {
                exit = true;
            }

            if(driveController.isInPosition() && strafeController.isInPosition() && yawController.isInPosition()) {
                if(holdTimer.time() > holdTime) exit = true;
            } else {
                holdTimer.reset();
            }
        }

        drive(0,0,0);
    }

    private void updatePositionAndTelemetry() {
        odo.update();
        robotPosition = odo.getPosition();
        updateTelemetry();
    }
}
