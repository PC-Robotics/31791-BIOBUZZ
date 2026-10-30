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
import org.firstinspires.ftc.teamcode.drive.DriveBasePID;

/**
 * Simple TeleOp OpMode combining:
 * 1) DriveBasePID position lock system (automatic hold on joystick release)
 * 2) Launcher mechanism with flywheel speed control & feed
 * 3) Intake mechanism
 */
@TeleOp(name = "Position Lock TeleOp (Launcher + Intake)", group = "Driver")
public class BioBuzzPositionLockTeleOp extends LinearOpMode {

    public enum DriveState {
        DRIVE,
        POSITION_LOCK
    }

    // Hardware
    private DriveBasePID robot;
    private DcMotorEx launcher;
    private DcMotor intake;
    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;
    private CRServo windmillServo;

    // Driving & Position Lock state
    private DriveState driveState = DriveState.DRIVE;
    private double lockX = 0;
    private double lockY = 0;

    // Launcher & Intake state
    public int launcher_target_velocity = 1265;
    public final int LAUNCHER_MIN_VELOCITY = 50;
    public final int LAUNCHER_TIMEOUT = 5000;
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

        telemetry.addData("Status", "Initialized Position Lock TeleOp");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            handleDrive();
            handleIntake();
            launch();
            handleControllerButtons();

            robot.updatePositionAndTelemetry();

            // Telemetry
            telemetry.addData("Drive State", driveState);
            telemetry.addData("Lock X (in)", "%.2f", lockX);
            telemetry.addData("Lock Y (in)", "%.2f", lockY);
            telemetry.addData("Queued Launches", launches);
            telemetry.addData("Launcher Speed Target/Actual", "%d / %.0f", launcher_target_velocity, launcher.getVelocity());
            telemetry.update();
        }
    }

    private void handleDrive() {
        double axial = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yawInput = gamepad1.right_stick_x;

        // Manual state switch
        if (gamepad1.triangleWasPressed()) {
            Pose2D pose = robot.getRobotPosition();
            lockX = pose.getX(DistanceUnit.INCH);
            lockY = pose.getY(DistanceUnit.INCH);

            robot.setLockX(lockX);
            robot.setLockY(lockY);
            robot.setLockHeading(pose.getHeading(AngleUnit.DEGREES));

            driveState = DriveState.POSITION_LOCK;
        }

        if (gamepad1.crossWasPressed()) {
            driveState = DriveState.DRIVE;
        }

        // Joystick detection for automatic transition between DRIVE and POSITION_LOCK
        boolean sticksActive = Math.abs(axial) > 0.01 || Math.abs(lateral) > 0.01 || Math.abs(yawInput) > 0.01;

        if (sticksActive) {
            driveState = DriveState.DRIVE;
        } else if (!sticksActive && driveState == DriveState.DRIVE) {
            Pose2D pose = robot.getRobotPosition();
            lockX = pose.getX(DistanceUnit.INCH);
            lockY = pose.getY(DistanceUnit.INCH);

            robot.setLockX(lockX);
            robot.setLockY(lockY);
            robot.setLockHeading(pose.getHeading(AngleUnit.DEGREES));

            driveState = DriveState.POSITION_LOCK;
        }

        // Execute state machine
        switch (driveState) {
            case POSITION_LOCK:
                robot.goToPositionNonBlocking(
                        lockY,
                        lockX,
                        robot.getHeading(AngleUnit.DEGREES),
                        0.7
                );
                break;

            case DRIVE:
                robot.drive(axial, lateral, yawInput, 1);
                break;
        }
    }

    public void handleIntake() {
        intakePower = (gamepad1.right_trigger + gamepad2.right_trigger) - (gamepad1.left_trigger + gamepad2.left_trigger);

        intake.setPower(intakePower);
        leftIntakeServo.setPower(intakePower);
        rightIntakeServo.setPower(intakePower);
    }

    void launch() {
        if (gamepad1.rightBumperWasPressed() || gamepad2.rightBumperWasPressed()) {
            launches += 1;
            lastLaunchMillis.reset();
        }

        if (launches > 0) {
            launcher.setVelocity(launcher_target_velocity);
        } else {
            launcher.setVelocity(0);
            reachedTarget = false;
        }

        if (lastLaunchMillis.time() > LAUNCHER_TIMEOUT && launches > 0) {
            launches = 0;
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
}
