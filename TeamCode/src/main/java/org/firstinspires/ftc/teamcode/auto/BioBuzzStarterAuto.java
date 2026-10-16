package org.firstinspires.ftc.teamcode.auto;/*   MIT License
 *   Copyright (c) [2026] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.driver.support.GoBildaPinpointDriver;

import java.util.Locale;
import java.util.Vector;

@Autonomous(name = "Last resort: move back/forth", group = "StarterBot")
//@Disabled

public class BioBuzzStarterAuto extends OpMode {
    // CLASS VARIABLES =============================================================================
    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;


    // Set target and min velocity of flywheel for launching (ticks per second)
    public final int LAUNCHER_TARGET_VELOCITY = 1250; //2678 RPM
    public final int LAUNCHER_MIN_VELOCITY = 1200; //2571 RPM
    public final int LAUNCHER_TIMEOUT = 5000;


    // Define drive motor power in entire class for telemetry
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;


    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer


    // =============================================================================================
    // SYSTEM FUNCTION DEFINITIONS - Please don't put actual code in functions except init()
    // =============================================================================================

    // init() runs once after INIT is pressed
    @Override
    public void init() {
        // SET UP HARDWARE MAPS ====================================================================
        // Sensors
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Actuators
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");


        // INITIALIZE ACTUATORS ====================================================================
        // Reverse left motors because robots
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Brake when not powered
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
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
        // Set odo offsets REQUIRES CHANGING________________________________________________________
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);

        // Odo initialization
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);


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
        odo.update();
        driveFwdAndBack();
    }

    // Run repeatedly in START mode
    @Override
    public void loop() {
//        // Please do ***NOT*** put any code other than function calls here.
//        fieldCentricDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
//        launch();
//        handleIntake();
//        displayTelemetry();
//        controllerButtons();

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

    public void monitorPosition() {
        Pose2D pos = odo.getPosition();
        telemetry.addData("Position x: ", pos.getX(DistanceUnit.CM));
        telemetry.addData("Position y: ", pos.getY(DistanceUnit.CM));
        telemetry.update();
    }

    public void move(double x, double y) {
        Pose2D pos = odo.getPosition();
        double finalX = Math.abs(pos.getX(DistanceUnit.CM) + x);
        double finalY = Math.abs(pos.getY(DistanceUnit.CM) + y);
        double distance = Math.sqrt(x * x + y * y);
        double xMultiplier = x / distance;
        double yMultiplier = y / distance;
        while (Math.abs(pos.getX(DistanceUnit.CM)) < finalX) {
            mecanumDrive(0.3 * xMultiplier, 0.3 * yMultiplier, 0);
            odo.update();
            pos = odo.getPosition();
            monitorPosition();
        }
        while (Math.abs(pos.getX(DistanceUnit.CM)) < finalY) {
            mecanumDrive(0, 0.3, 0);
            odo.update();
            pos = odo.getPosition();
            monitorPosition();
        }
    }

    public void driveFwdAndBack() {
//        move(100, 0);
//        move(0,50);
//        move(-50,0);

        monitorPosition();

        Pose2D pos = odo.getPosition();
        while (pos.getX(DistanceUnit.CM) < 7) {
            mecanumDrive(0.3, 0, 0);
            odo.update();
            pos = odo.getPosition();
            monitorPosition();
        }
        mecanumDrive(0, 0, 0);
        while (pos.getY(DistanceUnit.CM) < 50) {
            mecanumDrive(0, -0.3, 0);
            odo.update();
            pos = odo.getPosition();
            monitorPosition();
        }
        mecanumDrive(0, 0, 0);
    }

    // Use this ONLY for driving robot directly
    void mecanumDrive(double forward, double strafe, double rotate) {
        // Get wheel speeds
        leftFrontPower = forward + strafe + rotate;
        rightFrontPower = forward - strafe - rotate;
        leftBackPower = forward - strafe + rotate;
        rightBackPower = forward + strafe - rotate;

        // Check highest wheel speed to avoid running all at 100% and not turning
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        // Limit results
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }
}