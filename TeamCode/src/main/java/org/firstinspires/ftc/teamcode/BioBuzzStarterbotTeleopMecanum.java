/*   MIT License
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

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.support.GoBildaPinpointDriver;

import java.util.Locale;

@TeleOp(name = "CLICK ME: StarterBot Teleop (main)", group = "StarterBot")
//@Disabled

public class BioBuzzStarterbotTeleopMecanum extends OpMode {
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


    // Define drive motor power in entire class for telemetry
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;
    double intakePower;
    boolean field_centric_drive = true;

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer


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
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);


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
    }

    // Run repeatedly in START mode
    @Override
    public void loop() {
        // Please do ***NOT*** put any code other than function calls here.
        odo.update();
        fieldCentricDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        launch();
        handleIntake();
        displayTelemetry();
        controllerButtons();
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

    // Update intake power
    public void handleIntake() {
        intakePower = gamepad1.right_trigger - gamepad1.left_trigger;

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

        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftFrontPower, rightFrontPower);
        telemetry.addData("Triggers", "left (%.2f, right (%.2f)",gamepad1.left_trigger, gamepad1.right_trigger);

    }

    // Handle top controller buttons
    public void controllerButtons() {
        // Handle field centric enable/disable buttons
        if (gamepad1.squareWasPressed()) { // Square looks like field
            field_centric_drive = true;
        } else if (gamepad1.triangleWasPressed()) { // Triangle is messy like bot
            field_centric_drive = false;
        }

        // Reset position or recalibrate
        if (gamepad1.circleWasPressed()) {
            odo.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
        } else if (gamepad1.crossWasPressed()) {
            odo.recalibrateIMU(); //recalibrates the IMU without resetting position
        }
    }

    // Call this function with drive commands and enable or disable field_centric_drive
    void fieldCentricDrive(double forward, double strafe, double rotate) {
        if (field_centric_drive) {
            // Get position in RADIANS for Math.sin
            Pose2D pos = odo.getPosition();
            double heading = pos.getHeading(AngleUnit.RADIANS);

            // Get new speeds
            double newStrafe = strafe * Math.cos(-heading) - forward * Math.sin(-heading);
            double newForward =  strafe * Math.sin(-heading) + forward * Math.cos(-heading);

            // Counteract imperfect strafing
            newForward = newForward * 1.1;

            // Drive accordingly
            mecanumDrive(newForward, newStrafe, rotate);
        } else {
            // Drive regularly
            mecanumDrive(forward, strafe, rotate);
        }
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

    // Call in loop to handle launcher functions
    void launch() {

        // Start or stop launcher based on bumper
        if (gamepad1.right_bumper) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else {
            launcher.setVelocity(0);
        }

        // Once the launcher gets up to speed, feed balls and slowly run intake to dislodge
        // stuck balls
        if (gamepad1.right_bumper && launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
            windmillServo.setPower(1);
            intakePower += 0.5;
        } else {
            windmillServo.setPower(0);
        }
    }

    // =============================================================================================
    // End helper function definitions
    // =============================================================================================
}