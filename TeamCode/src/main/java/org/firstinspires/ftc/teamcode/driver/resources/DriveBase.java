package org.firstinspires.ftc.teamcode.driver.resources;


import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Simplistic drive base capable of using mecanum wheels and driving around
 */
public class DriveBase
{
    protected LinearOpMode myOpMode = null;

    protected boolean fieldCentric = false;

    protected DcMotor leftFrontDrive = null;
    protected DcMotor leftRearDrive = null;
    protected DcMotor rightFrontDrive = null;
    protected DcMotor rightRearDrive = null;

    protected IMU imu = null;
    public DriveBase(LinearOpMode opMode, boolean isFC)
    {
        myOpMode = opMode;
        fieldCentric = isFC;
    }

    /**
     * Initializes the motors and sets their direction. Can also add other default settings.
     *  - Set Mode: run with/without encoders
     *  - Zero power behavior
     */
    public void init()
    {

        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_front_drive");
        leftRearDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_front_drive");
        rightRearDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_back_drive");


        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);


        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftRearDrive.setZeroPowerBehavior(BRAKE);
        rightRearDrive.setZeroPowerBehavior(BRAKE);


        setupOdometry();

        myOpMode.telemetry.addData("Status","Initialized");
        myOpMode.telemetry.update();

    }

    protected void setupOdometry()
    {
        // TODO: Update this based on how the hub is mounted on the robot
        imu = myOpMode.hardwareMap.get(IMU.class,"imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu.initialize(parameters);
    }

    protected double getHeading(AngleUnit unit)
    {
        return imu.getRobotYawPitchRollAngles().getYaw(unit);
    }

    public void drive(double axial, double lateral, double yaw)
    {
        drive(axial,lateral,yaw,1);
    }
    /**
     * Standard POV Mecanum drive code
     * @param axial left joystick y value
     * @param lateral left joystick x value
     * @param yaw right joystick x value
     */
    public void drive(double axial, double lateral, double yaw, double power)
    {
        // Convert to field-centric if enabled
        if (fieldCentric) {
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

        // Telemetry
        myOpMode.telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        myOpMode.telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftRearPower, rightRearPower);
        myOpMode.telemetry.addData("Heading", getHeading(AngleUnit.DEGREES));
    }



    /**  For toggling FC  **/
    public void toggleFC(){
        if(fieldCentric) fieldCentric = false;

        else fieldCentric = true;

    }
}
