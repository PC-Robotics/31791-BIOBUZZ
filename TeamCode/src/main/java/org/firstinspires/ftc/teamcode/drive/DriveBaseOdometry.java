package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.support.GoBildaPinpointDriver;

import java.util.Locale;

public class DriveBaseOdometry extends DriveBase
{

    private GoBildaPinpointDriver odo;

    private Pose2D robotPosition;

    public static Pose2D savedPose = null;



    public DriveBaseOdometry(LinearOpMode opMode, boolean isFC)
    {
        super(opMode, isFC);
    }

    public void init()
    {
        super.init();
    }

    @Override
    protected void setupOdometry()
    {
        // Initialize the hardware map using the same name that is used on the driver station
        odo = myOpMode.hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        // TODO: Set the appropriate offets based on your build
        odo.setOffsets(180.5, 48.0); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per mm of your odometry pod.
         */
        //TODO: Choose the appropriate pod type
        //For Starter Bot it is goBILDA_4_BAR_POD
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odo.setEncoderResolution(13.26291192);


        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        //TODO: Run the simple odometry teleop and pay attention to telemetry values to make sure the directions are set correctly
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED );

        resetPositionAndOdometry();

        odo.getPosition();




        myOpMode.telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
        myOpMode.telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
        myOpMode.telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        myOpMode.telemetry.addData("Device Scalar", odo.getYawScalar());

        robotPosition = odo.getPosition();
    }

    public void updatePositionAndTelemetry()
    {
        updatePosition();
        updateOdometryTelemetry();
    }

    /**
     * Updates the position of the robot based on the odometry pod readings
     */
    public void updatePosition()
    {
        odo.update();
        robotPosition = odo.getPosition();
    }

    public void updateOdometryTelemetry()
    {
        /*
        Prints the current Position (x & y in inches, and heading in degrees) of the robot
        Can alter the DistanceUnit to display MM or INCH
        */
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", robotPosition.getX(DistanceUnit.INCH),
                robotPosition.getY(DistanceUnit.INCH), robotPosition.getHeading(AngleUnit.DEGREES));
        myOpMode.telemetry.addData("Position", data);

        /*
        gets the current Velocity (x & y in in/sec and heading in degrees/sec) and prints it.
        */
        Pose2D vel = odo.getVelocity();
        String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.INCH), vel.getY(DistanceUnit.INCH), vel.getHeading(AngleUnit.DEGREES));
        myOpMode.telemetry.addData("Velocity", velocity);

         /*
        Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll primarily see
        READY: the device is working as normal
        CALIBRATING: the device is calibrating and outputs are put on hold
        NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
        FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
        FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
        FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in
        */
        myOpMode.telemetry.addData("Status", odo.getDeviceStatus());

        myOpMode.telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint

    }

    public void resetPositionAndOdometry()
    {
        odo.resetPosAndIMU();
    }

    public void setRobotPosition(Pose2D position)
    {
        odo.setPosition(position);
    }

    public Pose2D getRobotPosition()
    {
        return robotPosition;
    }

    public double getXPosition(DistanceUnit unit)
    {
        return robotPosition.getX(unit);
    }
    public double getYPosition(DistanceUnit unit)
    {
        return robotPosition.getY(unit);
    }
    @Override
    public double getHeading(AngleUnit unit)
    {
        return robotPosition.getHeading(unit);
    }

    public void saveCurrentPose(){
        savedPose = getRobotPosition();
    }

    public void restoreSavedPoseIfAvailable() {
        if (savedPose != null) {
            setRobotPosition(savedPose);
        }
    }
}
