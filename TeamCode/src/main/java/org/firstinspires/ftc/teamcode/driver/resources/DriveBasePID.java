package org.firstinspires.ftc.teamcode.driver.resources;

import static org.firstinspires.ftc.teamcode.driver.support.ConstantsPID.*;
import static org.firstinspires.ftc.teamcode.driver.support.ConstantsPID.DRIVE_DEADBAND;
import static org.firstinspires.ftc.teamcode.driver.support.ConstantsPID.DRIVE_KD;
import static org.firstinspires.ftc.teamcode.driver.support.ConstantsPID.DRIVE_KI;
import static org.firstinspires.ftc.teamcode.driver.support.ConstantsPID.DRIVE_KP;
import static org.firstinspires.ftc.teamcode.driver.support.ConstantsPID.DRIVE_MAX_AUTO;
import static org.firstinspires.ftc.teamcode.driver.support.ConstantsPID.DRIVE_TOLERANCE;
import static org.firstinspires.ftc.teamcode.driver.support.ConstantsPID.STRAFE_DEADBAND;
import static org.firstinspires.ftc.teamcode.driver.support.ConstantsPID.STRAFE_KD;
import static org.firstinspires.ftc.teamcode.driver.support.ConstantsPID.STRAFE_KI;
import static org.firstinspires.ftc.teamcode.driver.support.ConstantsPID.STRAFE_KP;
import static org.firstinspires.ftc.teamcode.driver.support.ConstantsPID.STRAFE_MAX_AUTO;
import static org.firstinspires.ftc.teamcode.driver.support.ConstantsPID.STRAFE_TOLERANCE;
import static org.firstinspires.ftc.teamcode.driver.support.ConstantsPID.YAW_DEADBAND;
import static org.firstinspires.ftc.teamcode.driver.support.ConstantsPID.YAW_KD;
import static org.firstinspires.ftc.teamcode.driver.support.ConstantsPID.YAW_KI;
import static org.firstinspires.ftc.teamcode.driver.support.ConstantsPID.YAW_KP;
import static org.firstinspires.ftc.teamcode.driver.support.ConstantsPID.YAW_MAX_AUTO;
import static org.firstinspires.ftc.teamcode.driver.support.ConstantsPID.YAW_TOLERANCE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.driver.support.PIDController;

public class DriveBasePID extends DriveBaseOdometry
{
    private ElapsedTime holdTimer = new ElapsedTime();
    private ElapsedTime pathTimer = new ElapsedTime();
    private PIDController driveController = new PIDController(DRIVE_KP,DRIVE_KI,DRIVE_KD,DRIVE_MAX_AUTO,DRIVE_TOLERANCE,DRIVE_DEADBAND,false);
    private PIDController strafeController = new PIDController(STRAFE_KP,STRAFE_KI,STRAFE_KD,STRAFE_MAX_AUTO,STRAFE_TOLERANCE,STRAFE_DEADBAND, false);
    private PIDController yawController = new PIDController(YAW_KP,YAW_KI,YAW_KD,YAW_MAX_AUTO,YAW_TOLERANCE,YAW_DEADBAND,true);

    private double xPosition;
    private double yPosition;
    private double heading;

    private double targetX;
    private double targetY;
    private double targetHeading;
    private double desiredX;
    private double desiredY;
    private double desiredHeading;

    private boolean holdingPosition = false;
    ElapsedTime loopTimer = new ElapsedTime();




    public DriveBasePID(LinearOpMode mode, boolean isFC)
    {
        super(mode,isFC);
    }

    public void init()
    {
        super.init();

        addLocationToTelemetry();
        myOpMode.telemetry.update();
    }

    public void forward(double distanceInches, double power, double holdTime)
    {
        targetX = getXPosition(DistanceUnit.INCH)+distanceInches;
        targetY = getYPosition(DistanceUnit.INCH);
        targetHeading = getHeading(AngleUnit.DEGREES);

        driveController.reset(targetX,power);
        strafeController.reset(targetY);
        yawController.reset(targetHeading);
        holdTimer.reset();

        while (myOpMode.opModeIsActive())
        {
            updatePositionAndTelemetry();

            xPosition = getXPosition(DistanceUnit.INCH);
            yPosition = getYPosition(DistanceUnit.INCH);
            heading = getHeading(AngleUnit.DEGREES);

            double axial = driveController.getOutput(xPosition);
            double lateral = -strafeController.getOutput(yPosition);
            double yaw = -yawController.getOutput(heading);

            addLocationToTelemetry();

            drive(axial,lateral,yaw,power);

            myOpMode.telemetry.update();

            if(driveController.isInPosition() && yawController.isInPosition())
            {
                if(holdTimer.time() > holdTime)  break;
            }
            else holdTimer.reset();

            myOpMode.sleep(10);
        }

        drive(0,0,0);
    }

    public void strafe(double distanceInches, double power, double holdTime)
    {
        targetX = getXPosition(DistanceUnit.INCH);
        targetY = getYPosition(DistanceUnit.INCH)+distanceInches;
        targetHeading = getHeading(AngleUnit.DEGREES);




        driveController.reset(targetX);
        strafeController.reset(targetY,power);
        yawController.reset(targetHeading);
        holdTimer.reset();

        while (myOpMode.opModeIsActive())
        {
            xPosition = getXPosition(DistanceUnit.INCH);
            yPosition = getYPosition(DistanceUnit.INCH);
            heading = getHeading(AngleUnit.DEGREES);

            double axial = driveController.getOutput(xPosition);
            double lateral = -strafeController.getOutput(yPosition);
            double yaw = -yawController.getOutput(heading);

            myOpMode.telemetry.addData("Axial :: " , axial);
            myOpMode.telemetry.addData("Lateral :: ", lateral);
            myOpMode.telemetry.addData("Yaw :: ", yaw);

            addLocationToTelemetry();
            updatePositionAndTelemetry();

            drive(driveController.getOutput(xPosition),-strafeController.getOutput(yPosition), -yawController.getOutput(heading),power);
            myOpMode.telemetry.update();
            if(strafeController.isInPosition() && yawController.isInPosition())
            {
                if(holdTimer.time() > holdTime)  break;
            }
            else holdTimer.reset();

            myOpMode.sleep(10);
        }

        drive(0,0,0);
    }

    public void turnTo(double headingDegrees, double power, double holdTime)
    {
        targetX = getXPosition(DistanceUnit.INCH);
        targetY = getYPosition(DistanceUnit.INCH);
        targetHeading = headingDegrees;

        yawController.reset(headingDegrees,power);

        while(myOpMode.opModeIsActive())
        {
            heading = getHeading(AngleUnit.DEGREES);

            addLocationToTelemetry();
            updatePositionAndTelemetry();

            drive(0,0,-yawController.getOutput(heading),power);
            myOpMode.telemetry.update();
            if(yawController.isInPosition())
            {
                if(holdTimer.time() > holdTime) break;
            }
            else holdTimer.reset();
            // Test if needed
            myOpMode.sleep(10);
        }

        // Test if needed
        drive(0,0,0);
    }

    //                     y=strafe   x=fwrd/bwrd
    //                     pos=left
    //        angle 0 = audience angle = 90 to blue people side.
    //        Positive x = audience  Positive y = blue people side
    public void goToPosition(double yLocation, double xLocation, double headingDegree, double power, double holdTime) {
        this.goToPosition(yLocation, xLocation, headingDegree, power, holdTime, 5.0);
    }

    public void goToPosition(double yLocation, double xLocation, double headingDegree, double power, double holdTime, double pathTime) {
        driveController.reset(yLocation, power);
        strafeController.reset(xLocation, power);
        yawController.reset(headingDegree, power);
        pathTimer.reset();

        while(myOpMode.opModeIsActive())
        {
            updatePositionAndTelemetry();

            double xDistance = xLocation - getXPosition(DistanceUnit.INCH);
            double yDistance = yLocation - getYPosition(DistanceUnit.INCH);

            double negativeRadianHeading = -getHeading(AngleUnit.RADIANS);

            double rotatedX = xDistance * Math.cos(negativeRadianHeading) - yDistance * Math.sin(negativeRadianHeading);
            double rotatedY = xDistance * Math.sin(negativeRadianHeading) + yDistance * Math.cos(negativeRadianHeading);

            double axialPower = driveController.getOutputFromError(rotatedX);
            double lateralPower = strafeController.getOutputFromError(rotatedY);
            double yawPower = yawController.getOutput(getHeading(AngleUnit.DEGREES));

            drive(axialPower,-lateralPower, -yawPower);

            myOpMode.telemetry.update();

            if (pathTimer.seconds() > 10) {
                break;
            }

            if(driveController.isInPosition() && strafeController.isInPosition() && yawController.isInPosition())
            {
                if(holdTimer.time() > holdTime) break;
            }
            else holdTimer.reset();

            myOpMode.sleep(10);
        }

        drive(0,0,0);
    }

    public void goToPositionNonBlocking(double yLocation, double xLocation, double headingDegree, double power) {
        updatePositionAndTelemetry();

        double xDistance = xLocation - getXPosition(DistanceUnit.INCH);
        double yDistance = yLocation - getYPosition(DistanceUnit.INCH);

        double negativeRadianHeading = -getHeading(AngleUnit.RADIANS);

        double rotatedX = xDistance * Math.cos(negativeRadianHeading) - yDistance * Math.sin(negativeRadianHeading);
        double rotatedY = xDistance * Math.sin(negativeRadianHeading) + yDistance * Math.cos(negativeRadianHeading);

        double axialPower = driveController.getOutputFromError(rotatedX);
        double lateralPower = strafeController.getOutputFromError(rotatedY);
        double yawPower = yawController.getOutput(headingDegree);

        drive(axialPower, -lateralPower, -yawPower);
    }

    public void driveWithHold(double axialInput, double lateralInput, double yawInput) {
        updatePosition();

        // Deadband joystick
        boolean driverMoving =
                Math.abs(axialInput) > 0.05 ||
                        Math.abs(lateralInput) > 0.05 ||
                        Math.abs(yawInput) > 0.05;

        if (driverMoving) {
            // Update targets based on driver input
            double headingRad = Math.toRadians(getHeading(AngleUnit.DEGREES));

            double fieldX =
                    axialInput * Math.cos(headingRad) -
                            lateralInput * Math.sin(headingRad);

            double fieldY =
                    axialInput * Math.sin(headingRad) +
                            lateralInput * Math.cos(headingRad);

            double dt = loopTimer.seconds();
            loopTimer.reset();

            double driveSpeed = 25;   // inches/sec
            double turnSpeed = 120;   // deg/sec

            desiredX += fieldX * driveSpeed * dt;
            desiredY += fieldY * driveSpeed * dt;
            desiredHeading += yawInput * turnSpeed * dt;

        }

        // PID correction back to target
        double xError = desiredX - getXPosition(DistanceUnit.INCH);
        double yError = desiredY - getYPosition(DistanceUnit.INCH);

        double negHeading = -getHeading(AngleUnit.RADIANS);

        double rotatedX =
                xError * Math.cos(negHeading) -
                        yError * Math.sin(negHeading);

        double rotatedY =
                xError * Math.sin(negHeading) +
                        yError * Math.cos(negHeading);

        double axialPower = driveController.getOutputFromError(rotatedX);
        double lateralPower = strafeController.getOutputFromError(rotatedY);
        double yawPower = yawController.getOutputFromError(
                desiredHeading - getHeading(AngleUnit.DEGREES)
        );

        drive(axialPower, lateralPower, yawPower);
    }

    public void initDriveHold() {
        loopTimer.reset();
        desiredX = getXPosition(DistanceUnit.INCH);
        desiredY = getYPosition(DistanceUnit.INCH);
        desiredHeading = getHeading(AngleUnit.DEGREES);

        driveController.reset(desiredX);
        strafeController.reset(desiredY);
        yawController.reset(desiredHeading);

        holdingPosition = true;
    }


    public void addLocationToTelemetry()
    {
        myOpMode.telemetry.addData("X Position:: ",xPosition);
        myOpMode.telemetry.addData("Y Position:: ",yPosition);
        myOpMode.telemetry.addData("Heading:: ",heading);

        myOpMode.telemetry.addData("X Target:: ",targetX);
        myOpMode.telemetry.addData("Y Target:: ",targetY);
        myOpMode.telemetry.addData("Heading Target:: ", targetHeading);
    }

    public void setLockHeading(double targetHeading){
        yawController.reset(targetHeading);
    }

    public void setLockX(double targetX){
        driveController.reset(targetX);
    }
    public void setLockY(double targetY){
        strafeController.reset(targetY);
    }
}
