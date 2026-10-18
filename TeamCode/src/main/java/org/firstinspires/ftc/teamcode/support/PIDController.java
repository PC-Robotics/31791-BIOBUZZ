package org.firstinspires.ftc.teamcode.support;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class PIDController {

    private final double Kp;
    private final double Ki;
    private final double Kd;

    private double integralSum;
    private double lastError;

    private double liveOutputLimit;
    private final double outputLimit;
    private double setPoint;
    private final double tolerance;
    private final double deadband;

    private final boolean circular;
    private boolean inPosition;

    ElapsedTime cycleTime = new ElapsedTime();

    public PIDController(double Kp, double Ki, double Kd,
                         double outputLimit, double tolerance,
                         double deadband, boolean circular)
    {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.outputLimit = outputLimit;
        liveOutputLimit = outputLimit;
        this.tolerance = tolerance;
        this.deadband = deadband;
        this.circular = circular;
    }

    public double getOutput(double input)
    {
        double error = setPoint - input;
        double output;


        if(Math.abs(error) < deadband)
        {
            output = 0;
        }
        else {
            if (circular) {
                while (error > 180) error -= 360;
                while (error <= -180) error += 360;
            }

            integralSum += error * cycleTime.seconds();
            double derivative = (error - lastError) / cycleTime.seconds();
            lastError = error;

            output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        }

        cycleTime.reset();

        inPosition = (Math.abs(error) < tolerance);

        return Range.clip(output,-liveOutputLimit,liveOutputLimit);
    }

    public boolean isInPosition()
    {
        return inPosition;
    }

    public double getSetPoint()
    {
        return setPoint;
    }

    public double getOutputFromError(double error)
    {
        double output;

        if(Math.abs(error) < deadband)
        {
            output = 0;
        }
        else {
            if (circular) {
                while (error > 180) error -= 360;
                while (error <= -180) error += 360;
            }

            integralSum += error * cycleTime.seconds();
            double derivative = (error - lastError) / cycleTime.seconds();
            lastError = error;

            output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        }

        cycleTime.reset();

        inPosition = (Math.abs(error) < tolerance);

        return Range.clip(output,-liveOutputLimit,liveOutputLimit);
    }

    public void reset(double setPoint, double powerLimit)
    {
        this.setPoint = setPoint;
        liveOutputLimit = Math.abs(powerLimit);
        reset();
    }

    public void reset(double setPoint)
    {
        this.setPoint = setPoint;
        liveOutputLimit = outputLimit;
        reset();
    }

    public void reset()
    {
        cycleTime.reset();
        inPosition = false;
        lastError = 0.0;
        integralSum = 0.0;
    }
}
