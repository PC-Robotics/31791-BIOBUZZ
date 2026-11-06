package org.firstinspires.ftc.teamcode.support;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PIDController {

    private double Kp;
    private double Ki;
    private double Kd;

    private double integralSum;
    private double lastError;

    private double liveOutputLimit;
    private double outputLimit;
    private double setPoint;
    private double tolerance;
    private double deadband;

    private boolean circular;
    private boolean inPosition;
    private String name;
    private Telemetry telemetry;

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

    public PIDController(double Kp, double Ki, double Kd,
                         double outputLimit, double tolerance,
                         double deadband, boolean circular,
                         String name, Telemetry telemetry)
    {
        this(Kp, Ki, Kd, outputLimit, tolerance, deadband, circular);
        this.name = name;
        this.telemetry = telemetry;
    }

    public void setPID(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public double getOutput(double input)
    {
        double error = setPoint - input;
        return getOutputFromError(error);
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

        if (Math.abs(error) < deadband)
        {
            output = 0;
        }
        else {
            if (circular) {
                while (error > 180) error -= 360;
                while (error <= -180) error += 360;
            }

            double dt = cycleTime.seconds();
            if (dt <= 0.0001) dt = 0.0001;

            integralSum += error * dt;

            double derivative = 0;
            if (lastError != 0.0) {
                double diff = error - lastError;
                if (circular) {
                    while (diff > 180) diff -= 360;
                    while (diff <= -180) diff += 360;
                }
                derivative = diff / dt;
            }
            lastError = error;

            output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        }

        cycleTime.reset();

        inPosition = (Math.abs(error) < tolerance);

        return Range.clip(output, -liveOutputLimit, liveOutputLimit);
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
