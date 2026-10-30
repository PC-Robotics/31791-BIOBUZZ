package org.firstinspires.ftc.teamcode.support;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PIDController {

    private final Telemetry dashboardTelemetry;


    private final double Kp;
    private final double Ki;
    private final double Kd;
    private final String name;

    private double integralSum;
    private double lastError;

    private double liveOutputLimit;
    private double setPoint;
    private final double tolerance;
    private final double deadband;

    private final boolean circular;
    private boolean inPosition;

    ElapsedTime cycleTime = new ElapsedTime();

    public PIDController(double Kp, double Ki, double Kd,
                         double outputLimit, double tolerance,
                         double deadband, boolean circular, String name, Telemetry telemetry)
    {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        liveOutputLimit = outputLimit;
        this.tolerance = tolerance;
        this.deadband = deadband;
        this.circular = circular;
        this.name = name;
        this.dashboardTelemetry = telemetry;
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

            this.dashboardTelemetry.addData((name + "_Error"), error);
            this.dashboardTelemetry.addData((name + "_P-Output"), (error * Kp));
            this.dashboardTelemetry.addData((name + "_I-Output"), (integralSum * Ki));
            this.dashboardTelemetry.addData((name + "_D-Output"), (derivative * Kd));
        }

        cycleTime.reset();

        inPosition = (Math.abs(error) < tolerance);

        return Range.clip(output,-liveOutputLimit,liveOutputLimit);
    }

    public boolean isInPosition()
    {
        return inPosition;
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

    public void reset()
    {
        cycleTime.reset();
        inPosition = false;
        lastError = 0.0;
        integralSum = 0.0;
    }
}
