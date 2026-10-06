package org.firstinspires.ftc.teamcode.resources;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public class PIDF {
    public double kp, ki, kd, kf;

    private double target = 0;
    private double error = 0;
    private double prevError = 0;
    private double prevTime = 0;
    private double integralSum = 0;
    private final ElapsedTime elapTime = new ElapsedTime();

    public void setTarget(double set) {
        this.target = set;
    }

    public double getError() {
        return this.error;
    }

    public double getOutput(double current) {
        double time = elapTime.seconds();
        this.error = this.target - current;

        if (time - this.prevTime <= 0) time = this.prevTime + 0.001;

        double p = this.error * this.kp;

        this.integralSum += this.error * (time - this.prevTime);
        double i = this.integralSum * this.ki;

        double d = kd * (this.error - this.prevError) / (time - this.prevTime);

        double f = this.target * this.kf;

        double output =  (p +i + f + d);

        this.prevTime = time;
        this.prevError = this.error;

        return output;
    }
}
