package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class PIDFTuner extends OpMode {
    public DcMotorEx flywheel;
    double highVel = 2000;
    double lowVel  = 800;
    double curTargetVel = highVel;

    double F = 0;
    double P = 0;
    double[] stepSizes = {10, 1, 0.1, 0.01, 0.001, 0.0001};

    int stepIndex = 1;



    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, "launcher");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    @Override
    public void loop() {
        if (gamepad1.yWasPressed()) curTargetVel = (curTargetVel == highVel) ? lowVel : highVel;
        if (gamepad1.bWasPressed()) stepIndex = (stepIndex + 1) % stepSizes.length;

        if (gamepad1.dpadDownWasPressed()) P -= stepSizes[stepIndex];
        if (gamepad1.dpadUpWasPressed()) P += stepSizes[stepIndex];

        if (gamepad1.dpadLeftWasPressed()) F -= stepSizes[stepIndex];
        if (gamepad1.dpadRightWasPressed()) F += stepSizes[stepIndex];

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        flywheel.setVelocity(curTargetVel);

        double curVel = flywheel.getVelocity();
        double err = curTargetVel - curVel;

        telemetry.addData("Target Velocity: ", curTargetVel);
        telemetry.addData("Real Velocity: ", curVel);
        telemetry.addData("Error: ", err);
        telemetry.addLine("--------------------------------");
        telemetry.addData("P-val: ", P);
        telemetry.addData("F-val: ", F);
        telemetry.addData("Step size: ", stepSizes[stepIndex]);
    }
}
