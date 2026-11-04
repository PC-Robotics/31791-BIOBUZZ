package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.support.RobotHardware;

@Autonomous(name="Go to Position Auto v1", group = "StarterBot")
public class GoToPositionAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        RobotHardware bot = new RobotHardware(this, DistanceUnit.INCH);
        bot.init();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            bot.goTo(0, 100, 0, 1, 1, 10);
            bot.goTo(0, 0, 0, 1, 1, 10);

            bot.goTo(0, 0, 0, 1, 1, 10);
        }
    }
}
