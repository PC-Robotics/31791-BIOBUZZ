package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.support.RobotHardware;

@Autonomous(name="Go to Position Auto v1", group = "StarterBot")
public class GoToPositionAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware bot = new RobotHardware(this);
        bot.init();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            bot.goTo(20, 20, 180, 1, 1 ,5);
            bot.goTo(100, 20, 180, 1, 1, 5);
            bot.goTo(100, 100, 180, 1, 1, 5);

            int fails = bot.launch(2, 2000);

            telemetry.addData("Fails", fails);
            telemetry.update();
            bot.goTo(0, 0, 0, 1, 1, 5);
        }
    }
}
