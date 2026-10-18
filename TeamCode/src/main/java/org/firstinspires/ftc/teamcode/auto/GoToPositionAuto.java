package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.support.RobotHardware;

@Autonomous(name="Go to Position Auto v1", group = "StarterBot")
public class GoToPositionAuto extends LinearOpMode {
    private RobotHardware bot;

    @Override
    public void runOpMode() throws InterruptedException {
        bot = new RobotHardware(this);
        bot.init();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            bot.goToPosition(20, 20, 170, 1, 0.5, 10);

            int fails = bot.launch(2, 2000);

            telemetry.addData("Fails", fails);
            telemetry.update();

            bot.goToPosition(0, 0, 0, 0.5, 1, 5);
        }
    }
}
