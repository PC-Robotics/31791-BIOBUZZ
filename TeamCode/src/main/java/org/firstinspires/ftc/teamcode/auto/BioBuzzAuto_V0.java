package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.support.RobotHardware;

@Autonomous(name="BioBuzzAuto_V0", group = "StarterBot")
public class BioBuzzAuto_V0 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware bot = new RobotHardware(this, DistanceUnit.MM);
        bot.init();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {

            bot.goTo(-2820, 200, 0, 1, 0.5, 5);
            bot.goTo(-2820, 1020, -90, 1, 0.5, 5);
            bot.goTo(-3320, 1020, -90, 1, 1, 5);
            bot.launch(4, 2000);


        }
    }
}
