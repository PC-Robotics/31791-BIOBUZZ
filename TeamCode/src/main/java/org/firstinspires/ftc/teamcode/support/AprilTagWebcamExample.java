package org.firstinspires.ftc.teamcode.support;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

@Autonomous(name = "Test AprilTag", group = "StarterBot")
public class AprilTagWebcamExample extends OpMode {
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        aprilTagWebcam.update();
        AprilTagSingleDetection id42 = aprilTagWebcam.getTagBySpecificId(42);
        aprilTagWebcam.displayDetectionTelemetry(id42);
    }
}
