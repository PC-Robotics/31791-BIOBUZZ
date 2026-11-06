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
//        for (int i=aprilTagWebcam.getAllDetections().size(); i > 0; i--) {
//            aprilTagWebcam.update();
//            AprilTagSingleDetection detectedTag = aprilTagWebcam.getTagBySpecificId(i);
//            aprilTagWebcam.displayDetectionTelemetry(detectedTag);
//        }

        telemetry.addData("# Tags Detected", aprilTagWebcam.getAllDetections().size());
        telemetry.addData("Raw Single Tags", (aprilTagWebcam.getDetectedTags()).toString());
    }

    @Override
    public void stop() {
        aprilTagWebcam.stop();
    }
}
