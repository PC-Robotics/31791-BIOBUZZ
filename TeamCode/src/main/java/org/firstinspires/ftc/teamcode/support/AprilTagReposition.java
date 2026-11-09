package org.firstinspires.ftc.teamcode.support;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Reposition AprilTag", group = "StarterBot")
public class AprilTagReposition extends OpMode {
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    boolean isRed = true;
    /* left -> right
    red tipped: 30, 31, 32, 33
    red initial: 34, 35, 36, 37
    blue tipped: 38, 39, 40, 41
    blue initial: 42, 43, 44, 45
    */

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        aprilTagWebcam.update();
        AprilTagSingleDetection id20 = aprilTagWebcam.getTagBySpecificId(20);
        aprilTagWebcam.displayDetectionTelemetry(id20);
    }

    public void repositionToScore() {
        aprilTagWebcam.update();
        List<AprilTagSingleDetection> tagList = aprilTagWebcam.getDetectedTags();
    }

    public AprilTagSingleDetection getCorrectAprilTag(List<AprilTagSingleDetection> tagList) {
        List<Integer> idList = new ArrayList<>();
        for(AprilTagSingleDetection detection: tagList) {
            int id = detection.id;
            if(isRed && id >= 30 && id < 38 || !isRed && id >= 38 && id < 46) {
                idList.add(detection.id);
            }
        }
        return null;
    }
}
