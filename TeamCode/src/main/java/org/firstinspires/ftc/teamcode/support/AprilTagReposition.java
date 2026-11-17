package org.firstinspires.ftc.teamcode.support;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

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
    }

    public void repositionToScore() {
        aprilTagWebcam.update();
        AprilTagClusterDetection tag = getCorrectTag(aprilTagWebcam.getDetectedTags());
    }

    public AprilTagClusterDetection getCorrectTag(List<AprilTagClusterDetection> tagList) {
        for(AprilTagClusterDetection detection: tagList) {
            String name = detection.metadata.name;
            if((isRed && (name.charAt(0) == 'R') || !isRed && (name.charAt(0) == 'B')) && Math.abs(detection.ftcPose.roll) < 90) {
                return detection;
            }
        }
        return null;
    }


}
