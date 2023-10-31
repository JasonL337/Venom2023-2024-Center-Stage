package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class AprilTagPos {

    GetAprilTags getAprilTags;
    LinearOpMode master;
    public void initAprilTag(LinearOpMode masterC) {
        master = masterC;
        getAprilTags = new GetAprilTags();
        getAprilTags.initAprilTagDetections(masterC);
    }   // end method initAprilTag()
}
