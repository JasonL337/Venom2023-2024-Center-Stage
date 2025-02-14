package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class AprilTagPos {

    GetAprilTags getAprilTags;
    LinearOpMode master;

    int validId = 0;

    public static final double KNOWN_DIST = 17;
    public static final double KNOWN_HEIGHT = 68;

    public static final double KNOWN_PIX = 200;

    public static final double KNOWN_X = 5.5;
   // public static final double DIST_BETWEEN = 6;

    AprilTagProcessor aprilTagProcessor;

    public void initAprilTag(LinearOpMode masterC, Camera camera) {
        master = masterC;
        getAprilTags = new GetAprilTags();
        getAprilTags.initAprilTagDetections(masterC, camera);
    }   // end method initAprilTag()

    public double getWidth(AprilTagDetection aprilTag)
    {
        return (Math.abs(aprilTag.corners[0].x - aprilTag.corners[1].x) + Math.abs(aprilTag.corners[3].x - aprilTag.corners[2].x)) / 2;
    }

    public void setAprilTagProcessor(AprilTagProcessor at)
    {
        aprilTagProcessor = at;
    }

    public List<AprilTagDetection> getDetections()
    {
        return aprilTagProcessor.getDetections();
    }

    public double[] getDist()
    {
        List<AprilTagDetection> aprilTags = getDetections();
        ArrayList<AprilTagDetection> correctAprilTags = new ArrayList<>();
        correctAprilTags.add(null);
        correctAprilTags.add(null);


        if (aprilTags.size() > 2)
        {
            for (AprilTagDetection aprilTagDetection : aprilTags)
            {
                if (aprilTagDetection.id == validId)
                    correctAprilTags.set(0, aprilTagDetection);
                if (validId == 2 || validId == 5) {
                    if (aprilTagDetection.id == 1 || aprilTagDetection.id == 4)
                        correctAprilTags.set(1, aprilTagDetection);
                }
                else if (aprilTagDetection.id == 4 - validId || aprilTagDetection.id == 10 - validId)
                        correctAprilTags.set(1, aprilTagDetection);
            }
        }
        else if (aprilTags.size() == 2)
        {
            for (AprilTagDetection aprilTagDetection : aprilTags)
            {
                if (aprilTagDetection.id == validId)
                    correctAprilTags.set(0, aprilTagDetection);
                else
                    correctAprilTags.set(1, aprilTagDetection);
            }
        }
        else
        {
            return new double[]{5, 10};
        }



        double distValid = KNOWN_HEIGHT / getWidth(correctAprilTags.get(0)) * KNOWN_DIST;

        double distBetween = correctAprilTags.get(0).center.x - 300;
        double x = (distBetween / KNOWN_PIX) * (distValid / KNOWN_DIST) * KNOWN_X;
        double distOther = KNOWN_HEIGHT / getWidth(correctAprilTags.get(1)) * KNOWN_DIST;
        master.telemetry.addData("dist Valid: ", distValid);
        master.telemetry.addData("dist Other: ", x);
        master.telemetry.addData("center: ", correctAprilTags.get(0).center.x);
        master.telemetry.addData("Known pix: ", correctAprilTags.get(0).center.x - 300);
        master.telemetry.addData("height: ", getWidth(correctAprilTags.get(0)));
       // double distBetween = Math.abs(correctAprilTags.get(0).id - correctAprilTags.get(1).id) * DIST_BETWEEN;

        double angleClose = Math.acos((Math.pow(distBetween, 2) - Math.pow(distValid, 2) - Math.pow(distOther, 2)) / (-2 * distValid * distOther));
        double angleFar;






        //     master.telemetry.addData("angle Close: ", angleClose);
        if (distValid < distOther)
            angleFar = Math.acos((Math.pow(distValid, 2) - Math.pow(distBetween, 2) - Math.pow(distOther, 2)) / (-2 * distBetween * distOther));
        else
            angleFar = Math.acos((Math.pow(distOther, 2) - Math.pow(distBetween, 2) - Math.pow(distValid, 2)) / (-2 * distBetween * distValid));
        master.telemetry.addData("angle Far: ", angleFar);
        double height = Math.sin(angleClose + angleFar) * distValid;
        double distX = Math.cos(angleClose + angleFar) * distValid;
        return new double[]{x, height};

    }

    public void setCorrectAprilTag(ProcessDetections.pos pos, boolean blue)
    {
        if (pos == ProcessDetections.pos.left)
        {
            if (blue)
                validId = 1;
            else
                validId = 4;
        }
        else if (pos == ProcessDetections.pos.middle)
        {
            if (blue)
                validId = 2;
            else
                validId = 5;
        }
        else
        {
            if (blue)
                validId = 3;
            else
                validId = 6;
        }
    }
}
