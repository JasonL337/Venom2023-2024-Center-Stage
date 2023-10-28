package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

public class ProcessDetections {
    DetectTFImages detectTFImages;
    LinearOpMode master;

    public static double xPosMiddle = .5;

    public enum pos
    {
        left,
        middle,
        right
    }
    // Initializes the master reference to use telemetry and the camera dimensions on detectTFImages.
    public void initialize(LinearOpMode masterC) {
        master = masterC;
        //masterC.telemetry.addData("test", 4);
        detectTFImages = new DetectTFImages();
        detectTFImages.initTfod(masterC);
        DetectTFImages.CAMERA_WIDTH = 1920;
        DetectTFImages.CAMERA_HEIGHT = 1080;
        DetectTFImages.CAMERA_ZOOM = 2;

    }

    // method returns 1 recog ("stop sign", pos, conf), give me list of all recognitions
    public Recognition getCorrectDetection() {
        List<Recognition> allDetections = detectTFImages.getTFDetections();
        Recognition correctRecog = null;
        double highestConfidence = 0;
        for (Recognition curRecog : allDetections) {
            if (curRecog.getConfidence() > highestConfidence && curRecog.getLabel().equals("parking meter")) {
                correctRecog = curRecog;
                highestConfidence = curRecog.getConfidence();

            }
        }
        return correctRecog;
    }

    // returns the calculated left, middle, right position of the prop.
    public pos getPos()
    {
        double x = getLerpX();
        if (x > .5)
            return pos.left;
        else if (x > .1)
            return pos.middle;
        return pos.right;
    }

    // Returns the true position of the x dimension. -1 if nothing
    public double getTrueX()
    {
        Recognition recognition = getCorrectDetection();
        if (recognition != null)
            return (getCorrectDetection().getLeft() + getCorrectDetection().getRight()) / 2;
        return -1;
    }

    // Returns the calculated lerp position of the x dimension of the prop to camera size. -1 if nothing.
    public double getLerpX()
    {
        double x = getTrueX();
        if(x != -1)
            return ((x - (DetectTFImages.CAMERA_WIDTH / (DetectTFImages.CAMERA_ZOOM * 2))) / ((double) DetectTFImages.CAMERA_WIDTH / 2));
        return -1;
    }

    public void test()
    {
        double x = getTrueX();
        master.telemetry.addData("true X", x);
        master.telemetry.addData("lerp X", (x - (DetectTFImages.CAMERA_WIDTH / (DetectTFImages.CAMERA_ZOOM * 2))) / ((double) DetectTFImages.CAMERA_WIDTH / 2));
    }
}
