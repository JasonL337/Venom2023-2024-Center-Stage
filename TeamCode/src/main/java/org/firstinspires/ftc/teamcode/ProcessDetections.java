package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

public class ProcessDetections {
    DetectTFImages detectTFImages;
    LinearOpMode master;

    public int phase = 0;

    public static double xPosMiddle = .5;

    Recognition recognition = null;

    public enum pos
    {
        left,
        middle,
        right,
        notFound,
    }
    // Initializes the master reference to use telemetry and the camera dimensions on detectTFImages.
    public void initialize(LinearOpMode masterC, Camera camera) {
        master = masterC;
        //masterC.telemetry.addData("test", 4);
        detectTFImages = new DetectTFImages();
        detectTFImages.initTfod(masterC, camera);
        DetectTFImages.CAMERA_WIDTH = 1920;
        DetectTFImages.CAMERA_HEIGHT = 1080;
        DetectTFImages.CAMERA_ZOOM = 2;

    }

    // Setting the phase variable to the current phase of the auto it's in
    public void setPhase(int p)
    {
        phase = p;
    }

    // method returns 1 recog ("stop sign", pos, conf), give me list of all recognitions
    public Recognition getCorrectDetection() {
        List<Recognition> allDetections = detectTFImages.getTFDetections();
        Recognition correctRecog = null;
        Recognition correctCupRecog = null;
        double highestCupConf = 0.3;
        double highestConfidence = 0.2;
        for (Recognition curRecog : allDetections) {
            if (curRecog.getConfidence() > highestConfidence && curRecog.getLabel().equals("parking meter")) {
                correctRecog = curRecog;
                highestConfidence = curRecog.getConfidence();
            }
            else if (curRecog.getConfidence() > highestCupConf && (curRecog.getLabel().equals("cup") || curRecog.getLabel().equals("sports ball")))
            {
                correctCupRecog = curRecog;
                highestCupConf = curRecog.getConfidence();
            }
        }
        if (correctRecog != null)
            return correctRecog;
        return correctCupRecog;
    }

    // returns the calculated left, middle, right position of the prop.
    public pos getPos(boolean right)
    {
        recognition = getCorrectDetection();
        if (phase == 1 && recognition != null)
        {
            if (!right)
                return pos.left;
            else return pos.right;
        }
        else if (phase == 2)
        {
            if (recognition == null)
            {
                if (!right)
                    return pos.right;
                else return pos.left;
            }
            else
            {
                return  pos.middle;
            }
        }
        return  pos.notFound;
        /*double x = getLerpX();
        if (x > .8 || x == -1)
            return pos.right;
        else if (x > .25)
            return pos.middle;
        return pos.left;*/
    }

    // Returns the true position of the x dimension. -1 if nothing
    public double getTrueX()
    {
        Recognition recognition = getCorrectDetection();
        if (recognition != null)
            return (recognition.getLeft() + recognition.getRight()) / 2;
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
