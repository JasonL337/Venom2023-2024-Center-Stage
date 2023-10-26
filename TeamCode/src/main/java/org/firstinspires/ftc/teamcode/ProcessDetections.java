package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

public class ProcessDetections {
    DetectTFImages detectTFImages;
    LinearOpMode master;

    public enum pos
    {
        left,
        middle,
        right
    }
    public void initialize(LinearOpMode masterC) {
        master = masterC;
        masterC.telemetry.addData("test", 4);
        detectTFImages = new DetectTFImages();
        detectTFImages.initTfod();

    }
    // method returns 1 recog ("stop sign", pos, conf), give me list of all recog
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

    public pos getPos()
    {
        return pos.left;
    }

    public void test()
    {

    }
}
