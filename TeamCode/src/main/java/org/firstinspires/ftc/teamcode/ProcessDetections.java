package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

public class ProcessDetections {
    // method returns 1 recog ("stop sign", pos, conf), give me list of all recog
    public Recognition getCorrectDetection(List<Recognition> allDetections) {
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
}
