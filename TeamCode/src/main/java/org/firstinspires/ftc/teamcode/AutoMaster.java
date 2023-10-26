package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class AutoMaster extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        ProcessDetections processDetections = new ProcessDetections();
        processDetections.initialize(this);
    }
}
