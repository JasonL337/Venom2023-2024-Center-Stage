package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TensorflowTestV2", group = "Concept")
public class TensorflowTestV2 extends LinearOpMode implements TensorflowProp, InitMotors, AprilTag{
    ProcessDetections processDetections;

    AprilTagPos aprilTagPos;
    public void runOpMode()
    {
        initProcessDetections();
        waitForStart();
        while (opModeIsActive() && !isStopRequested())
        {
            telemetry.addData("pos: ", processDetections.getPos());
            processDetections.test();
            telemetry.update();
        }
    }

    @Override
    public void initProcessDetections() {
        processDetections = new ProcessDetections();
        processDetections.initialize(this);
    }

    @Override
    public void initializeMotors() {
        telemetry.addLine("hi");
    }

    @Override
    public void initAprilTag() {
        aprilTagPos = new AprilTagPos();
        aprilTagPos.initAprilTag(this);
    }
}
