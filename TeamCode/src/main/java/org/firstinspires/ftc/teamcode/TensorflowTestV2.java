package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous(name = "TensorflowTestV2", group = "Concept")
public class TensorflowTestV2 extends LinearOpMode implements TensorflowProp, InitMotors, AprilTag, VisionPortalUser{
    ProcessDetections processDetections;

    AprilTagPos aprilTagPos;
    public void runOpMode()
    {
        //initProcessDetections();
        initAprilTag();
        telemetry.addLine("good to go!");
        telemetry.update();
        waitForStart();
        ElapsedTime elapsedTime = new ElapsedTime();
        aprilTagPos.setCorrectAprilTag(ProcessDetections.pos.middle, false);
        while (opModeIsActive() && !isStopRequested())
        {
            telemetry.addData("time", elapsedTime);
            //telemetry.addData("pos: ", processDetections.getPos());
            //List<AprilTagDetection> dets = aprilTagPos.getDetections();
                //telemetry.addData("height: ", aprilTagPos.getHeight(dets.get(0)));
                double[] dists = aprilTagPos.getDist();
                telemetry.addLine("dist-x: " + dists[0] + "\ndist-y: " + dists[1]);
                // 159.5
                // 14
            //processDetections.test();
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

    @Override
    public void initVisionPortal()
    {

    }
}
