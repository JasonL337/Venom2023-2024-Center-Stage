package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AML 1 Auto", group = "AML 1 Code`")
public class AMLauto extends LinearOpMode implements VisionPortalUser, TensorflowProp{
    Camera camera;
    ProcessDetections processDetections;
    ProcessDetections.pos pos;


    public int in2rev(double inches){
        return (int) Math.round((inches/(4 * Math.PI) * 537.7));

    }

    @Override
    public void runOpMode(){

        DriveTrain dt = new DriveTrain(this);
        initVisionPortal();
        initProcessDetections();
        processDetections.detectTFImages.setProcessor(true);
        waitForStart();
        telemetry.update();

        // move forwards towards the pixels
        dt.moveforward(in2rev(8), .5);
        sleep(3000);
        // lowers the intake
        dt.lowerarms();
        // sets the heading back to zero in order to start tfod detection
        dt.turn(0, .4);
        sleep(3000);
        telemetry.addData("pos", dt.frontR.getCurrentPosition());
        telemetry.update();
        // insert your tensorflow detections here, we still have to do the rotation
        processDetections.setPhase(1);
        pos = processDetections.getPos();
        if(/* insert tensorflow detection for the right*/){
            dt.turn(90,1);
            dt.moveforward(in2rev(60),0.5);
            dt.liftR.setPower(1);
            dt.liftL.setPower(1);
            sleep(300);
            dt.liftR.setPower(0);
            dt.liftL.setPower(0);
            dt.boxOutTake.setPosition(1 /* idk if this is the correct position */);
        }

        if(/* insert tensorflow detection for the middle*/){
            dt.turn(90,1);
            dt.straferight(5,0.5);
            dt.moveforward(in2rev(50),0.5);
            dt.liftR.setPower(1);
            dt.liftL.setPower(1);
            sleep(300);
            dt.liftR.setPower(0);
            dt.liftL.setPower(0);
            dt.boxOutTake.setPosition(1 /* idk if this is the correct position */);
        }

        if(/* insert tensorflow detection for the right*/){
            dt.turn(90,1);
            dt.straferight(10,0.5);
            dt.moveforward(in2rev(50),0.5);
            dt.liftR.setPower(1);
            dt.liftL.setPower(1);
            sleep(300);
            dt.liftR.setPower(0);
            dt.liftL.setPower(0);
            dt.boxOutTake.setPosition(1 /* idk if this is the correct position */);
        }



        // shoots out the pixel
        ElapsedTime time = new ElapsedTime();
        while(time.milliseconds() < 1000) {
            dt.setintakepower(.3);
        }
        // lift up the arm
        dt.liftarms();
        // strafe a bit to the right to detect some more
        dt.straferight(in2rev(5), .5);
        sleep(3000);

        processDetections.setPhase(2);

        if (pos == ProcessDetections.pos.notFound)
        {
            pos = processDetections.getPos();
        }

        dt.moveforward(in2rev(10), .5);

        if (pos == ProcessDetections.pos.left)
        {
            dt.turn(-90, .5);
        }
        if (pos == ProcessDetections.pos.middle)
        {
            dt.turn(180, .5);
        }
        if (pos == ProcessDetections.pos.right)
        {
            dt.turn(90, .5);
        }
        sleep(3000);
        dt.turn(0, .5);
        sleep(3000);

        // move forwards towards the larger door
        dt.moveforward(in2rev(20), 1);
        sleep(3000);
        telemetry.addData("pos", dt.frontR.getCurrentPosition());
        telemetry.update();
        // strafe to the parking zone (right)
        dt.straferight(in2rev(90), 1);
        telemetry.addData("pos", dt.frontR.getCurrentPosition());
        telemetry.update();



    }

    @Override
    public void initProcessDetections() {
        processDetections = new ProcessDetections();
        processDetections.initialize(this, getCamera());
    }

    @Override
    public void initVisionPortal()
    {
        camera = new Camera();
        camera.initVisionPortal(this);
    }

    @Override
    public Camera getCamera()
    {
        return camera;
    }

}
