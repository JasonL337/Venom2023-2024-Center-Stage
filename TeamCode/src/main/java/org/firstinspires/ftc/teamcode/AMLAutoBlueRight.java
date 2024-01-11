package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

@Autonomous(name = "AML 1 Auto Blue Right", group = "AML 1 Code`")
@Config
public class AMLAutoBlueRight extends LinearOpMode implements VisionPortalUser, TensorflowProp{
    Camera camera;
    ProcessDetections processDetections;
    ProcessDetections.pos pos;

    public static int angle = 0;
    public static boolean turnTest = false;


    public int in2rev(double inches){
        return (int) Math.round((inches/(4 * Math.PI) * 537.7));

    }

    @Override
    public void runOpMode(){


        //// INITIALIZATION
        DriveTrain dt = new DriveTrain(this);
        initVisionPortal();
        initProcessDetections();
        processDetections.detectTFImages.setProcessor(true);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        ////// DECLARING START POS FOR ROBOT
        Pose2d startPose = new Pose2d(12, -61, Math.toRadians(90));


        ////// CREATING THE TURN TEST TRAJECTORY SEQUENCE
        TrajectorySequence trajSeqTurnTest = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(angle))
                .build();


        ////// INITIALIZING THE STARTING POSITION OF THE AUTO PATHING USING startPose
        drive.setPoseEstimate(startPose);


        ////// CREATING THE FIRST TRAJECTORY SEQUENCE. THIS MAINTAINS THE SAME HEADING AS IT
        ////// MOVES DIAGONALLY TO OUR FIRST SCAN AREA ON THE RIGHT POSITION.
        TrajectorySequence trajSeq = returnFirstTraj(drive, startPose);


        ////// CREATING THE SECOND TRAJECTORY SEQUENCE. TO BE FOLLOWED (ALONG WITH THE FIRST)
        ////// NO MATTER THE SCAN/VISION DATA (IT DOES THIS TRAJECTORY IN ALL CASES)
        TrajectorySequence trajSeq2 = returnSecondTraj(drive, trajSeq.end());


        ////// CREATING TWO Pose2d OBJECTS CALLED. THE FIRST, end, IS THE END POSITION OF THE
        ////// SECOND TRAJECTORY SEQUENCE, THE ONE THAT STRAFES LEFT AFTER DOING THE FIRST SCAN.
        ////// THIS IS USED FOR NAVIGATING TO THE PIXEL DROP LOCATION. THE SECOND Pose2d OBJECT,
        ////// CALLED end2, IS THE END POSITION OF THE MOVEMENT NAVIGATING TO THE PIXEL.
        Pose2d end = trajSeq2.end();
        Pose2d end2;



        ///////////////////////////////////////////////// LEFT ///////////////////////////////////////////////////////



        ////// THIS IS THE TRAJECTORY SEQUENCE NAVIGATING TO THE PIXEL ON THE LEFT SIDE.
        TrajectorySequence trajSeq3Left = returnTrajLeft(drive, end, 1);

        ////// SETTING end2 TO THE ENDING POSITION OF THE FIRST NAVIGATION.
        end2 = trajSeq3Left.end();

        ////// THIS IS THE TRAJECTORY SEQUENCE NAVIGATING FROM THE PIXEL ON THE LEFT SIDE TOWARDS THE STARING POSITION
        ////// SO THAT WE MAY PARK.
        TrajectorySequence trajSeq3Left2 = returnTrajLeft(drive, end2, 2);


        ///////////////////////////////////////////////// MIDDLE ///////////////////////////////////////////////////////


        ////// THIS IS THE TRAJECTORY SEQUENCE NAVIGATING TO THE PIXEL IN THE MIDDLE.
        TrajectorySequence trajSeq3Middle = returnTrajMiddle(drive, end, 1);
       /* TrajectorySequence trajSeq3Middle2 = drive.trajectorySequenceBuilder(trajSeq3Middle.end())
                .turn(Math.toRadians(90))
                .build();

        */

        ////// SETTING end2 TO THE ENDING POSITION OF THE FIRST NAVIGATION.
        end2 = trajSeq3Middle.end();

        ////// THIS IS THE TRAJECTORY SEQUENCE NAVIGATING FROM THE PIXEL IN THE MIDDLE TOWARDS THE STARING POSITION
        ////// SO THAT WE MAY PARK.
        TrajectorySequence trajSeq3Middle3 = returnTrajMiddle(drive, end2, 2);


        ///////////////////////////////////////////////// RIGHT ///////////////////////////////////////////////////////


        ////// THIS IS THE TRAJECTORY SEQUENCE NAVIGATING TO THE PIXEL ON THE RIGHT SIDE.
        TrajectorySequence trajSeq3Right = returnTrajRight(drive, end, 1);

        ////// SETTING end2 TO THE ENDING POSITION OF THE FIRST NAVIGATION.
        end2 = trajSeq3Right.end();


        ////// THIS IS THE TRAJECTORY SEQUENCE NAVIGATING FROM THE PIXEL ON THE RIGHT SIDE TOWARDS THE STARING POSITION
        ////// SO THAT WE MAY PARK.
        TrajectorySequence trajSeq3Right2 = returnTrajRight(drive, end2, 2);


        /////////////////////////////////////////////////// PROGRAM STARTING ////////////////////////////////////////

        // PROGRAM STARTS
        waitForStart();
        telemetry.update();


        ///// Starting the program
        if (!isStopRequested()) {
            if (!turnTest) {

                // First traj sequence
                drive.followTrajectorySequence(trajSeq);

                // Scanning first time
                processDetections.setPhase(1);
                pos = processDetections.getPos(true);

                // Second traj sequence
                drive.followTrajectorySequence(trajSeq2);

                // Scanning second time if needed
                if (pos == ProcessDetections.pos.notFound) {
                    processDetections.setPhase(2);
                    pos = processDetections.getPos(true);
                }

                Pose2d endPlacePos = new Pose2d();

                if (pos == ProcessDetections.pos.left) {
                    drive.followTrajectorySequence(trajSeq3Left);
                    drop(dt);
                    drive.followTrajectorySequence(trajSeq3Left2);
                    endPlacePos = trajSeq3Left.end();
                }
                if (pos == ProcessDetections.pos.middle) {
                    drive.followTrajectorySequence(trajSeq3Middle);
                    //drive.followTrajectorySequence(trajSeq3Middle2);
                    drop(dt);
                    drive.followTrajectorySequence(trajSeq3Middle3);
                    endPlacePos = trajSeq3Middle.end();
                }
                if (pos == ProcessDetections.pos.right) {
                    drive.followTrajectorySequence(trajSeq3Right);
                    drop(dt);
                    drive.followTrajectorySequence(trajSeq3Right2);
                    endPlacePos = trajSeq3Right2.end();
                }


                ///////

                TrajectorySequence trajSeq4Right = drive.trajectorySequenceBuilder(endPlacePos)
                        .forward(2)
                        //.turn(Math.toRadians(0))
                        .lineToLinearHeading(new Pose2d(60, -70, Math.toRadians(0)))
                        .build();
                //drive.followTrajectorySequence(trajSeq4Right);
                //drive.followTrajectorySequence(trajSeq3);
            }
            else
            {
                drive.followTrajectorySequence(trajSeqTurnTest);
            }
        }



    }

    public TrajectorySequence returnFirstTraj(SampleMecanumDrive drive, Pose2d end)
    {
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(end)
                //.forward(4)
                //.strafeRight(11)
                .lineToLinearHeading(new Pose2d(end.getX() + 11, end.getY() + 8, Math.toRadians(90)))
                .waitSeconds(2)
                .build();
        return  trajSeq;
    }

    public TrajectorySequence returnSecondTraj(SampleMecanumDrive drive, Pose2d end)
    {
        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(end)
                .strafeLeft(10)
                .waitSeconds(1)
                .build();
        return  trajSeq2;
    }

    public TrajectorySequence returnTrajLeft(SampleMecanumDrive drive, Pose2d end, int step)
    {
        if (step == 1)
        {
            TrajectorySequence trajSeq3Left = drive.trajectorySequenceBuilder(end)
                    //.forward(16)
                    //.turn(Math.toRadians(-90))
                    .lineToLinearHeading(new Pose2d(end.getX(), end.getY() + 28, Math.toRadians(0)))
                    .back(8.5)
                    .build();
            return trajSeq3Left;
        }
        else
        {
            TrajectorySequence trajSeq3Left2 = drive.trajectorySequenceBuilder(end)
                    .forward(6)
                    //.forward(16)
                    //.turn(Math.toRadians(-90))
                    .lineToLinearHeading(new Pose2d(end.getX(), end.getY() - 28, Math.toRadians(0)))
                    //.back(3)
                    .build();
            return trajSeq3Left2;
        }
    }

    public TrajectorySequence returnTrajMiddle(SampleMecanumDrive drive, Pose2d end, int step)
    {
        if (step == 1)
        { // test for git
            TrajectorySequence trajSeq3Middle = drive.trajectorySequenceBuilder(end)
                    //.forward(16)
                    //.turn(Math.toRadians(180))
                    .lineToLinearHeading(new Pose2d(end.getX(), end.getY() + 15, Math.toRadians(90)))
                    .turn(Math.toRadians(180))
                    .back(2)
                    .build();
            return trajSeq3Middle;
        }
        else
        {
            TrajectorySequence trajSeq3Middle3 = drive.trajectorySequenceBuilder(end)
                    .forward(5)
                    //.turn(Math.toRadians(90))
                    //.turn(Math.toRadians(90))
                    .lineToLinearHeading(new Pose2d(end.getX(), end.getY() - 15, Math.toRadians(0)))
                    //.back(3)
                    .build();
            return trajSeq3Middle3;
        }
    }

    public TrajectorySequence returnTrajRight(SampleMecanumDrive drive, Pose2d end, int step)
    {
        if (step == 1)
        {
            TrajectorySequence trajSeq3Right = drive.trajectorySequenceBuilder(end)
                    //.forward(16)
                    //.turn(Math.toRadians(90))
                    .lineToLinearHeading(new Pose2d(end.getX(), end.getY() + 29, Math.toRadians(180)))
                    .back(5)
                    .build();
            return trajSeq3Right;
        }
        else
        {

            TrajectorySequence trajSeq3Right2 = drive.trajectorySequenceBuilder(end)
                    .forward(4)
                    .strafeLeft(15)
                    .turn(Math.toRadians(180))
                    .build();
            return trajSeq3Right2;

            /*TrajectorySequence trajSeq3Right2 = drive.trajectorySequenceBuilder(end)
                    .forward(4)
                    .lineToLinearHeading(new Pose2d(end.getX(), end.getY() - 22, Math.toRadians(0)))
                    //.strafeLeft(13)
                    //.turn(Math.toRadians(180))
                    .build();
            return trajSeq3Right2;*/
        }
    }

    public void drop(DriveTrain dt)
    {
        ElapsedTime outputTime = new ElapsedTime();
        while (outputTime.milliseconds() < 4000) {
            dt.liftarms();
        }
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
