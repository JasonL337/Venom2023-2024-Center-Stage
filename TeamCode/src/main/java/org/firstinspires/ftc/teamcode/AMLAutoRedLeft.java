package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

import java.util.TreeMap;

//luke was here
@Autonomous(name = "Red Auto Far Side", group = "Comp Autos")
@Config
public class AMLAutoRedLeft extends LinearOpMode implements VisionPortalUser, TensorflowProp, UsesTrajectories{
    Camera camera;
    ProcessDetections processDetections;
    ProcessDetections.pos pos;

    public static int angle = 0;
    public static boolean turnTest = false;

    public Pose2d curPose;

    public Pose2d startPose;

    public TreeMap<trajNames, TrajectorySequence> trajs;


    public int in2rev(double inches){
        return (int) Math.round((inches/(4 * Math.PI) * 537.7));

    }

    @Override
    public void initTrajMap()
    {
        trajs = new TreeMap<>();
    }

    private enum trajNames
    {
        turnTest,
        traj1,
        traj2,
        trajLeft1,
        trajLeft2,
        trajMiddle1,
        trajMiddle2,
        trajRight1,
        trajRight2,
        fromPixel,
        toBackBoard,
    }

    @Override
    public void runOpMode(){


        //// INITIALIZATION
        DriveTrain dt = new DriveTrain(this);
        initVisionPortal();
        initProcessDetections();
        initTrajMap();
        processDetections.detectTFImages.setProcessor(true);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        buildInitialTrajs(drive);




        /////////////////////////////////////////////////// PROGRAM STARTING ////////////////////////////////////////

        // PROGRAM STARTS
        waitForStart();
        telemetry.update();


        ///// Starting the program
        if (!isStopRequested()) {
            if (!turnTest) {

                // First traj sequence
                drive.followTrajectorySequence(trajs.get(trajNames.traj1));

                // Scanning first time
                processDetections.setPhase(1);
                pos = processDetections.getPos(false);

                // Second traj sequence
                drive.followTrajectorySequence(trajs.get(trajNames.traj2));

                // Scanning second time if needed
                if (pos == ProcessDetections.pos.notFound) {
                    processDetections.setPhase(2);
                    pos = processDetections.getPos(false);
                }

                Pose2d endPlacePos = new Pose2d();

                if (pos == ProcessDetections.pos.left) {
                    drive.followTrajectorySequence(trajs.get(trajNames.trajLeft1));
                    drop(dt);
                    drive.followTrajectorySequence(trajs.get(trajNames.trajLeft2));
                    endPlacePos = trajs.get(trajNames.trajLeft2).end();
                }
                if (pos == ProcessDetections.pos.middle) {
                    drive.followTrajectorySequence(trajs.get(trajNames.trajMiddle1));
                    //drive.followTrajectorySequence(trajSeq3Middle2);
                    drop(dt);
                    drive.followTrajectorySequence(trajs.get(trajNames.trajMiddle2));
                    endPlacePos = trajs.get(trajNames.trajMiddle2).end();
                }
                if (pos == ProcessDetections.pos.right) {
                    drive.followTrajectorySequence(trajs.get(trajNames.trajRight1));
                    drop(dt);
                    drive.followTrajectorySequence(trajs.get(trajNames.trajRight2));
                    endPlacePos = trajs.get(trajNames.trajRight2).end();
                }

                switchCurPose(endPlacePos);


                ///////

                TrajectorySequence trajSeq4Right = drive.trajectorySequenceBuilder(curPose)
                        .forward(2)
                        //.turn(Math.toRadians(0))
                        .lineToLinearHeading(new Pose2d(70, 65 , Math.toRadians(0)))
                        .build();
                //drive.followTrajectorySequence(trajSeq4Right);
                //drive.followTrajectorySequence(trajSeq3);
            }
            else
            {
                drive.followTrajectorySequence(trajs.get(trajNames.turnTest));
            }
        }



    }

    @Override
    public Pose2d switchCurPose(Pose2d newPose)
    {
        Pose2d tempPose = curPose;
        curPose = newPose;
        return tempPose;
    }

    public TrajectorySequence returnTajTurnTest(SampleMecanumDrive drive)
    {
        TrajectorySequence trajSeqTurnTest = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(angle))
                .build();
        trajs.put(trajNames.turnTest, trajSeqTurnTest);
        return trajSeqTurnTest;
    }

    public TrajectorySequence returnFirstTraj(SampleMecanumDrive drive)
    {
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(curPose)
                //.forward(4)
                //.strafeRight(11)
                .lineToLinearHeading(new Pose2d(curPose.getX() + 11, curPose.getY() - 8, Math.toRadians(270)))
                .waitSeconds(2)
                .build();
        trajs.put(trajNames.traj1, trajSeq);
        switchCurPose(trajSeq.end());
        return  trajSeq;
    }

    public TrajectorySequence returnSecondTraj(SampleMecanumDrive drive)
    {
        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(curPose)
                .strafeRight(10)
                .forward(5)
                .waitSeconds(1)
                .build();
        trajs.put(trajNames.traj2, trajSeq2);
        switchCurPose(trajSeq2.end());
        return  trajSeq2;
    }

    public TrajectorySequence returnTrajLeft(SampleMecanumDrive drive, int step)
    {
        if (step == 1)
        {
            TrajectorySequence trajSeq3Left = drive.trajectorySequenceBuilder(curPose)
                    //.forward(16)
                    //.turn(Math.toRadians(-90))
                    .lineToLinearHeading(new Pose2d(curPose.getX(), curPose.getY() - 28, Math.toRadians(180)))
                    .back(6)
                    .build();
            trajs.put(trajNames.traj1, trajSeq3Left);
            switchCurPose(trajSeq3Left.end());
            return trajSeq3Left;
        }
        else
        {
            TrajectorySequence trajSeq3Left2 = drive.trajectorySequenceBuilder(curPose)
                    .forward(5)
                    //.forward(16)
                    //.turn(Math.toRadians(-90))
                    .strafeRight(6)
                    .lineToLinearHeading(new Pose2d(curPose.getX(), curPose.getY() + 28, Math.toRadians(0)))
                    //.back(3)
                    .build();
            trajs.put(trajNames.traj1, trajSeq3Left2);
            switchCurPose(trajSeq3Left2.end());
            return trajSeq3Left2;
        }
    }

    public TrajectorySequence returnTrajMiddle(SampleMecanumDrive drive, int step)
    {
        if (step == 1)
        { // test for git
            TrajectorySequence trajSeq3Middle = drive.trajectorySequenceBuilder(curPose)
                    //.forward(16)
                    //.turn(Math.toRadians(180))
                    .lineToLinearHeading(new Pose2d(curPose.getX(), curPose.getY() - 12, Math.toRadians(270)))
                    .turn(Math.toRadians(180))
                    .back(3)
                    .build();
            trajs.put(trajNames.traj1, trajSeq3Middle);
            switchCurPose(trajSeq3Middle.end());
            return trajSeq3Middle;
        }
        else
        {
            TrajectorySequence trajSeq3Middle2 = drive.trajectorySequenceBuilder(curPose)
                    .forward(5)
                    //.turn(Math.toRadians(90))
                    //.turn(Math.toRadians(90))
                    .lineToLinearHeading(new Pose2d(curPose.getX(), curPose.getY() + 15, Math.toRadians(0)))
                    //.back(3)
                    .build();
            trajs.put(trajNames.traj1, trajSeq3Middle2);
            switchCurPose(trajSeq3Middle2.end());
            return trajSeq3Middle2;
        }
    }

    public TrajectorySequence returnTrajRight(SampleMecanumDrive drive, int step)
    {
        if (step == 1)
        {
            TrajectorySequence trajSeq3Right = drive.trajectorySequenceBuilder(curPose)
                    //.forward(16)
                    //.turn(Math.toRadians(90))
                    .lineToLinearHeading(new Pose2d(curPose.getX(), curPose.getY() - 29, Math.toRadians(0)))
                    .back(6.5)
                    .build();
            trajs.put(trajNames.traj1, trajSeq3Right);
            switchCurPose(trajSeq3Right.end());
            return trajSeq3Right; // test for 1/2
        }
        else
        {
            TrajectorySequence trajSeq3Right2 = drive.trajectorySequenceBuilder(curPose)
                    .forward(4)
                    .strafeLeft(15)
                    .turn(Math.toRadians(180))
                    .build();
            trajs.put(trajNames.traj1, trajSeq3Right2);
            switchCurPose(trajSeq3Right2.end());
            return trajSeq3Right2;

            /*TrajectorySequence trajSeq3Right2 = drive.trajectorySequenceBuilder(end)
                    .forward(5)
                    .lineToLinearHeading(new Pose2d(end.getX(), end.getY() + 22, Math.toRadians(0)))
                    //.strafeLeft(13)
                    //.turn(Math.toRadians(180))
                    .build();
            return trajSeq3Right2;*/
        }
    }

    public void drop(DriveTrain dt)
    {
        ElapsedTime outputTime = new ElapsedTime();
        while (outputTime.milliseconds() < 2000) {
            dt.liftarms();
        }
    }

    public void buildInitialTrajs(SampleMecanumDrive drive)
    {
        ////// DECLARING START POS FOR ROBOT
        startPose = new Pose2d(12, 61, Math.toRadians(270));
        switchCurPose(startPose);


        ////// CREATING THE TURN TEST TRAJECTORY SEQUENCE
        returnTajTurnTest(drive);


        ////// INITIALIZING THE STARTING POSITION OF THE AUTO PATHING USING startPose
        drive.setPoseEstimate(startPose);


        ////// CREATING THE FIRST TRAJECTORY SEQUENCE. THIS MAINTAINS THE SAME HEADING AS IT
        ////// MOVES DIAGONALLY TO OUR FIRST SCAN AREA ON THE RIGHT POSITION.
        returnFirstTraj(drive);


        ////// CREATING THE SECOND TRAJECTORY SEQUENCE. TO BE FOLLOWED (ALONG WITH THE FIRST)
        ////// NO MATTER THE SCAN/VISION DATA (IT DOES THIS TRAJECTORY IN ALL CASES)
        returnSecondTraj(drive);


        ////// CREATING TWO Pose2d OBJECTS CALLED. THE FIRST, end, IS THE END POSITION OF THE
        ////// SECOND TRAJECTORY SEQUENCE, THE ONE THAT STRAFES LEFT AFTER DOING THE FIRST SCAN.
        ////// THIS IS USED FOR NAVIGATING TO THE PIXEL DROP LOCATION. THE SECOND Pose2d OBJECT,
        ////// CALLED end2, IS THE END POSITION OF THE MOVEMENT NAVIGATING TO THE PIXEL.
        Pose2d endOfScans = trajs.get(trajNames.traj2).end();



        ///////////////////////////////////////////////// LEFT ///////////////////////////////////////////////////////

        switchCurPose(endOfScans);

        ////// THIS IS THE TRAJECTORY SEQUENCE NAVIGATING TO THE PIXEL ON THE LEFT SIDE.
        returnTrajLeft(drive, 1);

        ////// SETTING end2 TO THE ENDING POSITION OF THE FIRST NAVIGATION.

        ////// THIS IS THE TRAJECTORY SEQUENCE NAVIGATING FROM THE PIXEL ON THE LEFT SIDE TOWARDS THE STARING POSITION
        ////// SO THAT WE MAY PARK.
        returnTrajLeft(drive, 2);


        ///////////////////////////////////////////////// MIDDLE ///////////////////////////////////////////////////////


        switchCurPose(endOfScans);
        ////// THIS IS THE TRAJECTORY SEQUENCE NAVIGATING TO THE PIXEL IN THE MIDDLE.
        returnTrajMiddle(drive, 1);
       /* TrajectorySequence trajSeq3Middle2 = drive.trajectorySequenceBuilder(trajSeq3Middle.end())
                .turn(Math.toRadians(90))
                .build();

        */

        ////// SETTING end2 TO THE ENDING POSITION OF THE FIRST NAVIGATION.

        ////// THIS IS THE TRAJECTORY SEQUENCE NAVIGATING FROM THE PIXEL IN THE MIDDLE TOWARDS THE STARING POSITION
        ////// SO THAT WE MAY PARK.
        returnTrajMiddle(drive, 2);


        ///////////////////////////////////////////////// RIGHT ///////////////////////////////////////////////////////

        switchCurPose(endOfScans);

        ////// THIS IS THE TRAJECTORY SEQUENCE NAVIGATING TO THE PIXEL ON THE LEFT SIDE.
        returnTrajRight(drive, 1);

        ////// SETTING end2 TO THE ENDING POSITION OF THE FIRST NAVIGATION.


        ////// THIS IS THE TRAJECTORY SEQUENCE NAVIGATING FROM THE PIXEL ON THE RIGHT SIDE TOWARDS THE STARING POSITION
        ////// SO THAT WE MAY PARK.
        returnTrajRight(drive, 2);

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
