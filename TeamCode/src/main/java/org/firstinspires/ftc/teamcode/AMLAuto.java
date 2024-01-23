package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

import java.util.TreeMap;
@Autonomous(name = "Red Auto Close Side", group = "Comp Autos")
@Config
public class AMLAuto extends LinearOpMode implements VisionPortalUser, TensorflowProp, UsesTrajectories{

    Camera camera;
    ProcessDetections processDetections;
    ProcessDetections.pos pos;

    public static int angle = 0;
    public static boolean turnTest = false;

    public Pose2d curPose;

    public Pose2d startPose;

    public double dist;

    public TreeMap<AMLAuto.trajNames, TrajectorySequence> trajs;


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
        backBoardSetupLeft,
        strafeToDropLeft,
        trajMiddle1,
        trajMiddle2,
        backBoardSetupMiddle,
        strafeToDropMiddle,
        trajRight1,
        trajRight2,
        backBoardSetupRight,
        strafeToDropRight,
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
        DistanceSensorTest distanceSensorTest = new DistanceSensorTest();





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
                pos = processDetections.getPos(true);

                // Second traj sequence
                drive.followTrajectorySequence(trajs.get(trajNames.traj2));

                // Scanning second time if needed
                if (pos == ProcessDetections.pos.notFound) {
                    processDetections.setPhase(2);
                    pos = processDetections.getPos(true);
                }

                Pose2d endPlacePos = new Pose2d();

                if (pos == ProcessDetections.pos.left) {
                    drive.followTrajectorySequence(trajs.get(trajNames.trajLeft1));
                    drop(dt);
                    drive.followTrajectorySequence(trajs.get(trajNames.trajLeft2));
                    drive.followTrajectorySequence(trajs.get(trajNames.backBoardSetupLeft));
                    drive.followTrajectorySequence(trajs.get(trajNames.strafeToDropLeft));
                    dt.setMotorPowerDist(.25, 2, distanceSensorTest.getDist());
                    dt.raiselifts(1.3);
                    dt.lowerBox();
                    endPlacePos = trajs.get(trajNames.strafeToDropLeft).end();
                }

                if (pos == ProcessDetections.pos.middle) {
                    drive.followTrajectorySequence(trajs.get(trajNames.trajMiddle1));
                    drop(dt);
                    drive.followTrajectorySequence(trajs.get(trajNames.trajMiddle2));
                    drive.followTrajectorySequence(trajs.get(trajNames.backBoardSetupMiddle));
                    drive.followTrajectorySequence(trajs.get(trajNames.strafeToDropMiddle));
                    dt.setMotorPowerDist(.25, 2, distanceSensorTest.getDist());
                    dt.raiselifts(1.3);
                    dt.lowerBox();
                    endPlacePos = trajs.get(trajNames.strafeToDropMiddle).end();
                }

                if (pos == ProcessDetections.pos.right) {
                    drive.followTrajectorySequence(trajs.get(trajNames.trajRight1));
                    drop(dt);
                    drive.followTrajectorySequence(trajs.get(trajNames.trajRight2));
                    drive.followTrajectorySequence(trajs.get(trajNames.backBoardSetupRight));
                    drive.followTrajectorySequence(trajs.get(trajNames.strafeToDropRight));
                    dt.setMotorPowerDist(.25, 2, distanceSensorTest.getDist());
                    dt.raiselifts(1.3);
                    dt.lowerBox();
                    endPlacePos = trajs.get(trajNames.strafeToDropRight).end();
                }

                switchCurPose(endPlacePos);

                ///////

                TrajectorySequence trajSeq4Right = drive.trajectorySequenceBuilder(curPose)
                        .forward(2)
                        //.turn(Math.toRadians(0))
                        .lineToLinearHeading(new Pose2d(60, -70, Math.toRadians(0)))
                        .build();
                //drive.followTrajectorySequence(trajSeq4Right);
                //drive.followTrajectorySequence(trajSeq3);
            }
            else
            {
                //drive.followTrajectorySequence(trajSeqTurnTest);
            }
        }

// Testing Jason edits

    }

    @Override
    public Pose2d switchCurPose(Pose2d newPose)
    {
        Pose2d tempPose = curPose;
        curPose = newPose;
        return tempPose;
    }

    public TrajectorySequence returnFirstTraj(SampleMecanumDrive drive)
    {
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(curPose)
                //.forward(4)
                //.strafeRight(11)
                .lineToLinearHeading(new Pose2d(curPose.getX() + 11, curPose.getY() + 8, Math.toRadians(90)))
                .waitSeconds(2)
                .build();
        trajs.put(AMLAuto.trajNames.traj1, trajSeq);
        switchCurPose(trajSeq.end());
        return  trajSeq;
    }

    public TrajectorySequence returnSecondTraj(SampleMecanumDrive drive)
    {
        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(curPose)
                .strafeLeft(13)
                .waitSeconds(1)
                .build();
        trajs.put(AMLAuto.trajNames.traj2, trajSeq2);
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
                    .lineToLinearHeading(new Pose2d(curPose.getX(), curPose.getY() + 26, Math.toRadians(0)))
                    .back(7)
                    .build();
            trajs.put(AMLAuto.trajNames.trajLeft1, trajSeq3Left);
            switchCurPose(trajSeq3Left.end());
            return trajSeq3Left;
        }
        else
        {
            TrajectorySequence trajSeq3Left2 = drive.trajectorySequenceBuilder(curPose)
                    .forward(12)
                    //.forward(16)
                    //.turn(Math.toRadians(-90))
                    .lineToLinearHeading(new Pose2d(30, -40, Math.toRadians(0)))
                    //.back(3)
                    .build();
            trajs.put(AMLAuto.trajNames.trajLeft2, trajSeq3Left2);
            switchCurPose(trajSeq3Left2.end());
            return trajSeq3Left2;
        }
    }

    public TrajectorySequence moveToBackBoardLeft(SampleMecanumDrive drive)
    {
        TrajectorySequence backBoardSetupLeft = drive.trajectorySequenceBuilder(curPose)
                .lineToLinearHeading(new Pose2d(30, -25, Math.toRadians(0)))
                .build();
        trajs.put(AMLAuto.trajNames.backBoardSetupLeft, backBoardSetupLeft);
        switchCurPose(backBoardSetupLeft.end());
        return backBoardSetupLeft;
    }

    public TrajectorySequence strafeToDropLeft(SampleMecanumDrive drive)
    {
        AprilTagPos aprilTagPos = new AprilTagPos();
        aprilTagPos.setCorrectAprilTag(pos, false);
        TrajectorySequence dropSetupLeft = drive.trajectorySequenceBuilder(curPose)
                .strafeRight(aprilTagPos.getDist()[0])
                .build();
        trajs.put(trajNames.strafeToDropLeft, dropSetupLeft);
        switchCurPose(dropSetupLeft.end());
        return dropSetupLeft;
    }

    public TrajectorySequence returnTrajMiddle(SampleMecanumDrive drive, int step)
    {
        if (step == 1)
        { // test for git
            TrajectorySequence trajSeq3Middle = drive.trajectorySequenceBuilder(curPose)
                    //.forward(16)
                    //.turn(Math.toRadians(180))
                    .lineToLinearHeading(new Pose2d(curPose.getX(), curPose.getY() + 15, Math.toRadians(90)))
                    .turn(Math.toRadians(180))
                    .back(3)
                    .build();
            trajs.put(AMLAuto.trajNames.trajMiddle1, trajSeq3Middle);
            switchCurPose(trajSeq3Middle.end());
            return trajSeq3Middle;
        }
        else
        {
            TrajectorySequence trajSeq3Middle2 = drive.trajectorySequenceBuilder(curPose)
                    .forward(7)
                    //.turn(Math.toRadians(90))
                    //.turn(Math.toRadians(90))
                    .lineToLinearHeading(new Pose2d(30, -40, Math.toRadians(0)))
                    //.back(3)
                    .build();
            trajs.put(AMLAuto.trajNames.trajMiddle2, trajSeq3Middle2);
            switchCurPose(trajSeq3Middle2.end());
            return trajSeq3Middle2;
        }
    }

    public TrajectorySequence moveToBackBoardMiddle(SampleMecanumDrive drive)
    {
        TrajectorySequence backBoardSetupMiddle = drive.trajectorySequenceBuilder(curPose)
                .lineToLinearHeading(new Pose2d(30, -25, Math.toRadians(0)))
                .build();
        trajs.put(trajNames.backBoardSetupMiddle, backBoardSetupMiddle);
        switchCurPose(backBoardSetupMiddle.end());
        return backBoardSetupMiddle;
    }

    public TrajectorySequence strafeToDropMiddle(SampleMecanumDrive drive)
    {
        AprilTagPos aprilTagPos = new AprilTagPos();
        aprilTagPos.setCorrectAprilTag(pos, false);
        TrajectorySequence dropSetupMiddle = drive.trajectorySequenceBuilder(curPose)
                .strafeRight(aprilTagPos.getDist()[0])
                .build();
        trajs.put(trajNames.strafeToDropMiddle, dropSetupMiddle);
        switchCurPose(dropSetupMiddle.end());
        return dropSetupMiddle;
    }

    public TrajectorySequence returnTrajRight(SampleMecanumDrive drive, int step)
    {
        if (step == 1)
        {
            TrajectorySequence trajSeq3Right = drive.trajectorySequenceBuilder(curPose)
                    //.forward(16)
                    //.turn(Math.toRadians(90))
                    .lineToLinearHeading(new Pose2d(curPose.getX(), curPose.getY() + 29, Math.toRadians(180)))
                    .back(5)
                    .build();
            trajs.put(AMLAuto.trajNames.trajRight1, trajSeq3Right);
            switchCurPose(trajSeq3Right.end());
            return trajSeq3Right;
        }
        else
        {

            TrajectorySequence trajSeq3Right2 = drive.trajectorySequenceBuilder(curPose)
                    .forward(3)
                    .strafeLeft(25)
                    .lineToLinearHeading(new Pose2d(45, -40, Math.toRadians(0)))
                    .build();
            trajs.put(AMLAuto.trajNames.trajRight2, trajSeq3Right2);
            switchCurPose(trajSeq3Right2.end());
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
    public TrajectorySequence moveToBackBoardRight(SampleMecanumDrive drive)
    {
        TrajectorySequence backBoardSetupRight = drive.trajectorySequenceBuilder(curPose)
                .lineToLinearHeading(new Pose2d(45, -25, Math.toRadians(0)))
                .build();
        trajs.put(trajNames.backBoardSetupMiddle, backBoardSetupRight);
        switchCurPose(backBoardSetupRight.end());
        return backBoardSetupRight;
    }
    public TrajectorySequence strafeToDropRight(SampleMecanumDrive drive)
    {
        AprilTagPos aprilTagPos = new AprilTagPos();
        aprilTagPos.setCorrectAprilTag(pos, false);
        TrajectorySequence dropSetupRight = drive.trajectorySequenceBuilder(curPose)
                .strafeRight(aprilTagPos.getDist()[0])
                .build();
        trajs.put(trajNames.strafeToDropRight, dropSetupRight);
        switchCurPose(dropSetupRight.end());
        return dropSetupRight;
    }

    public void drop(DriveTrain dt)
    {
        ElapsedTime outputTime = new ElapsedTime();
        while (outputTime.milliseconds() < 4000) {
            dt.liftarms();
        }
    }

    public void buildInitialTrajs(SampleMecanumDrive drive)
    {
        ////// DECLARING START POS FOR ROBOT
        startPose = new Pose2d(12, -61, Math.toRadians(90));
        switchCurPose(startPose);


        ////// CREATING THE TURN TEST TRAJECTORY SEQUENCE
       /* TrajectorySequence trajSeqTurnTest = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(angle))
                .build();

        */



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
        //end2 = trajSeq3Left.end();

        ////// THIS IS THE TRAJECTORY SEQUENCE NAVIGATING FROM THE PIXEL ON THE LEFT SIDE TOWARDS THE STARING POSITION
        ////// SO THAT WE MAY PARK.
        returnTrajLeft(drive, 2);

        moveToBackBoardLeft(drive);

        strafeToDropLeft(drive);


        ///////////////////////////////////////////////// MIDDLE ///////////////////////////////////////////////////////

        switchCurPose(endOfScans);

        ////// THIS IS THE TRAJECTORY SEQUENCE NAVIGATING TO THE PIXEL IN THE MIDDLE.
        returnTrajMiddle(drive, 1);
       /* TrajectorySequence trajSeq3Middle2 = drive.trajectorySequenceBuilder(trajSeq3Middle.end())
                .turn(Math.toRadians(90))
                .build();

        */

        ////// SETTING end2 TO THE ENDING POSITION OF THE FIRST NAVIGATION.
        // end2 = trajSeq3Middle.end();

        ////// THIS IS THE TRAJECTORY SEQUENCE NAVIGATING FROM THE PIXEL IN THE MIDDLE TOWARDS THE STARING POSITION
        ////// SO THAT WE MAY PARK.
        returnTrajMiddle(drive, 2);

        moveToBackBoardMiddle(drive);

        strafeToDropMiddle(drive);


        ///////////////////////////////////////////////// RIGHT ///////////////////////////////////////////////////////

        switchCurPose(endOfScans);

        ////// THIS IS THE TRAJECTORY SEQUENCE NAVIGATING TO THE PIXEL ON THE LEFT SIDE.
        returnTrajRight(drive, 1);

        ////// SETTING end2 TO THE ENDING POSITION OF THE FIRST NAVIGATION.
        //end2 = trajSeq3Right.end();


        ////// THIS IS THE TRAJECTORY SEQUENCE NAVIGATING FROM THE PIXEL ON THE RIGHT SIDE TOWARDS THE STARING POSITION
        ////// SO THAT WE MAY PARK.
        returnTrajRight(drive, 2);

        moveToBackBoardRight(drive);

        strafeToDropRight(drive);
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

// test again...