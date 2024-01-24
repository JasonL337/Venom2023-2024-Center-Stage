package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
@Autonomous(name = "Red Auto Close Side Backboard", group = "Comp Autos")
@Config
public class AMLAutoBackboard extends LinearOpMode implements VisionPortalUser, TensorflowProp{
    Camera camera;
    ProcessDetections processDetections;
    ProcessDetections.pos pos;
    DistanceSensor distanceSensor; // We define our distance sensor

    public static int angle = 0;
    public static boolean turnTest = false;
    public double distance = 0;



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

        // Hardware mapping our distance sensor
        distanceSensor = hardwareMap.get(DistanceSensor.class, "Distance Sensor");


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
        Pose2d end3;
        end3 = trajSeq3Left2.end();
        TrajectorySequence backBoardSetupLeft = moveToBackBoardLeft(drive, end3);
        Pose2d end4;
        end4 = backBoardSetupLeft.end();
        TrajectorySequence dropSetupLeft = strafeToDropLeft(drive, end4);


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
        end3 = trajSeq3Middle3.end();
        TrajectorySequence backBoardSetupMiddle = moveToBackBoardMiddle(drive, end3);
        end4 = backBoardSetupMiddle.end();
        TrajectorySequence dropSetupMiddle = strafeToDropMiddle(drive, end4);

        ///////////////////////////////////////////////// RIGHT ///////////////////////////////////////////////////////


        ////// THIS IS THE TRAJECTORY SEQUENCE NAVIGATING TO THE PIXEL ON THE LEFT SIDE.
        TrajectorySequence trajSeq3Right = returnTrajRight(drive, end, 1);

        ////// SETTING end2 TO THE ENDING POSITION OF THE FIRST NAVIGATION.
        end2 = trajSeq3Right.end();


        ////// THIS IS THE TRAJECTORY SEQUENCE NAVIGATING FROM THE PIXEL ON THE RIGHT SIDE TOWARDS THE STARING POSITION
        ////// SO THAT WE MAY PARK.
        TrajectorySequence trajSeq3Right2 = returnTrajRight(drive, end2, 2);
        end3 = trajSeq3Right2.end();
        TrajectorySequence backBoardSetupRight = moveToBackBoardRight(drive, end3);
        end4 = backBoardSetupLeft.end();
        TrajectorySequence dropSetupRight = strafeToDropRight(drive, end4);

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
                    drive.followTrajectorySequence(backBoardSetupLeft);
                    currDist(); // sees how far we are from the backboard in inches
                    while (distance > 1.5) // we go forward until we are an inch away from the backboard
                        {
                            // we move forward at a moderate pace
                            dt.frontR.setPower(0.3);
                            dt.frontL.setPower(0.3);
                            dt.backR.setPower(0.3);
                            dt.backL.setPower(0.3);
                            currDist(); // we keep on checking our distance to see if we have gotten closer
                        }
                    dt.boxL.setPosition(0.5);
                    drive.followTrajectorySequence(dropSetupLeft);
                    dt.raiseLifts();
                    sleep((500));
                    dt.openBox();
                    endPlacePos = dropSetupLeft.end();
                }
                if (pos == ProcessDetections.pos.middle) {
                    drive.followTrajectorySequence(trajSeq3Middle);
                    //drive.followTrajectorySequence(trajSeq3Middle2);
                    drop(dt);
                    drive.followTrajectorySequence(trajSeq3Middle3);
                    drive.followTrajectorySequence(backBoardSetupMiddle);
                    currDist();
                    while (distance > 1.5)
                        {
                            dt.frontR.setPower(0.3);
                            dt.frontL.setPower(0.3);
                            dt.backR.setPower(0.3);
                            dt.backL.setPower(0.3);
                            currDist();
                        }
                    dt.boxL.setPosition(0.5);
                    drive.followTrajectorySequence(dropSetupMiddle);
                    dt.raiseLifts();
                    sleep((500));
                    dt.openBox();
                    endPlacePos = dropSetupMiddle.end();
                }
                if (pos == ProcessDetections.pos.right) {
                    drive.followTrajectorySequence(trajSeq3Right);
                    drop(dt);
                    drive.followTrajectorySequence(trajSeq3Right2);
                    drive.followTrajectorySequence(backBoardSetupRight);
                    currDist();
                    while (distance > 1.5)
                        {
                            dt.frontR.setPower(0.3);
                            dt.frontL.setPower(0.3);
                            dt.backR.setPower(0.3);
                            dt.backL.setPower(0.3);
                            currDist();
                        }
                    dt.boxL.setPosition(0.5);
                    drive.followTrajectorySequence(dropSetupRight);
                    dt.raiseLifts();
                    sleep((500));
                    dt.openBox();
                    endPlacePos = dropSetupRight.end();
                }


                ///////

                    /*TrajectorySequence trajSeq4Right = drive.trajectorySequenceBuilder(endPlacePos)
                            .forward(2)
                            //.turn(Math.toRadians(0))
                            .lineToLinearHeading(new Pose2d(60, -70, Math.toRadians(0)))
                            .build();
                    drive.followTrajectorySequence(trajSeq4Right);
                    //drive.followTrajectorySequence(trajSeq3);*/
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
                .strafeLeft(13)
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
                    .lineToLinearHeading(new Pose2d(end.getX(), end.getY() + 26, Math.toRadians(0)))
                    .back(7)
                    .build();
            return trajSeq3Left;
        }
        else
        {
            TrajectorySequence trajSeq3Left2 = drive.trajectorySequenceBuilder(end)
                    .forward(12)
                    //.forward(16)
                    //.turn(Math.toRadians(-90))
                    .lineToLinearHeading(new Pose2d(30, -40, Math.toRadians(0)))
                    //.back(3)
                    .build();
            return trajSeq3Left2;
        }
    }

    public TrajectorySequence moveToBackBoardLeft(SampleMecanumDrive drive, Pose2d end)
    {
        TrajectorySequence backBoardSetupLeft = drive.trajectorySequenceBuilder(end)
                .lineToLinearHeading(new Pose2d(30, -25, Math.toRadians(0)))
                .build();
        return backBoardSetupLeft;
    }

    public TrajectorySequence strafeToDropLeft(SampleMecanumDrive drive, Pose2d end)
    {
        TrajectorySequence dropSetupLeft = drive.trajectorySequenceBuilder(end)
                .strafeRight(1)
                .build();
        return dropSetupLeft;
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
                    .back(4)
                    .build();
            return trajSeq3Middle;
        }
        else
        {
            TrajectorySequence trajSeq3Middle3 = drive.trajectorySequenceBuilder(end)
                    .forward(7)
                    //.turn(Math.toRadians(90))
                    //.turn(Math.toRadians(90))
                    .lineToLinearHeading(new Pose2d(30, -40, Math.toRadians(0)))
                    //.back(3)
                    .build();
            return trajSeq3Middle3;
        }
    }
    public TrajectorySequence moveToBackBoardMiddle(SampleMecanumDrive drive, Pose2d end)
    {
        TrajectorySequence backBoardSetupMiddle = drive.trajectorySequenceBuilder(end)
                .lineToLinearHeading(new Pose2d(30, -25, Math.toRadians(0)))
                .build();
        return backBoardSetupMiddle;
    }

    public TrajectorySequence strafeToDropMiddle(SampleMecanumDrive drive, Pose2d end)
    {
        TrajectorySequence dropSetupMiddle = drive.trajectorySequenceBuilder(end)
                .strafeRight(4)
                .build();
        return dropSetupMiddle;
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
                    .forward(3)
                    .strafeLeft(25)
                    .lineToLinearHeading(new Pose2d(45, -40, Math.toRadians(0)))
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
    public TrajectorySequence moveToBackBoardRight(SampleMecanumDrive drive, Pose2d end)
    {
        TrajectorySequence backBoardSetupRight = drive.trajectorySequenceBuilder(end)
                .lineToLinearHeading(new Pose2d(45, -25, Math.toRadians(0)))
                .build();
        return backBoardSetupRight;
    }
    public TrajectorySequence strafeToDropRight(SampleMecanumDrive drive, Pose2d end)
    {
        TrajectorySequence dropSetupRight = drive.trajectorySequenceBuilder(end)
                .strafeRight(8)
                .build();
        return dropSetupRight;
    }
    public void currDist()
    {
        distance = distanceSensor.getDistance(DistanceUnit.INCH);
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
