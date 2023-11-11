package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RoadRunner.util.Encoder;

@Autonomous(name = "scrimmage auto", group = "scrimmage autos")
public class ScrimmageAuto extends LinearOpMode implements VisionPortalUser, TensorflowProp{
    DcMotor frontL;
    DcMotor frontR;
    DcMotor backL;
    DcMotor backR;
    Servo armL;
    Servo armR;
    ProcessDetections processDetections;
    Encoder parallelEncoder;
    Encoder perpendicularEncoder;
    DcMotor inTake;

    public BNO055IMU imu;
    BNO055IMU.Parameters pars= new BNO055IMU.Parameters();
    Orientation angles;

    Camera camera;
    public void runOpMode()
    {
        frontL = hardwareMap.dcMotor.get("frontLeftMotor");
        frontR = hardwareMap.dcMotor.get("frontRightMotor");
        backL = hardwareMap.dcMotor.get("backLeftMotor");
        backR = hardwareMap.dcMotor.get("backRightMotor");
        armR = hardwareMap.servo.get("armR");
        armL = hardwareMap.servo.get("armL");
        inTake = hardwareMap.dcMotor.get("intake");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        pars.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        pars.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(pars);

        frontL.setDirection(DcMotor.Direction.REVERSE);
        backL.setDirection(DcMotor.Direction.REVERSE);

        backL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        inTake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "liftRightMotor")); // 3 liftRightMotor
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "intake")); // 2 intake
        initVisionPortal();
        initProcessDetections();
        ProcessDetections.pos pos;


        waitForStart();
        ElapsedTime stateTime = new ElapsedTime();
        pos = processDetections.getPos();
        while (opModeIsActive() && !isStopRequested())
        {
            while (stateTime.milliseconds() < 1000)
            {
                frontL.setPower(.5 * ((1000 - stateTime.milliseconds()) / 1000 + .4));
                frontR.setPower(.5 * ((1000 - stateTime.milliseconds()) / 1000 + .4));
                backL.setPower(.5 * ((1000 - stateTime.milliseconds()) / 1000 + .4));
                backR.setPower(.5 * ((1000 - stateTime.milliseconds()) / 1000 + .4));
            }
            frontL.setPower(0);
            frontR.setPower(0);
            backL.setPower(0);
            backR.setPower(0);
            armL.setPosition(1);
            armR.setPosition(0);
            if (pos == ProcessDetections.pos.left)
            {
                while (returnGyroYaw() < 90) {
                    frontL.setPower(-.2);
                    frontR.setPower(.2);
                    backL.setPower(-.2);
                    backR.setPower(.2);
                }
            }
            else if (pos == ProcessDetections.pos.middle)
            {
                while (returnGyroYaw() < 175 && returnGyroYaw() > -175) {
                    frontL.setPower(.2);
                    frontR.setPower(-.2);
                    backL.setPower(.2);
                    backR.setPower(-.2);
                }
            }
            else if (pos == ProcessDetections.pos.right)
            {
                while (returnGyroYaw() > -90) {
                    frontL.setPower(.2);
                    frontR.setPower(-.2);
                    backL.setPower(.2);
                    backR.setPower(-.2);
                }
            }
            frontL.setPower(0);
            frontR.setPower(0);
            backL.setPower(0);
            backR.setPower(0);
            stateTime.reset();
            while (stateTime.milliseconds() < 1000)
            {
                frontL.setPower(0.2);
                frontR.setPower(0.2);
                backL.setPower(0.2);
                backR.setPower(0.2);
            }
            break;

        }


    }
    public void initVisionPortal()
    {
        camera = new Camera();
        camera.initVisionPortal(this);
    }

    public Camera getCamera()
    {
        return camera;
    }

    public double returnGyroYaw()
    {
        angles = imu.getAngularOrientation();
        return angles.firstAngle;
    }

    public void initProcessDetections()
    {
        processDetections = new ProcessDetections();
        processDetections.initialize(this, getCamera());
    }
}
