package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class DriveTrain {
    public DcMotor backL;
    public DcMotor backR;
    public DcMotor frontL;
    public DcMotor frontR;

    public DcMotor inTake;

    public DcMotor liftL;

    public DcMotor liftR;

    public Servo boxL;

    Servo armR;

    Servo armL;

    Servo boxOutTake;
    CRServo processing;

    //IMU imu;

    //public BNO055IMU imu;
    BNO055IMU.Parameters pars = new BNO055IMU.Parameters();
    Orientation angles;
    LinearOpMode master;




    public DriveTrain(LinearOpMode auto) {
        master = auto;
        backL = auto.hardwareMap.dcMotor.get("backLeftMotor");
        backR = auto.hardwareMap.dcMotor.get("backRightMotor");
        frontR = auto.hardwareMap.dcMotor.get("frontRightMotor");
        frontL = auto.hardwareMap.dcMotor.get("frontLeftMotor");
        inTake = auto.hardwareMap.dcMotor.get("intake");
        armR = auto.hardwareMap.servo.get("armR");
        armL = auto.hardwareMap.servo.get("armL");
        liftL = auto.hardwareMap.dcMotor.get("liftLeftMotor");
        //liftR = auto.hardwareMap.dcMotor.get("liftRightMotor");
        boxOutTake = auto.hardwareMap.servo.get("outtake");
        processing = auto.hardwareMap.crservo.get("processing");
        boxL = auto.hardwareMap.servo.get("boxL");



        backL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        backL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        frontL.setDirection(DcMotor.Direction.REVERSE);
        backL.setDirection(DcMotor.Direction.REVERSE);

        //imu = auto.hardwareMap.get(BNO055IMU.class, "imu");

        pars.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        pars.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //imu.initialize(pars);
    }

        public void turn(double heading, double power)
        {
            double startError = Math.abs(getTrueAngle(heading));
            double trueAngle = getTrueAngle(heading);
            master.telemetry.addData("diff", trueAngle);
            while (trueAngle > 1 || trueAngle < -1)
            {
                master.telemetry.addData("diff", trueAngle);
                trueAngle = getTrueAngle(heading);
                double truePower = power * (trueAngle / startError) + .15 * Math.signum(trueAngle);
                backR.setPower(truePower);
                backL.setPower(-truePower);
                frontL.setPower(-truePower);
                frontR.setPower(truePower);
                master.telemetry.update();
            }
        }

        public void lowerBox()
        {
            boxL.setPosition(0.4);
        }

        public void setMotorPowerDist(double power, double minDist, double curDist)
        {
            while (curDist > minDist)
            {
                backL.setPower(power);
                backR.setPower(power);
                frontL.setPower(power);
                frontR.setPower(power);
            }
            backL.setPower(0);
            backR.setPower(0);
            frontL.setPower(0);
            frontR.setPower(0);
        }

        public double getTrueAngle(double heading)
        {
            double angle = returnGyroYaw();
            if (Math.abs(heading - angle) < 180 )
            {
                return heading - angle;
            }
            if (heading - angle >= 180)
            {
                return  360 - heading - angle;
            }
            return heading - angle;
        }

        public void moveforward(double inches, double power){
            backL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            double distTraveled = 0;
                while (distTraveled < inches) {
                    // this is to slowly decrease the power of the wheel as we reach the distance we want to travel
                    distTraveled = (frontR.getCurrentPosition() + frontL.getCurrentPosition() + backL.getCurrentPosition() + backR.getCurrentPosition())/4;
                    double truePower = ((inches - distTraveled) / inches) * power + .15;
                    backR.setPower(truePower);
                    backL.setPower(truePower);
                    frontL.setPower(truePower);
                    frontR.setPower(truePower);
                }
            // stopping motors
            backR.setPower(0);
            backL.setPower(0);
            frontR.setPower(0);
            frontL.setPower(0);

    }

    // raise the lifts for the outtake
    public void raiselifts(double seconds) {
        ElapsedTime time = new ElapsedTime();
        while (time.milliseconds() < seconds * 1000) {
            liftL.setPower(1);
            liftR.setPower(1);
        }

    }

    public void straferight(double inches, double power){
        backL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double distTraveled = 0;
            while (distTraveled < inches) {
                distTraveled = (Math.abs(frontR.getCurrentPosition()) + Math.abs(frontL.getCurrentPosition()) + Math.abs(backL.getCurrentPosition()) + Math.abs(backR.getCurrentPosition()))/4;
                double truePower = ((inches - distTraveled) / inches) * power + .15;
                backL.setPower(-truePower);
                backR.setPower(truePower);
                frontL.setPower(truePower);
                frontR.setPower(-truePower);
            }

        // stopping motors
        backR.setPower(0);
        backL.setPower(0);
        frontR.setPower(0);
        frontL.setPower(0);

    }

    public void setintakepower(double power) {
        inTake.setPower(power);

    }

    public void liftarms() {
        processing.setPower(-1);
        armL.setPosition(1);
        armR.setPosition(0);

    }

    public void lowerarms(){
        armL.setPosition(0.2);
        armR.setPosition(1);
    }

    public void raiseLifts() {
        ElapsedTime raiselifts = new ElapsedTime();
        while (raiselifts.milliseconds() < 400) {
            liftL.setPower(1);
        }
    }

    public void openBox() {
        ElapsedTime openbox = new ElapsedTime();
        while (openbox.milliseconds() < 200) {
            boxOutTake.setPosition(0);
        }
    }

    public void stopBox() {
        ElapsedTime stopBox = new ElapsedTime();
        while (stopBox.milliseconds() < 1000) {
            boxOutTake.setPosition(1);
        }
    }

    public double returnGyroYaw()
    {
        //angles = imu.getAngularOrientation();
        //return angles.firstAngle;
        return 0;
    }

}
