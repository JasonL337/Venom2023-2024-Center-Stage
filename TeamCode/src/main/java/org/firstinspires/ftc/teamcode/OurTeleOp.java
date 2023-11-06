package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "TeleOpV2", group = "Tele")

public class OurTeleOp extends OpMode {
    // Hardware mapped by Hanut (10/24), these are the four wheels
    DcMotor frontL;
    DcMotor frontR;
    DcMotor backL;
    DcMotor backR;
    // These motors are yet to be hardware mapped
    DcMotor liftR;
    DcMotor liftL;
    Servo armL;
    DcMotor inTake;

    Servo armR;

    double lockHeading = 0;
    boolean startLock = true;

    public BNO055IMU imu;
    BNO055IMU.Parameters pars = new BNO055IMU.Parameters();
    Orientation angles;
    @Override
    public void init() {

        final double ARM_STARTING = 0.0; // starting position for the servo on the arm
        final double ARM_MINIMUM = 0.0; // sets the minimum rotation of the servo to be 0
        final double ARM_MAXIMUM = 1; // sets the maximum value of the rotation to be 1

        frontL = hardwareMap.dcMotor.get("frontLeftMotor");
        frontR = hardwareMap.dcMotor.get("frontRightMotor");
        backL = hardwareMap.dcMotor.get("backLeftMotor");
        backR = hardwareMap.dcMotor.get("backRightMotor");
        inTake = hardwareMap.dcMotor.get("intake");
        armR = hardwareMap.servo.get("armR");
        armL = hardwareMap.servo.get("armL");


        /* liftR = hardwareMap.dcMotor.get("liftR");
        liftL = hardwareMap.dcMotor.get("liftL"); */



        frontL.setDirection(DcMotor.Direction.REVERSE);
        backL.setDirection(DcMotor.Direction.REVERSE);

        backL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        inTake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    public double returnGyroYaw()
    {
        angles = imu.getAngularOrientation();
        return angles.firstAngle;
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y*Math.abs(gamepad1.left_stick_y);
        double x = gamepad1.left_stick_x*Math.abs(gamepad1.left_stick_x);
        double rx = gamepad1.right_stick_x*Math.abs(gamepad1.right_stick_x);
        double left_trigger = gamepad1.left_trigger;
        double right_trigger = gamepad1.right_trigger;
        boolean B_button = gamepad2.b;
        boolean X_button = gamepad2.x;
        boolean A_button = gamepad1.a;

        double adder = 0;

        /* double liftPower = gamepad2.left_stick_y*Math.abs(gamepad2.left_stick_y); */



        if (B_button == true) {
            inTake.setPower(1);
        } else {
            inTake.setPower(0);
        }

        if (X_button == true) {
            armL.setPosition(0);
            armR.setPosition(1);
        } else {
            armL.setPosition(1);
            armR.setPosition(0);
        }

        if (A_button)
        {
            if (startLock)
            {
                lockHeading = returnGyroYaw();
                startLock = false;
            }
            adder = returnGyroYaw() - lockHeading * .02;
        }

        frontL.setPower((y + x + rx) - adder);
        backL.setPower((y - x + rx) - adder);
        frontR.setPower((y - x - rx) + adder);
        backR.setPower((y + x - rx) + adder);

        /* if (left_trigger == 1) {

            double multiplier = 1;

            frontL.setPower((y + x + rx) * multiplier);
            frontR.setPower((y - x + rx) * multiplier);
            backL.setPower((y - x - rx) * multiplier);
            backR.setPower((y + x - rx) * multiplier);

        } if (right_trigger == 1) {

            double multiplier = 0.25;

            frontL.setPower((y + x + rx) * multiplier);
            frontR.setPower((y - x + rx) * multiplier);
            backL.setPower((y - x - rx) * multiplier);
            backR.setPower((y + x - rx) * multiplier);

        } else {*/

            /*double multiplier = 1;

            frontL.setPower((y + x + rx) * multiplier);
            frontR.setPower((y - x - rx) * multiplier);
            backL.setPower((y - x + rx) * multiplier);
            backR.setPower((y + x - rx) * multiplier);*/
        }

        /* liftL.setPower(liftPower);
        liftR.setPower(liftPower); */

        /* arm.setPower(armPower); */




    @Override
    public void stop() {

    }
}