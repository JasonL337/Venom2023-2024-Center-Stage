package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    Servo boxOutTake;
    Servo boxInTake;
    Servo boxL;
    Servo boxR;

    // Gyro locking and starting positions
    double lockHeading = 0;
    boolean startLock = true;
    boolean isBoxOpen = false;
    boolean isBoxDown = false;

    public BNO055IMU imu;
    BNO055IMU.Parameters pars = new BNO055IMU.Parameters();
    Orientation angles;

    ElapsedTime dpadUp = new ElapsedTime();
    ElapsedTime upArmTime = new ElapsedTime();
    boolean armUp = false;
    @Override
    public void init() {

        final double ARM_STARTING = 0.0; // starting position for the servo on the arm
        final double ARM_MINIMUM = 0.0; // sets the minimum rotation of the servo to be 0
        final double ARM_MAXIMUM = 1; // sets the maximum value of the rotation to be 1

        // Drive train motors
        frontL = hardwareMap.dcMotor.get("frontLeftMotor");
        frontR = hardwareMap.dcMotor.get("frontRightMotor");
        backL = hardwareMap.dcMotor.get("backLeftMotor");
        backR = hardwareMap.dcMotor.get("backRightMotor");

        // Intake motor
        inTake = hardwareMap.dcMotor.get("intake");

        // Lift motors
        liftL = hardwareMap.dcMotor.get("liftLeftMotor");
        liftR = hardwareMap.dcMotor.get("liftRightMotor");

        // Left and right sides of giant arm
        armR = hardwareMap.servo.get("armR");
        armL = hardwareMap.servo.get("armL");

        // Not sure what these are
        boxInTake = hardwareMap.servo.get("boxIntake");
        boxOutTake = hardwareMap.servo.get("outtake");

        // Left and right sides of box that change its angle
        boxL = hardwareMap.servo.get("boxL");
        boxR = hardwareMap.servo.get("boxR");

        // gyro
        imu = hardwareMap.get(BNO055IMU.class, "imu");




        frontL.setDirection(DcMotor.Direction.REVERSE);
        backL.setDirection(DcMotor.Direction.REVERSE);

        backL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        inTake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public double returnGyroYaw()
    {
        angles = imu.getAngularOrientation();
        return angles.firstAngle;
    }

    @Override
    public void loop() {

        /* Gamepad 1:
        RC control left/right joystick
        Left trigger pressed and held = turbo
        Right trigger pressed and held = turtle
        A button pressed and held = gyro lock
         */

        runDriver1Methods();


        /* Gamepad 2:
        B button pressed and held = Intake power on
        X button pressed = set position of giant arm up/down
        Left joystick = move lift up/down
        Right trigger pressed and held = turtle lift
        Right button lower box
        Left button raise box
         */

        runDriver2Methods();


        }

        ///////////////////////////////////////////////////// DRIVER 1 METHODS /////////////////////////////////////////////////
        public void runDriver1Methods()
        {
            runDriveTrain();
        }

        public double gyroLockAdder()
        {
            boolean A_button = gamepad2.a;
            double adder = 0;

            if (A_button)
            {
                if (startLock)
                {
                    lockHeading = returnGyroYaw();
                    startLock = false;
                }
                adder = returnGyroYaw() - lockHeading * .02;
            }
            return adder;
        }

        public void runDriveTrain()
        {
            double y = -gamepad1.left_stick_y*Math.abs(gamepad1.left_stick_y);
            double x = gamepad1.left_stick_x*Math.abs(gamepad1.left_stick_x);
            double rx = gamepad1.right_stick_x*Math.abs(gamepad1.right_stick_x);
            double adder = gyroLockAdder();

            double multiplier = .75;
            if (gamepad1.left_trigger > .1)
                multiplier = 1;
            if (gamepad1.right_trigger > .1)
                multiplier = .25;

            frontL.setPower(((y + x + rx) - adder) * multiplier);
            backL.setPower(((y - x + rx) - adder) * multiplier);
            frontR.setPower(((y - x - rx) + adder) * multiplier);
            backR.setPower(((y + x - rx) + adder) * multiplier);
        }



        ///////////////////////////////////////////////////// DRIVER 2 METHODS /////////////////////////////////////////////////
        public void runDriver2Methods()
        {
            changeBoxUpDown();
            changeBoxPos();
            changeLiftPos();
            changeIntakePower();
            changeIntakeLift();
        }

        public void changeBoxUpDown()
        {
            boolean leftBump = gamepad2.left_bumper;
            boolean rightBump = gamepad2.right_bumper;
            if (leftBump) {
                boxL.setPosition(1);
                boxR.setPosition(0);
            }
            if (rightBump) {
                boxL.setPosition(0);
                boxR.setPosition(1);
            }
        }


        public void changeBoxPos()
        {
            boolean topDpadButton = gamepad2.dpad_up;
            boolean Y_button = gamepad2.y;

            if (topDpadButton && dpadUp.milliseconds() > 500) {
                if (isBoxOpen) {
                    isBoxOpen = false;
                } else {
                    isBoxOpen = true;
                }
                dpadUp.reset();
            }
            if (isBoxOpen)
            {
                boxInTake.setPosition(1);
            }
            else
            {
                boxInTake.setPosition(0);
            }

            if (Y_button) {
                boxOutTake.setPosition(1);
            } else {
                boxOutTake.setPosition(0);
            }
        }

        public void changeLiftPos()
        {
            double leftJoy = gamepad2.left_stick_y;
            double multiplier = Math.min(1, 1.2 - gamepad2.right_trigger);
            liftL.setPower(leftJoy * multiplier);
            liftR.setPower(leftJoy * multiplier);
        }

        public void changeIntakePower()
        {
            boolean B_button = gamepad2.b;
            if (B_button) {
                inTake.setPower(-1);
            } else {
                inTake.setPower(0);
            }
        }

        public void changeIntakeLift()
        {
            boolean X_button = gamepad2.x;
            if (X_button && upArmTime.milliseconds() > 500) {
                if (armUp)
                {
                    armUp = false;
                    armL.setPosition(0.2);
                    armR.setPosition(1);
                }
                else
                {
                    armUp = true;
                    armL.setPosition(1);
                    armR.setPosition(0);
                }
                upArmTime.reset();
            }
        }





    @Override
    public void stop() {

    }
}