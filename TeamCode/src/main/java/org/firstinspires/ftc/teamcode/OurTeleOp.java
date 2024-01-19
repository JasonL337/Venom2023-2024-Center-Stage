package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "TeleOpV2", group = "Tele")

public class OurTeleOp extends OpMode {
    // Hardware mapped by Hanut (10/24), these are the four wheels Changes made 11/8
    DcMotor frontL;
    DcMotor frontR;
    DcMotor backL;
    DcMotor backR;
    DcMotor liftL;
    Servo armL;
    DcMotor inTake;
    Servo armR;
    Servo boxOutTake;
    Servo boxInTake;
    Servo boxL;
    Servo boxR;
    CRServo processing;
    DcMotor hangL;
    DcMotor hangR;
    Servo dronelauncher;
    // Gyro locking and starting positions
    double lockHeading = 0;

    double playerDegree = 0;
    boolean startLock = true;

    boolean playerRelative = false;
    boolean isBoxOpen = false;
    boolean isBoxDown = false;

    public BNO055IMU imu;
    BNO055IMU.Parameters pars = new BNO055IMU.Parameters();
    Orientation angles;

    ElapsedTime dpadUp = new ElapsedTime();
    ElapsedTime upArmTime = new ElapsedTime();

    ElapsedTime playerRelTime = new ElapsedTime();
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

        pars.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        pars.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(pars);

        // Processing Servo
        processing = hardwareMap.crservo.get("processing");

        // Hanging motors
        hangL = hardwareMap.dcMotor.get("hangL");
        hangR = hardwareMap.dcMotor.get("hangR");

        // Drone Launcher
        dronelauncher = hardwareMap.servo.get("dronelauncher");


        frontL.setDirection(DcMotor.Direction.REVERSE);
        backL.setDirection(DcMotor.Direction.REVERSE);

        backL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        inTake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //boxL.setPosition(0.6);
        //boxR.setPosition(-0.6);


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
        B button pressed and held = Intake power on (includes processing wheel)
        X button pressed = set position of giant arm up/down
        A button pressed = move hanging up
        Y button = move hanging down
        Left joystick = move lift up/down
        Right trigger pressed and held = turtle lift
        Left Trigger Pressed = Drone Launcher
        Right button lower box
        Left button raise box
         */

        runDriver2Methods();

        telemetry.update();


        }

        ///////////////////////////////////////////////////// DRIVER 1 METHODS /////////////////////////////////////////////////
        public void runDriver1Methods()
        {
            runDriveTrain();
            setPlayerRelative();
        }

        public double gyroLockAdder()
        {
            boolean A_button = gamepad1.a;
            double adder = 0;

            if (A_button)
            {
                if (startLock)
                {
                    lockHeading = returnGyroYaw();
                    startLock = false;
                }
                adder = (returnGyroYaw() - lockHeading) * .02;
            }else
            {
                startLock = true;
            }
            return adder;
        }

        public void setPlayerRelative()
        {
            if (gamepad1.y && playerRelTime.milliseconds() > 500)
            {
                if (playerRelative)
                {
                    playerRelative = false;
                }
                else
                {
                    playerRelative = true;
                    playerDegree = returnGyroYaw();
                }
                playerRelTime.reset();
            }
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

            if (!playerRelative) {
                frontL.setPower(((y + x + rx) + adder) * multiplier);
                backL.setPower(((y - x + rx) + adder) * multiplier);
                frontR.setPower(((y - x - rx) - adder) * multiplier);
                backR.setPower(((y + x - rx) - adder) * multiplier);
            }
            else
            {
                double trueDiff = getTrueAngle(playerDegree);
                double joyAngle = Math.toDegrees(Math.atan2(y, x));
                double trueJoy = 90 - joyAngle;
                if (trueJoy > 180)
                {
                    trueJoy = (trueJoy - 360);
                }
                double cos = Math.cos(Math.toRadians(trueJoy - trueDiff));
                double sin = Math.sin(Math.toRadians(trueJoy - trueDiff));
                telemetry.addData("joyAngle", trueJoy);
                telemetry.addData("y", y);
                telemetry.addData("x", x);
                telemetry.addData("true diff:", trueDiff);
                telemetry.addData("true diff:", trueJoy - trueDiff);
                telemetry.addData("cos:", cos);
                telemetry.addData("sin:", sin);
                /*
                frontL.setPower((y * cos + x * sin + rx) * multiplier);
                backL.setPower((y * cos - x * sin + rx) * multiplier);
                frontR.setPower((y * cos - x * sin - rx) * multiplier);
                backR.setPower((y * cos + x * sin - rx) * multiplier);
                */
                double power = Math.min(Math.abs(y) + Math.abs(x), 1);
                frontL.setPower((power * cos + power * sin + rx + adder) * multiplier);
                backL.setPower((power * cos - power * sin + rx + adder) * multiplier);
                frontR.setPower((power * cos - power * sin - rx - adder) * multiplier);
                backR.setPower((power * cos + power * sin - rx - adder) * multiplier);


            }
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
            if (angle - heading <= -180)
            {
                return angle - heading + 360;
            }
            return heading - angle;
        }



        ///////////////////////////////////////////////////// DRIVER 2 METHODS /////////////////////////////////////////////////
        public void runDriver2Methods()
        {
            changeBoxUpDown();
            changeBoxPos();
            changeLiftPos();
            changeIntakePower();
            changeIntakeLift();
            hanging();
            shootdrone();
        }

        public void shootdrone() {
            double left_trigger = gamepad2.left_trigger;
            if (left_trigger > 0.3) {
                dronelauncher.setPosition(.55);
            }
            else {
                dronelauncher.setPosition(.5);
            }
        }
        public void changeBoxUpDown()
        {
            boolean leftBump = gamepad2.left_bumper;
            boolean rightBump = gamepad2.right_bumper;
            if (leftBump) {
                boxL.setPosition(0.9);
                //boxR.setPosition(1);
            }
            else if (rightBump){
                boxL.setPosition(0.4);
                //boxR.setPosition(0.0);
            }
        }

        public void changeBoxPos()
        {
            boolean topDpadButton = gamepad2.dpad_up;
            boolean dPadDown = gamepad2.dpad_down;

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
                //boxInTake.setPosition(1);
            }
            else
            {
                //boxInTake.setPosition(0);
            }

            if (dPadDown) {
                boxOutTake.setPosition(0);
            } else {
                boxOutTake.setPosition(1);
            }
        }

        public void hanging() {

            boolean A_button = gamepad2.a;
            if (A_button) {

                hangL.setPower(-1);
                hangR.setPower(-1);
            }

            boolean Y_button = gamepad2.y;
            if (Y_button) {
                hangL.setPower(-0.3);
                hangR.setPower(-0.3);

            } else {

                hangL.setPower(0);
                hangR.setPower(0);


            }
        }
//luke was here
        public void changeLiftPos()
        {
            double leftJoy = gamepad2.left_stick_y;
            double multiplier = Math.min(1, 1.2 - gamepad2.right_trigger);
            liftL.setPower(leftJoy * multiplier);
        }

        public void changeIntakePower()
        {
            boolean B_button = gamepad2.b;
            if (B_button) {
                inTake.setPower(-1);
                processing.setPower(1);
            } else if (gamepad2.dpad_left) {
                inTake.setPower(1);
                processing.setPower(-1);
            }
            else {
                inTake.setPower(0);
                processing.setPower(0);
            }
        } //hi jason i love u

//i dont love you
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