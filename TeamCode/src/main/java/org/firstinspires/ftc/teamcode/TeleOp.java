package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {
    // Hardware mapped by Hanut (10/24), these are the four wheels
    DcMotor frontL;
    DcMotor frontR;
    DcMotor backL;
    DcMotor backR;
    // These motors are yet to be hardware mapped
    DcMotor liftR;
    DcMotor liftL;
    DcMotor arm;
    @Override
    public void init() {

        frontL = hardwareMap.dcMotor.get("frontLeftMotor");
        frontR = hardwareMap.dcMotor.get("frontRightMotor");
        backL = hardwareMap.dcMotor.get("backLeftMotor");
        backR = hardwareMap.dcMotor.get("backRightMotor");

        /* liftR = hardwareMap.dcMotor.get("liftR");
        liftL = hardwareMap.dcMotor.get("liftL");
        arm = hardwareMap.dcMotor.get("arm"); */

        frontL.setDirection(DcMotor.Direction.REVERSE);
        backL.setDirection(DcMotor.Direction.REVERSE);

        backL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y*Math.abs(gamepad1.left_stick_y);
        double x = gamepad1.left_stick_x*Math.abs(gamepad1.left_stick_x);
        double rx = gamepad1.right_stick_x*Math.abs(gamepad1.right_stick_x);
        double left_trigger = gamepad1.left_trigger;
        double right_trigger = gamepad1.right_trigger;

        /* double liftPower = gamepad2.left_stick_y*Math.abs(gamepad2.left_stick_y); */

        double armPower = gamepad2.right_stick_x*Math.abs(gamepad2.right_stick_x);

        frontL.setPower(y + x + rx);
        backL.setPower(y - x + rx);
        frontR.setPower(y - x - rx);
        backR.setPower(y + x - rx);

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

            double multiplier = 1;
            
            frontL.setPower((y + x + rx) * multiplier);
            frontR.setPower((y - x + rx) * multiplier);
            backL.setPower((y - x - rx) * multiplier);
            backR.setPower((y + x - rx) * multiplier);
        }

        /* liftL.setPower(liftPower);
        liftR.setPower(liftPower); */

        /* arm.setPower(armPower); */


    @Override
    public void stop() {

    }
}