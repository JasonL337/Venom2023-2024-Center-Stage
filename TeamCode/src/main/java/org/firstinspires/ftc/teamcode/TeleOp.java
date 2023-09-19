package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {
    DcMotor frontL;
    DcMotor frontR;
    DcMotor backL;
    DcMotor backR;
    DcMotor liftR;
    DcMotor liftL;
    DcMotor arm;
    @Override
    public void init() {
        frontL = hardwareMap.dcMotor.get("frontLeftMotor");
        frontR = hardwareMap.dcMotor.get("frontRightMotor");
        backL = hardwareMap.dcMotor.get("backLeftMotor");
        backR = hardwareMap.dcMotor.get("backRightMotor");

        liftR = hardwareMap.dcMotor.get("liftR");
        liftL = hardwareMap.dcMotor.get("liftL");
        arm = hardwareMap.dcMotor.get("arm");
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y*Math.abs(gamepad1.left_stick_y);
        double x = gamepad1.left_stick_x*Math.abs(gamepad1.left_stick_y);
        double rx = gamepad1.right_stick_x*Math.abs(gamepad1.right_stick_x);

        double liftPower = gamepad2.left_stick_y*Math.abs(gamepad2.left_stick_y);

        frontL.setPower(y + x + rx);
        backL.setPower(y - x + rx);
        frontR.setPower(y - x - rx);
        backR.setPower(y + x - rx);

        liftL.setPower(liftPower);
        liftR.setPower(liftPower);
    }

    @Override
    public void stop() {

    }
}