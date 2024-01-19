package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Distance Sensor Test", group = "Tele")
public class DistanceSensorTest extends OpMode {
    DistanceSensor distanceSensor;


    @Override
    public void init()
    {
        distanceSensor = hardwareMap.get(DistanceSensor.class, "Distance Sensor");
    }
    public void distance()
    {
        double current_dist = distanceSensor.getDistance(DistanceUnit.INCH);
        telemetry.addData("Distance: ", current_dist);
    }
    @Override
    public void loop()
    {
        distance();
    }
}
