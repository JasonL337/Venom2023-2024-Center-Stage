package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensorData {
    Rev2mDistanceSensor distanceSensor;
    OpMode parent;


    public void initDistance(OpMode master)
    {
        parent = master;
        distanceSensor = parent.hardwareMap.get(Rev2mDistanceSensor.class, "Distance Sensor");
    }
    public void distance()
    {
        double current_dist = distanceSensor.getDistance(DistanceUnit.INCH);
        parent.telemetry.addData("Distance: ", current_dist);
    }

    public double getDist()
    {
        return distanceSensor.getDistance(DistanceUnit.INCH);
    }
    public void loop()
    {
        distance();
    }
}
