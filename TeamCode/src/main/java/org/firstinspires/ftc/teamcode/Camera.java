package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

public class Camera {

    LinearOpMode master;

    VisionPortal.Builder builder;

    public void initVisionPortal(LinearOpMode masterC)
    {
        master = masterC;
        builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(master.hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(DetectTFImages.CAMERA_WIDTH, DetectTFImages.CAMERA_HEIGHT));

        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
    }

    public VisionPortal.Builder getBuilder()
    {
        return builder;
    }

}
