package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class GetAprilTags {

    AprilTagProcessor aprilTag;

    Camera camera;

    VisionPortal visionPortal;

    LinearOpMode master;

    public void initAprilTagDetections(LinearOpMode masterC, Camera camera) {
        this.camera = camera;
        master = masterC;
        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)

                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Create the vision portal by using a builder.
        //VisionPortal.Builder builder = camera.getBuilder();

        // Set the camera (webcam vs. built-in RC phone camera).
        //builder.setCamera(master.hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(DetectTFImages.CAMERA_WIDTH, DetectTFImages.CAMERA_HEIGHT));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        this.camera.getBuilder().addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.

        //visionPortal = camera.getBuilder().build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()

    public List<AprilTagDetection> getDetections()
    {
        return aprilTag.getDetections();
    }

    public void setProcessor(boolean active)
    {
        camera.getBuilder().build().setProcessorEnabled(aprilTag, active);
    }
}
