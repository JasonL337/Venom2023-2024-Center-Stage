/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static java.lang.Double.parseDouble;

import android.content.res.AssetManager;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.tensorflow.lite.Tensor;
import org.tensorflow.lite.support.common.FileUtil;
import org.tensorflow.lite.support.label.TensorLabel;
import org.tensorflow.lite.task.vision.detector.Detection;
import org.tensorflow.lite.task.vision.segmenter.OutputType;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "Concept: TensorFlow Object DetectionV2", group = "Concept")
public class DetectTFImages extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    ElapsedTime increaseTime = new ElapsedTime();

    FtcDashboard ftcDashboard;

    Telemetry dashboardTelemetry;

    int rightWidth = 0;
    int upHeight = 400;

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    public static final int CAMERA_WIDTH = 1920;

    public static final int CAMERA_HEIGHT = 1080;

    public static final int CAMERA_ZOOM = 2;

    boolean isWrite = false;

    FileWriter myWriter;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        //  while (!opModeIsActive()) {
        initTfod();

        dashboardTelemetry.addLine("TESTING ONLY TO FTC DASHBOARD");
        dashboardTelemetry.update();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        // }
        waitForStart();
        CameraStreamSource cameraStreamSource;

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetryTfod();

                // Push telemetry to the Driver Station.
                telemetry.update();
                //dashboardTelemetry.update();
                //FtcDashboard.getInstance().startCameraStream(visionPortal.str, 0);

                // Save CPU resources; can resume streaming when needed.
                //  if (gamepad1.dpad_down) {
                //      visionPortal.stopStreaming();
                //  } else if (gamepad1.dpad_up) {
                visionPortal.resumeStreaming();
                //  }

                // Share the CPU.
                sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        ftcDashboard = FtcDashboard.getInstance();
        dashboardTelemetry = ftcDashboard.getTelemetry();

        try {
            myWriter = new FileWriter("Locations.txt");
            myWriter.write("");
            myWriter.close();
        } catch (IOException e) {
            telemetry.addData("error: ", e.toString());
        }

        // Create the TensorFlow processor by using a builder.

        List<String> labels = new ArrayList<>();
        try {
            // new code
            AssetManager assetManager = hardwareMap.appContext.getAssets();
            InputStream inputStream = assetManager.open("labels.txt");

            //File file = new File("readme.md");
            Scanner scanner = new Scanner(inputStream);
            while (scanner.hasNextLine()) {
                labels.add(scanner.nextLine());
            }
            tfod = new TfodProcessor.Builder().setModelFileName("DetectionWithLabels.tflite")
                    .setModelLabels(labels)
                    .setIsModelQuantized(true)
                    .setIsModelTensorFlow2(true)
                    .build();
        }
        catch (IOException e)
        {
            telemetry.addData("Could not find labels.txt ", e.toString());
            tfod = new TfodProcessor.Builder().setModelFileName("DetectionWithLabels.tflite")
                    .setIsModelQuantized(true)
                    .setIsModelTensorFlow2(true)
                    .build();
        }




        tfod.setClippingMargins(0, upHeight, rightWidth, 0);
        // New clipping code
        if (gamepad1.dpad_right) {
            rightWidth += 10;
            tfod.setClippingMargins(0, upHeight, rightWidth, 0);
        }
        else if (gamepad1.dpad_left)
        {
            rightWidth -= 10;
            tfod.setClippingMargins(0, upHeight, rightWidth, 0);
        }
        else if (gamepad1.dpad_up)
        {
            upHeight += 10;
            tfod.setClippingMargins(0, upHeight, rightWidth, 0);
        }
        else if (gamepad1.dpad_down)
        {
            upHeight -= 10;
            tfod.setClippingMargins(0, upHeight, rightWidth, 0);
        }


        //File tfliteModel = new File("***.tflite");
        //Interpreter tflite = new Interpreter(tfliteModel);  // Load model.

        // Use setModelAssetName() if the TF Model is built in as an asset.
        // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
        //.setModelAssetName(TFOD_MODEL_ASSET)
        //.setModelFileName(TFOD_MODEL_FILE)

        //.setModelLabels(LABELS)
        //.setIsModelTensorFlow2(true)
        //.setIsModelQuantized(true)
        //.setModelInputSize(300)
        //.setModelAspectRatio(16.0 / 9.0)


        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        if (tfod != null) {
            builder.addProcessor(tfod);

            // Build the Vision Portal, using the above settings.
            visionPortal = builder.build();

            // Set confidence threshold for TFOD recognitions, at any time.
            tfod.setMinResultConfidence(0.1f);
            // From camera, left is positive, down is positive
            tfod.setZoom(CAMERA_ZOOM);

            // Disable or re-enable the TFOD processor at any time.
            //visionPortal.setProcessorEnabled(tfod, true);
        }
        else
        {
            telemetry.addLine("No Processor");
        }
    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        //List<String> labels = FileUtil.loadLabels(context, "labels.txt");
        //public TensorLabel tensorLabel = new TensorLabel();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        if (gamepad1.a)
        {
            isWrite = false;
        }
        else
        {
            isWrite = false;
        }

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            if (!recognition.getLabel().equals("person")) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2;
                double y = (recognition.getTop() + recognition.getBottom()) / 2;

                if (recognition.getLabel().equals("parking meter"))
                {
                    dashboardTelemetry.addData("Labelll: ", recognition.getLabel());
                    dashboardTelemetry.addData("- Position", "%.4f / %.0f", (x - (CAMERA_WIDTH / (CAMERA_ZOOM * 2))) / ((double) CAMERA_WIDTH / 2), y);
                    dashboardTelemetry.update();
                }

                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                telemetry.addData("- Position", "%.0f / %.0f", x, y);
                telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
                if (isWrite) {
                    try {
                        myWriter.append("\n\tImage" + String.format("%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100) + "\n");
                        myWriter.append("position: " + String.format("%.0f / %.0f", x, y));
                        myWriter.close();
                    } catch (IOException e) {
                        telemetry.addData("error: ", e.toString());
                    }
                }
            }
        }
        // end for() loop

    }   // end method telemetryTfod()

}   // end class
