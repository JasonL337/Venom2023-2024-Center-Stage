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
//@OurTeleOp(name = "Detect", group = "Concept")
public class DetectTFImages {

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

    public static int CAMERA_WIDTH = 1920;

    public static int CAMERA_HEIGHT = 1080;

    public static int CAMERA_ZOOM = 2;

    boolean isWrite = false;

    LinearOpMode master;

    FileWriter myWriter;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;


    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    public void initTfod(LinearOpMode m) {

        master = m;
        ftcDashboard = FtcDashboard.getInstance();
        dashboardTelemetry = ftcDashboard.getTelemetry();

        // Create the TensorFlow processor by using a builder.

        List<String> labels = new ArrayList<>();
        try {
            // new code
            master.telemetry.addLine("I'm in");
            master.telemetry.update();
            AssetManager assetManager = master.hardwareMap.appContext.getAssets();
            InputStream inputStream = assetManager.open("labels.txt");

            //File file = new File("readme.md");
            Scanner scanner = new Scanner(inputStream);
            while (scanner.hasNextLine()) {
                master.telemetry.addLine("looping");
                master.telemetry.update();
                labels.add(scanner.nextLine());
            }
            master.telemetry.addLine("out");
            master.telemetry.update();
            tfod = new TfodProcessor.Builder().setModelFileName("DetectionWithLabels.tflite")
                    .setModelLabels(labels)
                    .setIsModelQuantized(true)
                    .setIsModelTensorFlow2(true)
                    .build();
        }
        catch (IOException e)
        {
            master.telemetry.addData("Could not find labels.txt ", e.toString());
            tfod = new TfodProcessor.Builder().setModelFileName("DetectionWithLabels.tflite")
                    .setIsModelQuantized(true)
                    .setIsModelTensorFlow2(true)
                    .build();
        }




        tfod.setClippingMargins(0, upHeight, rightWidth, 0);
        // New clipping code


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
            builder.setCamera(master.hardwareMap.get(WebcamName.class, "Webcam 1"));
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
            master.telemetry.addLine("No Processor");
        }
    }   // end method initTfod()


    // Returns the list of all the detections from the model.
    public List<Recognition> getTFDetections() {
        return tfod.getRecognitions();

    }

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */



}   // end class
