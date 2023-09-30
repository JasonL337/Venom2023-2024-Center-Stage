package org.firstinspires.ftc.teamcode;

import android.content.Context;

import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.tensorflow.lite.task.core.BaseOptions;
import org.tensorflow.lite.task.vision.detector.Detection;
import org.tensorflow.lite.task.vision.detector.ObjectDetector;

import java.util.List;

public class TensorFlowTest {
    // Initialization
    ObjectDetector.ObjectDetectorOptions options =
            ObjectDetector.ObjectDetectorOptions.builder()
                    .setBaseOptions(BaseOptions.builder().useGpu().build())
                    .setMaxResults(1)
                    .build();
    private Context context;
   // ObjectDetector objectDetector =
     //       ObjectDetector.createFromFileAndOptions(
        //            context, new String("tensorflowssd-mobilenet_v12.tflite"), options);

    // Run inference
    //List<Detection> results = objectDetector.detect(image);
}
