package org.firstinspires.ftc.teamcode.Vision.tensorFlow;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
public class TensorFlowInstance {

    public WebcamName Webcam;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "old/20231203_FTC_Red_And_Blue_Crown_Props_Tensor_Flow_Model.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "Crown",
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    public TfodProcessor tfod;

    private HardwareMap hardwareMap;

    /**
     * The variable to store our instance of the vision portal.
     */
    public VisionPortal visionPortal;

    public String DetectProps() {

        int loop = 0;
        int cycles = 20;

        String PropLocation = "not scanned";

        while (loop <= cycles) {
            List<Recognition> currentRecognitions = tfod.getRecognitions();

            for (Recognition recognition : currentRecognitions) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
                double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                
                if (x < 640) {
                    PropLocation = "left";
                    break;
                } else if (x >= 640 && x < 1280) {
                    PropLocation = "center";
                    break;
                } else if (x >= 1280) {
                    PropLocation = "right";
                    break;
                }
            }
        }

        return PropLocation;
    }

    public void SetWebcamStreamStatus(String Action) {
        if (Action == "start") {
            visionPortal.resumeStreaming();
        } else if (Action == "stop") {
            visionPortal.stopStreaming();
        }
    }

    public void InitializeTensorFlow(HardwareMap robotHardware) {

        hardwareMap = robotHardware;

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();



        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            Webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
            builder.setCamera(Webcam);
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1920, 1080));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.77f);


        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method InitializeTensorFlowInstance()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */

}   // end class