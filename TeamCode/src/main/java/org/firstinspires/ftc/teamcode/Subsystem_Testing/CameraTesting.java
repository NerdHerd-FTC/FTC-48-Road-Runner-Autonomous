package org.firstinspires.ftc.teamcode.Subsystem_Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Vision.tensorFlow.TensorFlowInstance_Individual_Scanning;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp
@Disabled


public class CameraTesting extends LinearOpMode {
    TensorFlowInstance_Individual_Scanning TensorFlow = new TensorFlowInstance_Individual_Scanning();
    int PanTiltModulatingValue = 10;
    int ZoomModulatingValue = 5;

    int panAdjustment = 0;
    int tiltAdjustment = 0;
    int zoomAdjustment = 0;

    public void ControlCamera() {
        if (gamepad1.dpad_right) {
            TensorFlow.ActuateCameraBy(0, PanTiltModulatingValue, 0);
            panAdjustment += PanTiltModulatingValue;
        }
        if (gamepad1.dpad_left) {
            TensorFlow.ActuateCameraBy(0, -PanTiltModulatingValue, 0);
            panAdjustment -= PanTiltModulatingValue;
        }
        if (gamepad1.dpad_up) {
            TensorFlow.ActuateCameraBy(ZoomModulatingValue, 0, 0);
            zoomAdjustment += ZoomModulatingValue;
        }
        if (gamepad1.dpad_down) {
            TensorFlow.ActuateCameraBy(-ZoomModulatingValue, 0, 0);
            zoomAdjustment -= ZoomModulatingValue;
        }
        if (gamepad1.y) {
            TensorFlow.ActuateCameraBy(0, 0, PanTiltModulatingValue);
            tiltAdjustment += PanTiltModulatingValue;
        }
        if (gamepad1.a) {
            TensorFlow.ActuateCameraBy(0,0, -PanTiltModulatingValue);
            tiltAdjustment -= PanTiltModulatingValue;
        }
        if (gamepad1.back) {
            TensorFlow.ActuateCameraTo(0,0,0);
            panAdjustment = 0;
            tiltAdjustment = 0;
            zoomAdjustment = 0;
        }

        telemetry.addData("Pan: ", panAdjustment);
        telemetry.addData("Tilt: ", tiltAdjustment);
        telemetry.addData("Zoom: ", zoomAdjustment);
        telemetry.update();
    }

    public void runOpMode() {

        final TensorFlowInstance_Individual_Scanning.CameraStreamProcessor processor = new TensorFlowInstance_Individual_Scanning.CameraStreamProcessor();


        TensorFlow.IntitializeTensorFlow(hardwareMap, processor);

        while (TensorFlow.visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            if (isStopRequested()) return;
            sleep(20);

        }

        while (opModeInInit()) {
            //TensorFlow.ActuateCameraBy(200,0,0);
            ControlCamera();
        }


        waitForStart();

        while (opModeIsActive()) {

           ControlCamera();


        }








    }


}
