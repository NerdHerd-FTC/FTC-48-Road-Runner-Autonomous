package org.firstinspires.ftc.teamcode.Vision.tensorFlow;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class WebcamPTZControlsTesting extends LinearOpMode {
    //declarations
    public void runOpMode() {
        //Init Code
        TensorFlowInstance TensorFlow = new TensorFlowInstance();

        TensorFlow.InitializeTensorFlow(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            if (isStopRequested()) {return;}
            //Active code
        }
    }


}
