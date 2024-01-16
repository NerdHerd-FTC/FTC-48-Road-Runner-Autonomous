package org.firstinspires.ftc.teamcode.Subsystem_Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp

public class OdometryPodTester extends LinearOpMode {
    @Override
    public void runOpMode() {

        DcMotor LeftOdometryPod = hardwareMap.get(DcMotor.class, "motorBL");
        DcMotor RightOdometryPod = hardwareMap.get(DcMotor.class, "motorBR");
        DcMotor FrontOdometryPod = hardwareMap.get(DcMotor.class, "motorFL");


        LeftOdometryPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightOdometryPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontOdometryPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftOdometryPod.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontOdometryPod.setDirection(DcMotorSimple.Direction.REVERSE);



        waitForStart();

        while (opModeIsActive()) {
            if (isStopRequested()) return;

            telemetry.addData("Left Odometry Pod: ", LeftOdometryPod.getCurrentPosition());
            telemetry.addData("Right Odometry Pod: ", RightOdometryPod.getCurrentPosition());
            telemetry.addData("Front Odometry Pod: ", FrontOdometryPod.getCurrentPosition());
            telemetry.update();

        }



    }
}

