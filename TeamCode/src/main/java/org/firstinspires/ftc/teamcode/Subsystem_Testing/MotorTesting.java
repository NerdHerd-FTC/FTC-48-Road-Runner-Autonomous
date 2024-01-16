package org.firstinspires.ftc.teamcode.Subsystem_Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp

public class MotorTesting extends LinearOpMode {
    @Override
    public void runOpMode() {

        DcMotor Motor = hardwareMap.get(DcMotor.class, "motorBL");

        waitForStart();

        while (opModeIsActive()) {
            if (isStopRequested()) return;

            Motor.setPower(0);

            Motor.setPower(gamepad1.right_trigger);
        }



    }
}

