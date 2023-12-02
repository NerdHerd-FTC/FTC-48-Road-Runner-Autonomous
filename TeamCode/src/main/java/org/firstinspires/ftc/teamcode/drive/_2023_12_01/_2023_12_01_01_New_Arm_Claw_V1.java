package org.firstinspires.ftc.teamcode.drive._2023_12_01;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "V1 New Arm Claw (Drivebase)", group = "2023-12-01")
public class _2023_12_01_01_New_Arm_Claw_V1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("motorFL");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("motorBL");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("motorFR");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("motorBR");

        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor arm = hardwareMap.dcMotor.get("arm");
        Servo clawTop = hardwareMap.servo.get("clawTop");
        Servo clawBottom = hardwareMap.servo.get("clawBottom");

        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        clawTop.setDirection(Servo.Direction.FORWARD);
        clawBottom.setDirection(Servo.Direction.REVERSE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        clawTop.scaleRange(0, 1);
        clawBottom.scaleRange(0, 1);

        clawBottom.setPosition(0);
        clawTop.setPosition(0);

        int liftTargetPosition = 0;
        double openClaw = 0.4; //increase to open more
        double closeClaw = 0.2; //decrease to close more
        //boolean armDown = true;

        waitForStart();

        clawTop.setPosition(openClaw);
        clawBottom.setPosition(openClaw);

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            x = x * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = Math.round(((y + x + rx) / denominator));
            double backLeftPower = Math.round(((y - x + rx) / denominator));
            double frontRightPower = Math.round(((y - x - rx) / denominator));
            double backRightPower = Math.round(((y + x - rx) / denominator));

            frontLeftMotor.setPower(frontLeftPower * 0.8);
            backLeftMotor.setPower(backLeftPower * 0.8);
            frontRightMotor.setPower(frontRightPower * 0.8);
            backRightMotor.setPower(backRightPower * 0.8);

            if (gamepad1.right_trigger > 0) {
                liftTargetPosition += 1;
            } else if (gamepad1.left_trigger > 0) {
                liftTargetPosition -= 1;
            }

            /*
            if (gamepad1.right_bumper) {
                for (int liftPos = 0; liftPos <= 500; liftPos = liftPos + 4) {
                    liftTargetPosition = liftPos;
                    arm.setTargetPosition(liftTargetPosition);
                    arm.setPower(0.7);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(12);
                }
                clawBottom.setPosition(openClaw);
                sleep(700);
                clawTop.setPosition(openClaw);

            }
            if (gamepad1.left_bumper) {
                if (armDown) {
                    liftTargetPosition = 300;
                    armDown = false;
                    arm.setTargetPosition(liftTargetPosition);
                    arm.setPower(0.07);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    clawTop.setPosition(openClaw);
                    clawBottom.setPosition(openClaw);
                } else {
                    liftTargetPosition = 5;
                    armDown = true;
                    arm.setTargetPosition(liftTargetPosition);
                    arm.setPower(0.07);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    clawTop.setPosition(closeClaw);
                    clawBottom.setPosition(closeClaw);
                }
            }

            if (!arm.isBusy()) {
                arm.setTargetPosition(liftTargetPosition);
                arm.setPower(0.7);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
             */


            if (gamepad1.y) {
                clawTop.setPosition(closeClaw);
            }
            if (gamepad1.x) {
                clawTop.setPosition(openClaw);
            }

            if (gamepad1.b) {
                clawBottom.setPosition(closeClaw);
            }
            if (gamepad1.a) {
                clawBottom.setPosition(openClaw);
            }

            arm.setTargetPosition(liftTargetPosition);
            arm.setPower(0.7);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            telemetry.addData("y (left stick y): ", y);
            telemetry.addData("x (left stick x): ", x);
            telemetry.addData("rx (right stick x): ", rx);

            telemetry.addData("Front Left Power: ", frontLeftPower);
            telemetry.addData("Front Right Power: ", frontRightPower);
            telemetry.addData("Back Left Power: ", backLeftPower);
            telemetry.addData("Back Right Power: ", backRightPower);

            telemetry.addData("Claw Top Position: ", clawTop.getPosition());
            telemetry.addData("Claw Bottom Position: ", clawBottom.getPosition());

            telemetry.addData("Arm position: ", arm.getCurrentPosition());
            telemetry.addData("Arm Target Position Requested: ", liftTargetPosition);
            telemetry.addData("Arm Actual Target Position: ", arm.getTargetPosition());

            telemetry.update();
        }
    }
}