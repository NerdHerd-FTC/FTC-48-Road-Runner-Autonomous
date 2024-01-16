package org.firstinspires.ftc.teamcode.Subsystem_Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class ElevatorTest extends LinearOpMode {
    @Override
    public void runOpMode() {

        double directionMultiplier = 1;

        DcMotor ElevatorMotor = hardwareMap.get(DcMotor.class, "Left_Elevator_Motor");
        CRServo ElevatorServo = hardwareMap.get(CRServo.class, "Left_Elevator_Servo");
        ElevatorServo.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            if (isStopRequested()) return;
            ElevatorMotor.setPower(0);
            ElevatorServo.setPower(0);
            if (gamepad1.dpad_up) {
                directionMultiplier = 1;
                telemetry.addData("Direction Multiplier: ", directionMultiplier);
            }
            if (gamepad1.dpad_down) {
                directionMultiplier = -1;
                telemetry.addData("Direction Multiplier: ", directionMultiplier);
            }
            if (gamepad1.right_trigger > 0.1) {
                ElevatorMotor.setPower(gamepad1.right_trigger*directionMultiplier);
                telemetry.addLine("Motor Spinning");
            }
            if (gamepad1.left_trigger > 0.1) {
                ElevatorServo.setPower(gamepad1.left_trigger*directionMultiplier);
                telemetry.addLine("Servo Working");
            }
            if (gamepad1.back) {
                ElevatorMotor.setPower(0);
                telemetry.addLine("Reset");
            }
            telemetry.update();
        }
    }
}

