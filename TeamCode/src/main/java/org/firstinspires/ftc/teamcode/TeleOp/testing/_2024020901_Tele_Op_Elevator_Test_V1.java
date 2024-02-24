package org.firstinspires.ftc.teamcode.TeleOp.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Disabled
public class _2024020901_Tele_Op_Elevator_Test_V1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor LeftElevatorMotor = hardwareMap.get(DcMotor.class, "Left_Elevator_Motor");
        CRServo LeftElevatorServo = hardwareMap.get(CRServo.class, "Left_Elevator_Servo");
        DcMotor RightElevatorMotor = hardwareMap.get(DcMotor.class, "Right_Elevator_Motor");
        CRServo RightElevatorServo = hardwareMap.get(CRServo.class, "Right_Elevator_Servo");

        LeftElevatorServo.setDirection(DcMotorSimple.Direction.REVERSE);
        RightElevatorServo.setDirection(DcMotorSimple.Direction.REVERSE);
        double directionMultiplier = 1;

        int Elevator_Run_Duration_Milliseconds = 5000;
        boolean Operate_Elevator = false;
        String Direction = "Up";


        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                Operate_Elevator = true;
                Direction = "Up";
            }
            if (gamepad1.dpad_down) {
                Operate_Elevator = true;
                Direction = "Down";
            }

            if (Operate_Elevator) {
                if (Direction == "Up") {
                    LeftElevatorMotor.setPower(1);
                    RightElevatorMotor.setPower(1);
                    LeftElevatorServo.setPower(1);
                    RightElevatorServo.setPower(1);
                } else if (Direction == "Down") {
                    LeftElevatorMotor.setPower(-1);
                    RightElevatorMotor.setPower(-1);
                    LeftElevatorServo.setPower(-1);
                    RightElevatorServo.setPower(-1);
                }
            }
        }
    }
}

