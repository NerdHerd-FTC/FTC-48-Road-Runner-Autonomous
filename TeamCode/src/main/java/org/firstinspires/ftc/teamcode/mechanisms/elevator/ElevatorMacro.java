package org.firstinspires.ftc.teamcode.mechanisms.elevator;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ElevatorMacro {
    private HardwareMap hardwareMap;

    public DcMotor LeftElevatorMotor;
    public DcMotor RightElevatorMotor;
    public CRServo LeftElevatorServo;
    public CRServo RightElevatorServo;

    private double Elevator_Servos_Maximum_Movement_Cycles = 500;
    private int Elevator_Motors_Maximum_Encoder_Ticks = 1000;

    public double Elevator_Servo_Elapsed_Movement_Cycles = 0;

    private double Elevator_Power = 1;

    public void initializeElevators(HardwareMap hardwareMap) {
        DcMotor LeftElevatorMotor = hardwareMap.get(DcMotor.class, "Left_Elevator_Motor");
        CRServo LeftElevatorServo = hardwareMap.get(CRServo.class, "Left_Elevator_Servo");
        DcMotor RightElevatorMotor = hardwareMap.get(DcMotor.class, "Right_Elevator_Motor");
        CRServo RightElevatorServo = hardwareMap.get(CRServo.class, "Right_Elevator_Servo");
        LeftElevatorServo.setDirection(DcMotorSimple.Direction.REVERSE);
        RightElevatorServo.setDirection(DcMotorSimple.Direction.REVERSE);

        LeftElevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftElevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightElevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightElevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftElevatorMotor.setPower(0);
        LeftElevatorServo.setPower(0);
        RightElevatorMotor.setPower(0);
        RightElevatorServo.setPower(0);

        Elevator_Servo_Elapsed_Movement_Cycles = 0;

    }
    /*

    public void ElevatorMotorUp(double power, long timems) throws InterruptedException {
        LeftElevatorMotor.setPower(power);
        RightElevatorMotor.setPower(power);
        sleep(timems);
        LeftElevatorMotor.setPower(0);
        RightElevatorMotor.setPower(0);
    }

    public void ElevatorMotorDown(double power, long timems) throws InterruptedException {
        LeftElevatorMotor.setPower(-power);
        RightElevatorMotor.setPower(-power);
        sleep(timems);
        LeftElevatorMotor.setPower(0);
        RightElevatorMotor.setPower(0);
    }

    public void ElevatorServoUp(double power, long timems) throws InterruptedException {
        LeftElevatorServo.setPower(power);
        RightElevatorServo.setPower(power);
        sleep(timems);
        LeftElevatorServo.setPower(0);
        RightElevatorServo.setPower(0);
    }
    */
    public void SetElevatorServosPower(double Power) {
        LeftElevatorServo.setPower(Power);
        RightElevatorServo.setPower(Power);
    }

    public void UpdateElevatorPositionToPreset(String Direction) {
        if (Direction == "Up") {
            if (Elevator_Servo_Elapsed_Movement_Cycles < Elevator_Servos_Maximum_Movement_Cycles) {
                Elevator_Servo_Elapsed_Movement_Cycles += 1;
                SetElevatorServosPower(Elevator_Power);
            } else {
                SetElevatorServosPower(0);
            }
            if (LeftElevatorMotor.getCurrentPosition() < Elevator_Motors_Maximum_Encoder_Ticks) {
                LeftElevatorMotor.setPower(1);
            } else {
                LeftElevatorMotor.setPower(0);
            }

            if (RightElevatorMotor.getCurrentPosition() < Elevator_Motors_Maximum_Encoder_Ticks) {
                RightElevatorMotor.setPower(1);
            } else {
                RightElevatorMotor.setPower(0);
            }
        } else if (Direction == "Down") {
            if (Elevator_Servo_Elapsed_Movement_Cycles < Elevator_Servos_Maximum_Movement_Cycles) {
                Elevator_Servo_Elapsed_Movement_Cycles -= 1;
                SetElevatorServosPower(Elevator_Power);
            } else {
                SetElevatorServosPower(0);
            }
            if (LeftElevatorMotor.getCurrentPosition() > 0) {
                LeftElevatorMotor.setPower(-1);
            } else {
                LeftElevatorMotor.setPower(0);
            }

            if (RightElevatorMotor.getCurrentPosition() > 0) {
                RightElevatorMotor.setPower(-1);
            } else {
                RightElevatorMotor.setPower(0);
            }
        }
    }


}
