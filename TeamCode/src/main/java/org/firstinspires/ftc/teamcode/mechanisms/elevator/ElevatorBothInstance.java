package org.firstinspires.ftc.teamcode.mechanisms.elevator;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ElevatorBothInstance {
    private HardwareMap hardwareMap;

    public DcMotor LeftElevatorMotor;
    public DcMotor RightElevatorMotor;
    public CRServo LeftElevatorServo;
    public CRServo RightElevatorServo;

    public void initializeElevators(HardwareMap hardwareMap) {
        DcMotor LeftElevatorMotor = hardwareMap.get(DcMotor.class, "Left_Elevator_Motor");
        CRServo LeftElevatorServo = hardwareMap.get(CRServo.class, "Left_Elevator_Servo");
        DcMotor RightElevatorMotor = hardwareMap.get(DcMotor.class, "Right_Elevator_Motor");
        CRServo RightElevatorServo = hardwareMap.get(CRServo.class, "Right_Elevator_Servo");
        LeftElevatorServo.setDirection(DcMotorSimple.Direction.REVERSE);
        RightElevatorServo.setDirection(DcMotorSimple.Direction.REVERSE);

        LeftElevatorMotor.setPower(0);
        LeftElevatorServo.setPower(0);
        RightElevatorMotor.setPower(0);
        RightElevatorServo.setPower(0);

    }

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


}
