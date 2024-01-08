package org.firstinspires.ftc.teamcode.mechanisms.arm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ArmInstanceWithPID {

    private double Arm_Motor_Power = 0.2;
    private HardwareMap hardwareMap;
    public int currentArmPos;

<<<<<<< Updated upstream
=======

    public DcMotor Arm_Motor;

    ElapsedTime MoveArmToTimer = new ElapsedTime();

>>>>>>> Stashed changes
    double armKp = 0.13;//0.029399999; 0.13
    double armKi = 0.00005999999;
    double armKd = 0.00001;

    double integralSum = 0;
    double lastError = 0;
<<<<<<< Updated upstream

    ElapsedTime timer = new ElapsedTime();


    public DcMotor Arm_Motor;

=======
>>>>>>> Stashed changes
    public void initializeArm(HardwareMap hardwareMap) {
        Arm_Motor = hardwareMap.get(DcMotor.class, "Arm_Motor");
        Arm_Motor.setDirection(DcMotor.Direction.REVERSE);
        Arm_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm_Motor.setTargetPosition(15);
        Arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void moveArmTo(int Desired_Arm_Position) {
        while (Arm_Motor.getCurrentPosition() != Desired_Arm_Position) {
            double Arm_Position = Arm_Motor.getCurrentPosition();

            double error = Desired_Arm_Position - Arm_Position;

<<<<<<< Updated upstream
            double derivative = (error - lastError) /timer.seconds();

            integralSum = integralSum + (error * timer.seconds());
=======
            double derivative = (error - lastError) /MoveArmToTimer.seconds();

            integralSum = integralSum + (error * MoveArmToTimer.seconds());
>>>>>>> Stashed changes

            double out = (armKp * error) + (armKi * integralSum) + (armKd * derivative);

            Arm_Motor.setPower(out);

            lastError = error;
<<<<<<< Updated upstream
=======

            MoveArmToTimer.reset();
>>>>>>> Stashed changes
        }

    }
}