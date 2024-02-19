package org.firstinspires.ftc.teamcode.mechanisms.arm;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm_Instance_With_PIDF_Kavi {
    private PIDController Arm_PID_Controller;

    public static double P_Coefficient = 0.022;
    public static double I_Coefficient= 0.22;

    public static double D_Coefficient= 0.0015;

    public static double F_Coefficient = 0.22;

    public static int Arm_Target_Angle = 0;

    private int Arm_Positional_Error;

    public static int Arm_Motor_Tolerance = 5; //ticks

    public static int Arm_Current_Position;

    private final double Arm_Ticks_Per_Revolution = 537.7; //goBilda Yellow Jacket 5203

    private final double Arm_Motor_Ticks_Per_Degree = Arm_Ticks_Per_Revolution / 180;

    private DcMotorEx Arm_Motor;// = hardwareMap.get(DcMotorEx.class, "Arm_Motor");


    public void Initialize_Arm_Instance(HardwareMap hardwareMap) {
        //Init Code
        Arm_PID_Controller = new PIDController(P_Coefficient, I_Coefficient, D_Coefficient);
        Arm_Motor = hardwareMap.get(DcMotorEx.class, "Arm_Motor");
        Arm_Motor.setDirection(DcMotorEx.Direction.REVERSE);
        Arm_Motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Arm_Motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void Update_Arm_Position_With_PIDF() {
        //Active code
        Arm_PID_Controller.setPID(P_Coefficient, I_Coefficient, D_Coefficient);

        Arm_Current_Position = Arm_Motor.getCurrentPosition();

        Arm_Positional_Error = Arm_Target_Angle - Arm_Current_Position;

        if (Math.abs(Arm_Positional_Error) > Arm_Motor_Tolerance) {
            double CalculatedPID = Arm_PID_Controller.calculate(Arm_Current_Position, Arm_Target_Angle);

            double CalculatedFeedForward = Math.cos(Math.toRadians(Arm_Target_Angle / Arm_Motor_Ticks_Per_Degree))*F_Coefficient;

            double Calculated_Power_For_Arm_Motor = CalculatedPID + CalculatedFeedForward;

            Arm_Motor.setPower(Calculated_Power_For_Arm_Motor);
        }
    }
}