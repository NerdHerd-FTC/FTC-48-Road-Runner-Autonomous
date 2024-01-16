package org.firstinspires.ftc.teamcode.Autonomous_Drive.Blue_Alliance_Front;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Vision.tensorFlow.TensorFlowInstance_Individual_Scanning;
import org.firstinspires.ftc.teamcode.drive.MecanumDrivebaseInstance;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.mechanisms.arm.ArmInstance;
import org.firstinspires.ftc.teamcode.mechanisms.claw.ClawInstance;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class _2023122103_Kavi_Gupta_Blue_Alliance_Front_V3 extends LinearOpMode {
    @Override
    public void runOpMode() {

        int ArmMovementValue = 555;
        MecanumDrivebaseInstance MecanumDrivebase = new MecanumDrivebaseInstance(hardwareMap);
        TensorFlowInstance_Individual_Scanning TensorFlow = new TensorFlowInstance_Individual_Scanning();
        ClawInstance Claw = new ClawInstance();
        ArmInstance Arm = new ArmInstance();
        final TensorFlowInstance_Individual_Scanning.CameraStreamProcessor processor = new TensorFlowInstance_Individual_Scanning.CameraStreamProcessor();

        Pose2d StartingCoordinates = new Pose2d(12, 60, Math.toRadians(90));
        MecanumDrivebase.setPoseEstimate(StartingCoordinates);
        TensorFlow.IntitializeTensorFlow(hardwareMap, processor);
        Claw.initializeClaw(hardwareMap);
        Arm.initializeArm(hardwareMap);

        double leftDetectionConfidence = 0;
        double centerDetectionConfidence = 0;
        double rightDetectionConfidence = 0;

        List<Double> detectionConfidences = new ArrayList<Double>();
        String PropLocation = null;
        TensorFlow.SetWebcamStreamStatus("start");

        Pose2d BlueFrontCenterStartingCoordinates = new Pose2d(12, 60, Math.toRadians(90));

        TrajectorySequence BlueFrontCenter = MecanumDrivebase.trajectorySequenceBuilder(BlueFrontCenterStartingCoordinates)
                .lineTo(new Vector2d(12, 32))
                .addDisplacementMarker(() -> {
                    Arm.moveArmTo(5);
                    while (Arm.Arm_Motor.isBusy()) {}
                    Claw.Actuate_Claw_Bottom_Finger("open");
                    Arm.moveArmTo(ArmMovementValue);
                })
                .lineToLinearHeading(new Pose2d(42,35, Math.toRadians(0)))
                .addDisplacementMarker(() -> {
                    while (Arm.Arm_Motor.isBusy()) {}
                    Claw.Actuate_Claw_Top_Finger("open");
                })
                .build();

        waitForStart();

        if (opModeIsActive()) {
            Claw.Actuate_Claw_Top_Finger("close");
            Claw.Actuate_Claw_Bottom_Finger("close");
            if (isStopRequested()) return;
            //Arm.moveArmTo(100);
            //MecanumDrivebase.followTrajectorySequence(BlueFrontCenter);
            //TensorFlow.ActuateCameraTo(5, 0 , 0);
        }

        PoseStorage.currentPose = MecanumDrivebase.getPoseEstimate();

    }
}
