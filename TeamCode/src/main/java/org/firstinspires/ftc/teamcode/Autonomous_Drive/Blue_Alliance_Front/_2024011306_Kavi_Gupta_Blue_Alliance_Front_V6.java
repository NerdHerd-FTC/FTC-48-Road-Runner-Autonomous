package org.firstinspires.ftc.teamcode.Autonomous_Drive.Blue_Alliance_Front;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.MecanumDrivebaseInstance;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.mechanisms.arm.ArmInstancePrevious;
import org.firstinspires.ftc.teamcode.mechanisms.claw.ClawInstance;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.List;

@Autonomous
@Disabled
public class _2024011306_Kavi_Gupta_Blue_Alliance_Front_V6 extends LinearOpMode {
    @Override
    public void runOpMode() {

        final short ArmBackboardPosition = 525;
        final short ArmRestPosition = 150;
        final short ArmPlacePixelOnFloorPosition = 5;

        final MecanumDrivebaseInstance MecanumDrivebase = new MecanumDrivebaseInstance(hardwareMap);
        //final TensorFlowInstance_Individual_Scanning TensorFlow = new TensorFlowInstance_Individual_Scanning();
        final ClawInstance Claw = new ClawInstance();
        final ArmInstancePrevious Arm = new ArmInstancePrevious();
        //final TensorFlowInstance_Individual_Scanning.CameraStreamProcessor processor = new TensorFlowInstance_Individual_Scanning.CameraStreamProcessor();

        final Pose2d BlueFrontStartingCoordinates = new Pose2d(12, 60, Math.toRadians(90));
        MecanumDrivebase.setPoseEstimate(BlueFrontStartingCoordinates);
        //TensorFlow.InititializeTensorFlow(hardwareMap, processor);
        Claw.initializeClaw(hardwareMap);
        Arm.initializeArm(hardwareMap);

        double leftDetectionConfidence = 0;
        double centerDetectionConfidence = 0;
        double rightDetectionConfidence = 0;

        List<Double> detectionConfidences = new ArrayList<Double>();
        String PropLocation = null;
        //TensorFlow.SetWebcamStreamStatus("start");

        TrajectorySequence BlueFrontCenter = MecanumDrivebase.trajectorySequenceBuilder(BlueFrontStartingCoordinates)
                .lineTo(new Vector2d(12, 32))
                .addDisplacementMarker(() -> {
                    Arm.moveArmTo(ArmPlacePixelOnFloorPosition);
                    while (Arm.Arm_Motor.isBusy()) {if(isStopRequested()){return;}}
                    Claw.Actuate_Claw_Bottom_Finger("open");
                    Arm.moveArmTo(ArmBackboardPosition);
                })
                .forward(3)
                .lineToLinearHeading(new Pose2d(50,35, Math.toRadians(0)))
                .addDisplacementMarker(() ->{
                    while (Arm.Arm_Motor.isBusy()) {if(isStopRequested()){return;}}
                    Claw.Actuate_Claw_Top_Finger("open");
                    Arm.moveArmTo(ArmRestPosition);
                })
                .lineToLinearHeading(new Pose2d(50, 60, Math.toRadians(0)))
                .lineTo(new Vector2d(60,60))
                .build();

        TrajectorySequence BlueFrontLeft = MecanumDrivebase.trajectorySequenceBuilder(BlueFrontStartingCoordinates)
                .lineToLinearHeading(new Pose2d(15, 30, Math.toRadians(180)))
                .addTemporalMarker(() -> {
                    Arm.moveArmTo(ArmPlacePixelOnFloorPosition);
                    while (Arm.Arm_Motor.isBusy()) {if(isStopRequested()){return;}}
                    Claw.Actuate_Claw_Bottom_Finger("open");
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    Arm.moveArmTo(ArmBackboardPosition);
                })
                .waitSeconds(1)
                .forward(3)
                .waitSeconds(1)
                .strafeRight(15)
                .lineToLinearHeading(new Pose2d(50,42, Math.toRadians(0)))
                .waitSeconds(2)
                .addTemporalMarker(() ->{
                    while (Arm.Arm_Motor.isBusy()) {if(isStopRequested()){return;}}
                    Claw.Actuate_Claw_Top_Finger("open");
                })
                .waitSeconds(2)
                .forward(3)
                .lineToLinearHeading(new Pose2d(47, 60, Math.toRadians(0)))
                .lineTo(new Vector2d(60,60))
                .build();

        TrajectorySequence BlueFrontRight = MecanumDrivebase.trajectorySequenceBuilder(BlueFrontStartingCoordinates)
                .lineToLinearHeading(new Pose2d(9, 30, Math.toRadians(0)))
                .addDisplacementMarker(() -> {
                    Arm.moveArmTo(ArmPlacePixelOnFloorPosition);
                    while (Arm.Arm_Motor.isBusy()) {if(isStopRequested()){return;}}
                    Claw.Actuate_Claw_Bottom_Finger("open");
                    Arm.moveArmTo(ArmBackboardPosition);
                })
                .lineToLinearHeading(new Pose2d(50,42, Math.toRadians(0)))
                .addDisplacementMarker(() ->{
                    while (Arm.Arm_Motor.isBusy()) {if(isStopRequested()){return;}}
                    Claw.Actuate_Claw_Top_Finger("open");
                    Arm.moveArmTo(ArmRestPosition);
                })
                .lineToLinearHeading(new Pose2d(50, 60, Math.toRadians(0)))
                .lineTo(new Vector2d(60,60))
                .build();

        TrajectorySequence Park = MecanumDrivebase.trajectorySequenceBuilder(BlueFrontStartingCoordinates)
                .lineToLinearHeading(new Pose2d(50, 60, Math.toRadians(0)))
                .lineTo(new Vector2d(60,60))
                .build();

        Claw.Actuate_Claw_Top_Finger("close");
        Claw.Actuate_Claw_Bottom_Finger("close");
        waitForStart();

        PropLocation = "left";

        if (opModeIsActive()) {
            Claw.Actuate_Claw_Top_Finger("close");
            Claw.Actuate_Claw_Bottom_Finger("close");
            sleep(1000);
            Arm.moveArmTo(ArmRestPosition);

            if ("center" == PropLocation) {
                MecanumDrivebase.followTrajectorySequence(BlueFrontCenter);
            } else if ("left" == PropLocation) {
                MecanumDrivebase.followTrajectorySequence(BlueFrontLeft);
                Claw.Actuate_Claw_Bottom_Finger("close");
                Claw.Actuate_Claw_Top_Finger("close");
                sleep(500);
                Arm.moveArmTo(50);
                while (Arm.Arm_Motor.isBusy()) {if(isStopRequested()){return;}}
                stop();
            } else if ("right" == PropLocation) {
                MecanumDrivebase.followTrajectorySequence(BlueFrontRight);
            } else {
                MecanumDrivebase.followTrajectorySequence(Park);
            }
        }

        PoseStorage.currentPose = MecanumDrivebase.getPoseEstimate();

    }
}
