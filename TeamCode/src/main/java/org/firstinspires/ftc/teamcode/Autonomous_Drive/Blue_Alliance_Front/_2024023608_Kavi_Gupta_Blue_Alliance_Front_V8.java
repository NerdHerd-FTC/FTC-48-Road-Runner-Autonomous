package org.firstinspires.ftc.teamcode.Autonomous_Drive.Blue_Alliance_Front;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.MecanumDrivebaseInstance;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.mechanisms.arm.ArmInstancePrevious;
import org.firstinspires.ftc.teamcode.mechanisms.claw.ClawInstance;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class _2024023608_Kavi_Gupta_Blue_Alliance_Front_V8 extends LinearOpMode {
    @Override
    public void runOpMode() {

        final short ArmBackboardPosition = 580;
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
                .setTurnConstraint(5, 5)
                .back(10)
                .turn(Math.toRadians(45))
                .setTurnConstraint(5, 5)
                .turn(Math.toRadians(-90))
                .setTurnConstraint(5,5)
                .turn(Math.toRadians(45))
                .back(4)
                //scn end

                .back(16)
                .addTemporalMarker(() -> {
                    Arm.moveArmTo(5);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    Claw.Actuate_Claw_Bottom_Finger("open");
                })
                .waitSeconds(0.5)
                .addTemporalMarker( () -> {
                    Arm.moveArmTo(ArmBackboardPosition);
                })
                //floor pixel
                .waitSeconds(0.5)
                .back(3)
                .waitSeconds(0.5)
                .forward(8)
                .turn(Math.toRadians(-90))
                .resetConstraints()
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    Arm.moveArmTo(ArmBackboardPosition);
                })
                .forward(37)
                .turn(Math.toRadians(5))
                .waitSeconds(0.5)
                //backboard pixel
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    Claw.Actuate_Claw_Top_Finger("open");
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Arm.moveArmTo(50);
                })
                .waitSeconds(1)
                .back(5)
                .strafeLeft(25)
                .forward(12)
                .build();

        TrajectorySequence BlueFrontLeft = MecanumDrivebase.trajectorySequenceBuilder(BlueFrontStartingCoordinates)
                .setTurnConstraint(5, 5)
                .back(10)
                .turn(Math.toRadians(45))
                .setTurnConstraint(5, 5)
                .turn(Math.toRadians(-90))
                .setTurnConstraint(5,5)
                .turn(Math.toRadians(45))
                .back(4)
                //scn end
                .back(18)
                .turn(Math.toRadians(90))
                .resetConstraints()
                .addTemporalMarker(() -> {
                    Arm.moveArmTo(5);
                })
                .back(4)
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    Claw.Actuate_Claw_Bottom_Finger("open");
                })
                .waitSeconds(0.5)
                .addTemporalMarker( () -> {
                    Arm.moveArmTo(ArmBackboardPosition);
                })
                .forward(7)
                .strafeRight(10)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    Arm.moveArmTo(ArmBackboardPosition);
                })
                .back(35)
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    Claw.Actuate_Claw_Top_Finger("open");
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Arm.moveArmTo(50);
                })
                .waitSeconds(1)
                .forward(5)
                .strafeRight(15)
                .back(15)
                .build();

        TrajectorySequence BlueFrontRight = MecanumDrivebase.trajectorySequenceBuilder(BlueFrontStartingCoordinates)
                .setTurnConstraint(5, 5)
                .back(10)
                .turn(Math.toRadians(45))
                .setTurnConstraint(5, 5)
                .turn(Math.toRadians(-90))
                .setTurnConstraint(5,5)
                .turn(Math.toRadians(45))
                .back(4)
                //scn end
                .back(17)
                .turn(Math.toRadians(-90))
                .resetConstraints()
                .addTemporalMarker(() -> {
                    Arm.moveArmTo(5);
                })
                //drop on floor
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    Claw.Actuate_Claw_Bottom_Finger("open");
                })
                .waitSeconds(0.5)
                .addTemporalMarker( () -> {
                    Arm.moveArmTo(ArmBackboardPosition);
                })
                .waitSeconds(0.5)
                .back(4)
                .waitSeconds(0.5)
                .forward(42)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    Arm.moveArmTo(ArmBackboardPosition);
                })
                //Drop Backboard
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    Claw.Actuate_Claw_Top_Finger("open");
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Arm.moveArmTo(50);
                })
                .waitSeconds(1)
                .back(4)
                .strafeLeft(30)
                .forward(15)
                .build();

        TrajectorySequence Park = MecanumDrivebase.trajectorySequenceBuilder(BlueFrontStartingCoordinates)
                .lineToLinearHeading(new Pose2d(50, 60, Math.toRadians(0)))
                .lineTo(new Vector2d(60,60))
                .build();

        Claw.Actuate_Claw_Top_Finger("close");
        Claw.Actuate_Claw_Bottom_Finger("close");
        waitForStart();

        PropLocation = "right";

        if (opModeIsActive()) {
            Arm.moveArmTo(ArmRestPosition);
            Claw.Actuate_Claw_Top_Finger("close");
            Claw.Actuate_Claw_Bottom_Finger("close");
            sleep(1000);
            Arm.moveArmTo(300);

            if ("center" == PropLocation) {
                MecanumDrivebase.followTrajectorySequence(BlueFrontCenter);
            } else if ("left" == PropLocation) {
                MecanumDrivebase.followTrajectorySequence(BlueFrontLeft);
            } else if ("right" == PropLocation) {
                MecanumDrivebase.followTrajectorySequence(BlueFrontRight);
            } else {
                //MecanumDrivebase.followTrajectorySequence(Park);
            }
            while (Arm.Arm_Motor.isBusy()) {};
        }

        PoseStorage.currentPose = MecanumDrivebase.getPoseEstimate();

    }
}
