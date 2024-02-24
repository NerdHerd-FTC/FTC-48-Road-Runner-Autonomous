package org.firstinspires.ftc.teamcode.Autonomous_Drive.Blue_Alliance_Back;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
public class _2024022304_Kavi_Gupta_Blue_Alliance_Back_V4 extends LinearOpMode {
    @Override
    public void runOpMode() {

        final short ArmBackboardPosition = 570;
        final short ArmRestPosition = 150;
        final short ArmPlacePixelOnFloorPosition = 5;

        final MecanumDrivebaseInstance MecanumDrivebase = new MecanumDrivebaseInstance(hardwareMap);
        //final TensorFlowInstance_Individual_Scanning TensorFlow = new TensorFlowInstance_Individual_Scanning();
        final ClawInstance Claw = new ClawInstance();
        final ArmInstancePrevious Arm = new ArmInstancePrevious();
        //final TensorFlowInstance_Individual_Scanning.CameraStreamProcessor processor = new TensorFlowInstance_Individual_Scanning.CameraStreamProcessor();

        Pose2d StartingPositionRedBack = new Pose2d(-36, 60, Math.toRadians(90));
        MecanumDrivebase.setPoseEstimate(StartingPositionRedBack);
        //TensorFlow.InititializeTensorFlow(hardwareMap, processor);
        Claw.initializeClaw(hardwareMap);
        Arm.initializeArm(hardwareMap);

        double leftDetectionConfidence = 0;
        double centerDetectionConfidence = 0;
        double rightDetectionConfidence = 0;

        List<Double> detectionConfidences = new ArrayList<Double>();
        String PropLocation = null;
        //TensorFlow.SetWebcamStreamStatus("start");

        TrajectorySequence blueBackRight = MecanumDrivebaseInstance.trajectorySequenceBuilder(StartingPositionRedBack)
                .setTurnConstraint(5, 5)
                .back(10)
                .turn(Math.toRadians(45))
                .setTurnConstraint(5, 5)
                .turn(Math.toRadians(-90))
                .setTurnConstraint(5,5)
                .turn(Math.toRadians(45))
                .back(4)
                //scn end
                .back(13)
                .turn(Math.toRadians(-90))
                .addTemporalMarker(() -> {
                    Arm.moveArmTo(5);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    Claw.Actuate_Claw_Bottom_Finger("open");
                })
                .waitSeconds(0.5)
                .addTemporalMarker( () -> {
                    Arm.moveArmTo(ArmRestPosition);
                })
                .waitSeconds(0.5)
                .back(4)
                //floor
                .waitSeconds(1)
                .forward(5)
                .strafeRight(24)
                .forward(70)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    Arm.moveArmTo(ArmBackboardPosition);
                })
                .strafeLeft(27)
                .forward(12)
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    Claw.Actuate_Claw_Top_Finger("open");
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Arm.moveArmTo(50);
                })
                .waitSeconds(1)
                .back(5)
                .strafeRight(15)
                .forward(15)
                .build();

        TrajectorySequence blueBackLeft = MecanumDrivebaseInstance.trajectorySequenceBuilder(StartingPositionRedBack)
                .setTurnConstraint(5, 5)
                .back(10)
                .turn(Math.toRadians(45))
                .setTurnConstraint(5, 5)
                .turn(Math.toRadians(-90))
                .setTurnConstraint(5,5)
                .turn(Math.toRadians(45))
                .back(4)
                //scn end
                .back(13)
                .addTemporalMarker(() -> {
                    Arm.moveArmTo(5);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    Claw.Actuate_Claw_Bottom_Finger("open");
                })
                .waitSeconds(0.5)
                .addTemporalMarker( () -> {
                    Arm.moveArmTo(ArmRestPosition);
                })
                .waitSeconds(0.5)
                //Beginning Sequence
                .turn(Math.toRadians(90))
                .back(4)
                .waitSeconds(0.5)
                .forward(5)
                .strafeLeft(24)
                .turn(Math.toRadians(170))
                .waitSeconds(0.5)
                .forward(70)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    Arm.moveArmTo(ArmBackboardPosition);
                })
                .strafeLeft(27)
                .turn(Math.toRadians(-10))
                .forward(12)
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    Claw.Actuate_Claw_Top_Finger("open");
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Arm.moveArmTo(50);
                })
                .waitSeconds(1)
                .back(5)
                .strafeRight(30)
                .forward(15)
                .resetConstraints()
                .build();

        TrajectorySequence blueBackCenter = MecanumDrivebaseInstance.trajectorySequenceBuilder(StartingPositionRedBack)
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
                .addTemporalMarker(() -> {
                    Arm.moveArmTo(5);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    Claw.Actuate_Claw_Bottom_Finger("open");
                })
                .waitSeconds(0.5)
                .addTemporalMarker( () -> {
                    Arm.moveArmTo(ArmRestPosition);
                })
                .waitSeconds(0.5)
                //Beginning Sequence
                .forward(5)
                .strafeLeft(16)
                .turn(Math.toRadians(-90))
                .waitSeconds(0.5)
                .strafeRight(23)
                .turn(Math.toRadians(5))
                //truss forward
                .forward(95)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    Arm.moveArmTo(ArmBackboardPosition);
                })

                .strafeLeft(27)
                .forward(5)
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    Claw.Actuate_Claw_Top_Finger("open");
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Arm.moveArmTo(50);
                })
                .waitSeconds(1)
                .back(10)
                .strafeRight(24)
                .forward(18)
                .resetConstraints()
                .build();


        waitForStart();

        PropLocation = "center";

        if (opModeIsActive()) {
            Arm.moveArmTo(ArmRestPosition);
            Claw.Actuate_Claw_Top_Finger("close");
            Claw.Actuate_Claw_Bottom_Finger("close");
            sleep(1000);
            Arm.moveArmTo(300);
            if ("center" == PropLocation) {
                MecanumDrivebase.followTrajectorySequence(blueBackCenter);
            } else if ("left" == PropLocation) {
                MecanumDrivebase.followTrajectorySequence(blueBackLeft);

            } else if ("right" == PropLocation) {
                MecanumDrivebase.followTrajectorySequence(blueBackRight);
            } else {
                //MecanumDrivebase.followTrajectorySequence(Park);
            }
            PoseStorage.currentPose = MecanumDrivebase.getPoseEstimate();
            while (Arm.Arm_Motor.isBusy()) {};
        }


    }
}
