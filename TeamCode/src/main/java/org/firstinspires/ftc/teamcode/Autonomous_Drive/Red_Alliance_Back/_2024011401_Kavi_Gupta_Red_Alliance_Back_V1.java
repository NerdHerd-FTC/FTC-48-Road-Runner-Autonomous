package org.firstinspires.ftc.teamcode.Autonomous_Drive.Red_Alliance_Back;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.MecanumDrivebaseInstance;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.mechanisms.arm.ArmInstance;
import org.firstinspires.ftc.teamcode.mechanisms.claw.ClawInstance;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class _2024011401_Kavi_Gupta_Red_Alliance_Back_V1 extends LinearOpMode {
    @Override
    public void runOpMode() {

        final short ArmBackboardPosition = 555;
        final short ArmRestPosition = 150;
        final short ArmPlacePixelOnFloorPosition = 5;

        final MecanumDrivebaseInstance MecanumDrivebase = new MecanumDrivebaseInstance(hardwareMap);
        //final TensorFlowInstance_Individual_Scanning TensorFlow = new TensorFlowInstance_Individual_Scanning();
        final ClawInstance Claw = new ClawInstance();
        final ArmInstance Arm = new ArmInstance();
        //final TensorFlowInstance_Individual_Scanning.CameraStreamProcessor processor = new TensorFlowInstance_Individual_Scanning.CameraStreamProcessor();

        Pose2d StartingPositionRedBack = new Pose2d(-36, -60, Math.toRadians(-90));
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

        TrajectorySequence redBackRight = MecanumDrivebaseInstance.trajectorySequenceBuilder(StartingPositionRedBack)
                .setTurnConstraint(5, 5)
                .back(10)
                .turn(Math.toRadians(45))
                .setTurnConstraint(5, 5)
                .turn(Math.toRadians(-90))
                .setTurnConstraint(5,5)
                .turn(Math.toRadians(45))
                .back(4)
                .back(18)
                .turn(Math.toRadians(-90))
                .resetConstraints()
                .back(4)
                .waitSeconds(0.5)
                .waitSeconds(0.5)
                .waitSeconds(1)
                .forward(7)
                .strafeRight(17)
                .turn(Math.toRadians(180))
                .forward(70)
                .strafeRight(17)
                .forward(17)
                .back(5)
                .strafeLeft(15)
                .forward(15)
                .build();

        waitForStart();

        PropLocation = "right";

        if (opModeIsActive()) {
            Arm.moveArmTo(ArmRestPosition);
            Claw.Actuate_Claw_Top_Finger("close");
            Claw.Actuate_Claw_Bottom_Finger("close");
            if ("center" == PropLocation) {
                //MecanumDrivebase.followTrajectorySequence(BlueFrontCenter);
            } else if ("left" == PropLocation) {
                //MecanumDrivebase.followTrajectorySequence(BlueFrontLeft);
                Claw.Actuate_Claw_Bottom_Finger("close");
                Claw.Actuate_Claw_Top_Finger("close");
                sleep(500);
                Arm.moveArmTo(50);
                while (Arm.Arm_Motor.isBusy()) {if(isStopRequested()){return;}}
                stop();
            } else if ("right" == PropLocation) {
                Claw.Actuate_Claw_Bottom_Finger("close");
                Claw.Actuate_Claw_Top_Finger("close");
                sleep(500);
                Arm.moveArmTo(50);
                MecanumDrivebase.followTrajectorySequence(redBackRight);
            } else {
                //MecanumDrivebase.followTrajectorySequence(Park);
            }
        }

        PoseStorage.currentPose = MecanumDrivebase.getPoseEstimate();

    }
}
