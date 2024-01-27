package org.firstinspires.ftc.teamcode.Autonomous_Drive.Red_Alliance_Front;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Vision.tensorFlow.TensorFlowInstance;
import org.firstinspires.ftc.teamcode.drive.MecanumDrivebaseInstance;
import org.firstinspires.ftc.teamcode.mechanisms.arm.ArmInstancePrevious;
import org.firstinspires.ftc.teamcode.mechanisms.claw.ClawInstance;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
@Disabled

public class _2023120404_Kavi_Gupta_Red_Alliance_Front_V4 extends LinearOpMode {
    @Override
    public void runOpMode() {

        MecanumDrivebaseInstance MecanumDrivebase = new MecanumDrivebaseInstance(hardwareMap);
        TensorFlowInstance TensorFlow = new TensorFlowInstance();
        ClawInstance Claw = new ClawInstance();
        ArmInstancePrevious Arm = new ArmInstancePrevious();
        final TensorFlowInstance.CameraStreamProcessor processor = new TensorFlowInstance.CameraStreamProcessor();

        Pose2d StartingCoordinates = new Pose2d(12, -60, Math.toRadians(270));
        MecanumDrivebase.setPoseEstimate(StartingCoordinates);
        TensorFlow.IntitializeTensorFlow(hardwareMap, processor);
        Claw.initializeClaw(hardwareMap);
        Arm.initializeArm(hardwareMap);

        String PropLocation;
        TensorFlow.SetWebcamStreamStatus("start");


        TrajectorySequence MoveToPropScanningLocation = MecanumDrivebase.trajectorySequenceBuilder(StartingCoordinates)
                .back(12)
                .addDisplacementMarker(() -> {
                    Claw.Actuate_Claw_Top_Finger("close");
                    Claw.Actuate_Claw_Bottom_Finger("close");
                    Arm.moveArmTo(200);
                })
                .waitSeconds(1)
                .build();

        TrajectorySequence PlacePixelOnCenterSpike = MecanumDrivebase.trajectorySequenceBuilder(MoveToPropScanningLocation.end())
                .strafeLeft(5)
                .back(20)
                .addDisplacementMarker(() -> {
                    //claw release
                    Arm.moveArmTo(10);
                    while (Arm.Arm_Motor.isBusy()) {}
                    Claw.Actuate_Claw_Bottom_Finger("open");
                })
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    Arm.moveArmTo(150);
                })
                .build();
        TrajectorySequence PlacePixelOnLeftSpike = MecanumDrivebase.trajectorySequenceBuilder(StartingCoordinates)
                .back(12)
                .turn(Math.toRadians(90))
                .strafeLeft(5)
                .forward(12)
                .addDisplacementMarker(() -> {
                    //claw release
                    Claw.Actuate_Claw_Bottom_Finger("open");
                })
                .waitSeconds(1)
                .build();
        TrajectorySequence PlaceYellowPixelOnCenterBackDrop = MecanumDrivebase.trajectorySequenceBuilder(PlacePixelOnCenterSpike.end())
                .forward(5)
                .strafeLeft(20)
                .turn(Math.toRadians(90))
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    Arm.moveArmTo(400);
                })
                .forward(5)
                .waitSeconds(2)
                .addDisplacementMarker(() -> {
                    Claw.Actuate_Claw_Top_Finger("open");
                })
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    Arm.setArmMotorPower(1);
                    Arm.moveArmTo(100);
                    while (Arm.Arm_Motor.isBusy()) {}
                    Arm.setArmMotorPower(0.3);
                })
                .back(5)
                .strafeRight(20)
                .forward(12)
                .build();

        TrajectorySequence PlaceYellowPixelOnLeftBackDrop = MecanumDrivebase.trajectorySequenceBuilder(PlacePixelOnLeftSpike.end())
                .back(30)
                .strafeLeft(10)
                .addDisplacementMarker(() -> {
                    Arm.moveArmTo(450);
                })
                .waitSeconds(3)
                .addDisplacementMarker(() -> {
                    Claw.Actuate_Claw_Top_Finger("open");
                })
                .waitSeconds(2)
                .back(5)
                .strafeRight(35)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        MecanumDrivebase.followTrajectorySequence(MoveToPropScanningLocation);

        TensorFlow.SetWebcamStreamStatus("start");

        PropLocation = "center";

        if (PropLocation == "left") {
            MecanumDrivebase.followTrajectorySequence(PlacePixelOnLeftSpike);
            MecanumDrivebase.followTrajectorySequence(PlaceYellowPixelOnLeftBackDrop);
        } else if (PropLocation == "center") {
            MecanumDrivebase.followTrajectorySequence(PlacePixelOnCenterSpike);
            MecanumDrivebase.followTrajectorySequence(PlaceYellowPixelOnCenterBackDrop);

        } else if (PropLocation == "right") {
            //do
        } else {
            //failsafe
        }




        //Scan for props

    }
}
