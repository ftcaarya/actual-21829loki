package org.firstinspires.ftc.teamcode.extraneous;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.vision.VisionPortal.CameraState.STREAMING;

import android.app.Notification;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.ColourMassDetectionOpMode;
import org.firstinspires.ftc.teamcode.vision.ColourMassDetectionProcessor;
import org.firstinspires.ftc.teamcode.vision.EnhancedColorDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

import java.nio.charset.CharacterCodingException;

public class DetermineBarnacle {
    private VisionPortal visionPortal;
    private ColourMassDetectionProcessor colourMassDetectionProcessor;
    static MecanumDrive drive;
    static AllMechs robot;

    // HSV takes the form: (HUE, SATURATION, VALUE)
    // the domains are: ([0, 180], [0, 255], [0, 255])
//    double lowerH = 150; // the lower hsv threshold for your detection
//    double upperH = 180; // the upper hsv threshold for your detection
//    double minArea = 100; // the minimum area for the detection to consider for your prop

    static ColourMassDetectionProcessor.PropPositions recordedBarnaclePosition;

    private static Action targetSampleTrajectory;

    static Pose2d pose;

    public DetermineBarnacle(double minArea, int left, int right, Pose2d poseGiven, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        pose = poseGiven;

        robot = new AllMechs(hardwareMap, left, right, gamepad1, gamepad2);

        colourMassDetectionProcessor = new ColourMassDetectionProcessor(
                () -> minArea,
                () -> left,
                () -> right
        );

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(colourMassDetectionProcessor)
                .build();

        drive = new MecanumDrive(hardwareMap, pose);

    }


    public Action detectTarget() {

            return p -> {
                if (visionPortal.getCameraState() == STREAMING) {
                    recordedBarnaclePosition = colourMassDetectionProcessor.getRecordedPropPosition();
                } else {
                    recordedBarnaclePosition = colourMassDetectionProcessor.getRecordedPropPosition();
                }

                visionPortal.stopLiveView();
                visionPortal.stopStreaming();

                return false;
            };

    }

    public static void generateTargetTrajectoryLeft() {

        // switch through the detected cases and run separate trajectories for each of them.
        switch (recordedBarnaclePosition) {
            // actually means middle for now
            case MIDDLE:
                targetSampleTrajectory = drive.actionBuilder(pose)
                        .turnTo(Math.toRadians(65))
//                        .stopAndAdd(
//                                new ParallelAction(
//                                        robot.setHorTarget(700),
//                                        robot.checkColorRed()
//                                )
//                        )
                        .setTangent(0)
//                        .stopAndAdd(new ParallelAction(
//
//                        ))
                        .splineToLinearHeading(new Pose2d(-52, -52, Math.toRadians(45)), -Math.PI)
                        // add the deposit action for the sample it holds
                        .setTangent(0)
                        .splineToLinearHeading(new Pose2d(-56, -44, Math.toRadians(95)), -Math.toRadians(180))
                        // add the intake for the middle sample
                        .setTangent(0)
                        .splineToLinearHeading(new Pose2d(-52, -52, Math.toRadians(45)), -Math.PI)
                        // add the deposit action for the sample it holds
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(-45, -20, Math.toRadians(90)), Math.toRadians(90))
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(-28, -9, Math.toRadians(0)), Math.toRadians(0))
                        // add the intake from the submersible
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-45, -20, Math.toRadians(90)), Math.toRadians(-90))
                        .setTangent(Math.toRadians(-90))
                        .splineToLinearHeading(new Pose2d(-52, -52, Math.toRadians(45)), Math.toRadians(180))
                        // deposit the sample that it has.
                        .build();
                break;
            // actually means left
            case LEFT:
                targetSampleTrajectory = drive.actionBuilder(pose)
                        .turnTo(Math.toRadians(65))
                        // intake the sample
                        .setTangent(180 + 65)
                        .splineToLinearHeading(new Pose2d(-52, -52, Math.toRadians(45)), -Math.PI)
                        // deposit the sample that is with the robot
                        .setTangent(0)
                        .splineToLinearHeading(new Pose2d(-56, -44, Math.toRadians(130)), -Math.toRadians(180))
                        // intake the sample
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-52, -52, Math.toRadians(45)), Math.toRadians(-90))
                        // deposit the sample that is with the robot
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(-40, -20, Math.toRadians(90)), Math.toRadians(90))
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(-28, -9, Math.toRadians(0)), Math.toRadians(0))
                        // intake the sample from the sub
                        .setTangent(Math.PI)
                        .splineToLinearHeading(new Pose2d(-40, -20, Math.toRadians(90)), Math.toRadians(-90))
                        .setTangent(Math.toRadians(-90))
                        .splineToLinearHeading(new Pose2d(-52, -52, Math.toRadians(45)), Math.toRadians(180))
                        // deposit the sample that is with the robot
                        .build();

                break;

                // stays right
            case RIGHT:
                targetSampleTrajectory = drive.actionBuilder(pose)
                        .turnTo(Math.toRadians(100))
                        // intake sample
                        .setTangent(Math.toRadians(180 + 120))
                        .splineToLinearHeading(new Pose2d(-52, -52, Math.toRadians(45)), Math.toRadians(180 + 120))
                        // deposit the sample that is with the robot
                        .setTangent((Math.PI - Math.atan((18/14.5))))
                        .splineToLinearHeading(new Pose2d(-54, -44, (Math.PI - Math.atan((18/14.5)))), (Math.PI - Math.atan((18/14.5))))
                        // intake sample
                        .setTangent(Math.toRadians(180 + 120))
                        .splineToLinearHeading(new Pose2d(-52, -52, Math.toRadians(45)), Math.toRadians(180 + 120))
                        // deposit the sample that is with the robot
                        .setTangent(Math.toRadians(180))
                        .splineToSplineHeading(new Pose2d(-59, -50, Math.toRadians(90)), Math.toRadians(180))
                        .setTangent(Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(-47, -6), Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(-30, -9, Math.toRadians(0)), Math.toRadians(0))
                        // intake sample from the sub
                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(-47, -6, Math.toRadians(90)), Math.toRadians(180))
                        .setTangent(Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(-59, -50), Math.toRadians(270))
                        .setTangent(0)
                        .splineToLinearHeading(new Pose2d(-52, -52, Math.toRadians(45)), Math.toRadians(270))
                        // deposit the sample that is with the robot
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(-30, -55, Math.toRadians(0)), Math.toRadians(0))
                        .build();



                break;

            case UNFOUND:
                // just do all of them.

        }

    }

    public static void generateTargetTrajectoryRight() {

        // switch through the detected cases and run separate trajectories for each of them.
        switch (recordedBarnaclePosition) {
            // stays left
            case LEFT:
                targetSampleTrajectory = drive.actionBuilder(pose)
                        .turnTo(Math.toRadians(80))
                        // intake the sample and pass it through
                        .setTangent(Math.toRadians(290))
                        .splineToLinearHeading(new Pose2d(59, -52, Math.toRadians(100)), Math.toRadians(290))
                        // dropped the sample off in the OZ
                        .setTangent(Math.PI/2)
                        .splineToLinearHeading(new Pose2d(59, -42, Math.toRadians(60)), Math.toRadians(90))
                        // intake the sample and pass it through
                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(57, -62, Math.toRadians(-270)), Math.toRadians(270))
                        // dropped the sample off in the OZ
                        // pick up spec from wall
                        .setTangent(Math.toRadians(150))
                        .splineToLinearHeading(new Pose2d(-5, -35, Math.toRadians(270)), Math.toRadians(100))
                        // score specimen that is with the robot
                        .setTangent(Math.toRadians(320))
                        .splineToConstantHeading(new Vector2d(40, -62), Math.toRadians(270))
                        // pick up spec from wall
                        .setTangent(Math.toRadians(320-180))
                        .splineToConstantHeading(new Vector2d(3, -35), Math.toRadians(320-180))
                        // score specimen that is with the robot
                        .setTangent(Math.toRadians(320))
                        .splineToConstantHeading(new Vector2d(40, -62), Math.toRadians(270))
                        // pick up spec from wall
                        .setTangent(Math.toRadians(150))
                        .splineToConstantHeading(new Vector2d(-3, -35), Math.toRadians(150))
                        // score specimen that is with the robot
                        .setTangent(Math.toRadians(230))

                        // go park
                        .splineToLinearHeading(new Pose2d(-37, -35, Math.toRadians(180)), Math.toRadians(180))

                        .build();
                break;
            // actually means right
            // supposed to be the middle
            case RIGHT:
                targetSampleTrajectory = drive.actionBuilder(pose)
                        .turnTo(Math.toRadians(110))
                        // intake the sample and pass it through
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(new Pose2d(54, -55, Math.toRadians(60)), Math.toRadians(270))
                        // dropped the sample in the OZ
                        .setTangent(Math.toRadians(60))
                        .splineToConstantHeading(new Vector2d(59, -42), Math.toRadians(60))
                        // intake the sample and pass it through
                        .setTangent(Math.toRadians(180 + 60))
                        .splineToLinearHeading(new Pose2d(54, -62, Math.toRadians(90)), Math.toRadians(270))
                        // dropped the sample in the OZ
                        // pick up spec from wall
                        .setTangent(Math.toRadians(140))
                        .splineToLinearHeading(new Pose2d(-5, -35, Math.toRadians(270)), Math.toRadians(100))
                        // score the specimen that is with the robot
                        .setTangent(Math.toRadians(320))
                        .splineToConstantHeading(new Vector2d(40, -62), Math.toRadians(270))
                        // pick up spec from wall
                        .setTangent(Math.toRadians(320-180))
                        .splineToConstantHeading(new Vector2d(3, -35), Math.toRadians(320-180))
                        // score the specimen that is with the robot
                        .setTangent(Math.toRadians(320))
                        .splineToConstantHeading(new Vector2d(40, -62), Math.toRadians(270))
                        // pick up spec from wall
                        .setTangent(Math.toRadians(150))
                        .splineToConstantHeading(new Vector2d(-3, -35), Math.toRadians(150))
                        // score the specimen that is with the robot
                        .setTangent(Math.toRadians(230))

                        // go park bum!
                        .splineToLinearHeading(new Pose2d(-20, -35, Math.toRadians(180)), Math.toRadians(180))

                        .build();

                break;

                // actually middle
            // supposed to be the right
            case MIDDLE:
                targetSampleTrajectory = drive.actionBuilder(pose)
                        .turnTo(Math.toRadians(80))
                        // intake the sample and pass it through
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(new Pose2d(55, -55, Math.toRadians(105)), Math.toRadians(270))
                        // dropped the sample off in the OZ
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(55, -45, Math.toRadians(105)), Math.toRadians(90))
                        // intake the sample and pass it through
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(new Pose2d(55, -59, Math.toRadians(90)), Math.toRadians(270))
                        // dropped the sample off in the OZ
                        // pick up spec from wall
                        .setTangent(Math.toRadians(140))
                        .splineToLinearHeading(new Pose2d(-5, -35, Math.toRadians(270)), Math.toRadians(100))
                        // score the specimen that is with the robot
                        .setTangent(Math.toRadians(320))
                        .splineToConstantHeading(new Vector2d(40, -62), Math.toRadians(270))
                        // pick up spec from wall
                        .setTangent(Math.toRadians(320-180))
                        .splineToConstantHeading(new Vector2d(3, -35), Math.toRadians(320-180))
                        // score the specimen that is with the robot
                        .setTangent(Math.toRadians(320))
                        .splineToConstantHeading(new Vector2d(40, -62), Math.toRadians(270))
                        // pick up spec from wall
                        .setTangent(Math.toRadians(150))
                        .splineToConstantHeading(new Vector2d(-3, -35), Math.toRadians(150))
                        // score the specimen that is with the robot
                        // in parking already YAYAYAYAYYAYAY!!!
                        .build();



                break;

            case UNFOUND:
                // just do all of them.

        }

    }

    public static Action getTargetSampleTrajectory() {
        return targetSampleTrajectory;
    }






}
