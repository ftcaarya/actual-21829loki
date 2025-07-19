package org.firstinspires.ftc.teamcode.extraneous.determineBarnacle;

import static org.firstinspires.ftc.vision.VisionPortal.CameraState.STREAMING;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.extraneous.AllMechs;
import org.firstinspires.ftc.teamcode.vision.ColourMassDetectionProcessorSpec;
import org.firstinspires.ftc.vision.VisionPortal;

public class DetermineBarnacleBlueSpec {
    public VisionPortal visionPortal;
    public ColourMassDetectionProcessorSpec colourMassDetectionProcessor;

    static PinpointDrive drive;
    static AllMechs robot;
    private HardwareMap hardwareMap;

    static ColourMassDetectionProcessorSpec.PropPositions recordedBarnaclePosition;
    private static Action targetSampleTrajectory;
    static Pose2d pose;

    private double minArea;
    private int left, right;


    public DetermineBarnacleBlueSpec(double minArea, int left, int right, Pose2d poseGiven, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, PinpointDrive drive, AllMechs robot) {
        pose = poseGiven;
        this.hardwareMap = hardwareMap;
        this.minArea = minArea;
        this.left = left;
        this.right = right;

        this.robot = robot;

        colourMassDetectionProcessor = new ColourMassDetectionProcessorSpec(
                () -> this.minArea,
                () -> this.left,
                () -> this.right
        );

        this.drive = drive;

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(colourMassDetectionProcessor)
                .build();

        FtcDashboard.getInstance().startCameraStream(visionPortal, 60);
    }

    public Action detectTarget() {
        return new InstantAction(() -> {
            try {
                // Actually wait for camera to be ready and process frames
                int attempts = 0;
                while (visionPortal.getCameraState() != STREAMING && attempts < 100) {
                    try {
                        Thread.sleep(50); // Wait 50ms between checks
                        attempts++;
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                        break;
                    }
                }


                // Get the detection result
                if (colourMassDetectionProcessor != null && visionPortal != null) {
                    if (visionPortal.getCameraState() == STREAMING) {
                        ColourMassDetectionProcessorSpec.PropPositions position = colourMassDetectionProcessor.getRecordedPropPosition();
                        recordedBarnaclePosition = (position != null) ? position : ColourMassDetectionProcessorSpec.PropPositions.UNFOUND;
                    } else {
                        ColourMassDetectionProcessorSpec.PropPositions position = colourMassDetectionProcessor.getRecordedPropPosition();
                        recordedBarnaclePosition = (position != null) ? position : ColourMassDetectionProcessorSpec.PropPositions.UNFOUND;
                    }
                } else {
                    recordedBarnaclePosition = ColourMassDetectionProcessorSpec.PropPositions.UNFOUND;
                }

                // Clean up vision portal
                if (visionPortal != null) {
                    visionPortal.stopLiveView();
                    visionPortal.stopStreaming();
                    visionPortal.close(); // Important: close to free resources
                }

            } catch (Exception e) {
                recordedBarnaclePosition = ColourMassDetectionProcessorSpec.PropPositions.UNFOUND;
                // Clean up on error
                if (visionPortal != null) {
                    try {
                        visionPortal.close();
                    } catch (Exception closeException) {
                        // Ignore close errors
                    }
                }
            }
        });
    }

    // Add a method to get the current recorded position for debugging
    public static ColourMassDetectionProcessorSpec.PropPositions getRecordedPosition() {
        return recordedBarnaclePosition;
    }

    /**
     * Universal trajectory generator that chooses the appropriate trajectory based on detection
     */
    public static void generateTargetTrajectory() {
        // Ensure we have a valid position
        if (recordedBarnaclePosition == null) {
            recordedBarnaclePosition = ColourMassDetectionProcessorSpec.PropPositions.UNFOUND;
        }

        // Generate the appropriate trajectory based on detection
        switch (recordedBarnaclePosition) {
            case LEFT:
                generateLeftTrajectory();
                break;
            case MIDDLE:
                generateLeftTrajectory();
                break;
            case RIGHT:
                generateRightTrajectory();
                break;
            case UNFOUND:
                generateRightTrajectory();
                break;
        }
    }

    private static void generateLeftTrajectory() {
        targetSampleTrajectory = drive.actionBuilder(pose)
//                .strafeToLinearHeading(new Vector2d(-52, -43), Math.toRadians(90))
                .stopAndAdd(
                        new ParallelAction(
                                robot.setExtTarget(100),
                                new InstantAction(() -> robot.hold.setPosition(.3))

                        )

                )
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(73, -45, Math.toRadians(270)), Math.toRadians(270))
                .stopAndAdd(
                        new SequentialAction(
                                new SequentialAction(
                                        new InstantAction(() -> robot.intake.setPower(.6)),
                                        new SleepAction(.5),
                                        new InstantAction(() -> robot.intake.setPower(0)),
                                        new SleepAction(1),
                                        robot.getSpec()
                                )
                        )
                )
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(73, -62), Math.toRadians(270))
                .stopAndAdd(
                        new SequentialAction(

                                new SleepAction(0.5),
                                robot.clawClose(),
                                new SleepAction(0.5),
                                robot.putSpec()
                        )
                )
                .setTangent(Math.toRadians(150))
                .splineToLinearHeading(new Pose2d(-5, -35, Math.toRadians(270)), Math.toRadians(100))
                .stopAndAdd(
                        new SequentialAction(
                                robot.clawClose()
                        )
                )
                .setTangent(Math.toRadians(230))
                .splineToLinearHeading(new Pose2d(-43, -35, Math.toRadians(180)), Math.toRadians(180))


                .build();
    }

    private static void generateMiddleTrajectory() {
        targetSampleTrajectory = drive.actionBuilder(pose)
                .turnTo(Math.toRadians(65))
                .setTangent(0)
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
    }

    private static void generateRightTrajectory() {
        targetSampleTrajectory = drive.actionBuilder(pose)
                .setTangent(Math.toRadians(110))
                .splineToConstantHeading(new Vector2d(0, -33), Math.toRadians(110))
                .stopAndAdd(
                        new SequentialAction(

                                robot.clawOpen(),

                                robot.armWait(),
                                robot.wristDown(),
                                robot.setVertTarget(0),
                                new InstantAction(()-> robot.hold.setPosition(.7))
                        )
                )
                .setTangent(Math.toRadians(300))
                .splineToLinearHeading(new Pose2d(50, -45, Math.toRadians(90)), Math.toRadians(70))
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(73, -45), Math.toRadians(0))
                // now after vision processing it starts
//
                .stopAndAdd(
                        new SequentialAction(
                                new InstantAction(() -> robot.hold.setPosition(.78)),
                                robot.setExtTarget(-180),
                                new SleepAction(1),
                                robot.checkColorBlue()
                        )
                )
                .stopAndAdd(
                        robot.setExtTarget(100)
                )
                .setTangent(180)
                .splineToLinearHeading(new Pose2d(63, -45, Math.toRadians(250)), Math.toRadians(180))
                .stopAndAdd(
                        new SequentialAction(
                                new InstantAction(() -> robot.intake.setPower(.6)),
                                new SleepAction(.5),
                                new InstantAction(() -> robot.intake.setPower(0)),
                                new InstantAction(()-> robot.hold.setPosition(0.75))
                        )
                )
                .splineToLinearHeading(new Pose2d(63, -45, Math.toRadians(90)), Math.toRadians(180))
                .stopAndAdd(
                        new SequentialAction(
                                new InstantAction(() -> robot.hold.setPosition(.78)),
                                robot.setExtTarget(-180),
                                new SleepAction(1),
                                robot.checkColorBlue()
                        )
                )

                .stopAndAdd(
                        new ParallelAction(
                                new InstantAction(() -> robot.hold.setPosition(.3)),
                                robot.setExtTarget(100)
                        )
                )
                .turnTo(Math.toRadians(270))

                .stopAndAdd(
                        new SequentialAction(
                                new SequentialAction(
                                        new InstantAction(() -> robot.intake.setPower(.6)),
                                        new SleepAction(.5),
                                        new InstantAction(() -> robot.intake.setPower(0)),
                                        new SleepAction(1),
                                        robot.getSpec()
                                )
                        )
                )
                .setTangent(Math.toRadians(250))
                .splineToConstantHeading(new Vector2d(55, -62), Math.toRadians(250))
                .stopAndAdd(
                        new SequentialAction(

                                new SleepAction(0.5),
                                robot.clawClose(),
                                new SleepAction(0.5),
                                robot.putSpec()
                        )
                )
                .setTangent(Math.toRadians(140))
                .splineToLinearHeading(new Pose2d(-5, -32, Math.toRadians(270)), Math.toRadians(100))

                .build();
    }


    /**
     * @deprecated Use generateTargetTrajectory() instead for automatic detection-based generation
     */
    public static void generateTargetTrajectoryLeft() {
        // Keep for backward compatibility, but now it just calls the universal method
        generateTargetTrajectory();
    }

    /**
     * @deprecated Use generateTargetTrajectory() instead for automatic detection-based generation
     */
    public static void generateTargetTrajectoryRight() {
        // Keep for backward compatibility, but now it just calls the universal method
        generateTargetTrajectory();
    }

    public static Action getTargetSampleTrajectory() {
        return targetSampleTrajectory;
    }
    public static boolean hasTrajectory() {
        return targetSampleTrajectory != null;
    }
}
