package org.firstinspires.ftc.teamcode.extraneous;

import static org.firstinspires.ftc.vision.VisionPortal.CameraState.STREAMING;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.vision.ColourMassDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

public class DetermineBarnacle {
    public VisionPortal visionPortal;
    public ColourMassDetectionProcessor colourMassDetectionProcessor;
    static PinpointDrive drive;
    static AllMechs robot;
    private HardwareMap hardwareMap; // Remove static and use instance variable

    static ColourMassDetectionProcessor.PropPositions recordedBarnaclePosition;
    private static Action targetSampleTrajectory;
    static Pose2d pose;

    private double minArea;
    private int left, right;

    public DetermineBarnacle(double minArea, int left, int right, Pose2d poseGiven, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, PinpointDrive drive) {
        pose = poseGiven;
        this.hardwareMap = hardwareMap; // Store as instance variable
        this.minArea = minArea;
        this.left = left;
        this.right = right;

        robot = new AllMechs(hardwareMap, left, right, gamepad1, gamepad2);

        colourMassDetectionProcessor = new ColourMassDetectionProcessor(
                () -> this.minArea,
                () -> this.left,
                () -> this.right
        );

        this.drive = drive;
    }

    public Action detectTarget() {
        return new InstantAction(() -> {
            try {
                // Create VisionPortal
                visionPortal = new VisionPortal.Builder()
                        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                        .addProcessor(colourMassDetectionProcessor)
                        .build();

                FtcDashboard.getInstance().startCameraStream(visionPortal, 60);

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

                // Give it extra time to process frames
                try {
                    Thread.sleep(2000); // Wait 2 seconds for processing
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }

                // Get the detection result
                if (colourMassDetectionProcessor != null && visionPortal != null) {
                    if (visionPortal.getCameraState() == STREAMING) {
                        ColourMassDetectionProcessor.PropPositions position = colourMassDetectionProcessor.getRecordedPropPosition();
                        recordedBarnaclePosition = (position != null) ? position : ColourMassDetectionProcessor.PropPositions.UNFOUND;
                    } else {
                        ColourMassDetectionProcessor.PropPositions position = colourMassDetectionProcessor.getRecordedPropPosition();
                        recordedBarnaclePosition = (position != null) ? position : ColourMassDetectionProcessor.PropPositions.UNFOUND;
                    }
                } else {
                    recordedBarnaclePosition = ColourMassDetectionProcessor.PropPositions.UNFOUND;
                }

                // Clean up vision portal
                if (visionPortal != null) {
                    visionPortal.stopLiveView();
                    visionPortal.stopStreaming();
                    visionPortal.close(); // Important: close to free resources
                }

            } catch (Exception e) {
                recordedBarnaclePosition = ColourMassDetectionProcessor.PropPositions.UNFOUND;
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
    public static ColourMassDetectionProcessor.PropPositions getRecordedPosition() {
        return recordedBarnaclePosition;
    }

    /**
     * Universal trajectory generator that chooses the appropriate trajectory based on detection
     */
    public static void generateTargetTrajectory() {
        // Ensure we have a valid position
        if (recordedBarnaclePosition == null) {
            recordedBarnaclePosition = ColourMassDetectionProcessor.PropPositions.UNFOUND;
        }

        // Generate the appropriate trajectory based on detection
        switch (recordedBarnaclePosition) {
            case LEFT:
                generateLeftTrajectory();
                break;
            case MIDDLE:
                generateMiddleTrajectory();
                break;
            case RIGHT:
                generateRightTrajectory();
                break;
            case UNFOUND:
                generateLeftTrajectory();
                break;
        }
    }

    private static void generateLeftTrajectory() {
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