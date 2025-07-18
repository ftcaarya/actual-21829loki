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
import org.firstinspires.ftc.teamcode.vision.ColourMassDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

public class DetermineBarnacleSample {
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

    public DetermineBarnacleSample(double minArea, int left, int right, Pose2d poseGiven, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, PinpointDrive drive, AllMechs robot) {
        pose = poseGiven;
        this.hardwareMap = hardwareMap; // Store as instance variable
        this.minArea = minArea;
        this.left = left;
        this.right = right;

        this.robot = robot;

        colourMassDetectionProcessor = new ColourMassDetectionProcessor(
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
//
                .strafeToLinearHeading(new Vector2d(-52, -43), Math.toRadians(76))
                .stopAndAdd(
                        new SequentialAction(
                                new InstantAction(()-> robot.hold.setPosition(0.75)),
                                robot.setExtTarget(-210),
                                new SleepAction(1),
                                robot.checkColorRed())

                )

                .stopAndAdd(
                        new SequentialAction(

                                robot.intakeIn(),
                                new SleepAction(0.3),
                                robot.stopIntake()
                        )
                )
                .setTangent(0)

                .stopAndAdd(
                        new ParallelAction(
                                new InstantAction(() -> robot.hold.setPosition(.3)),
                                robot.setExtTarget(100)
                        )
                )
                .splineToLinearHeading(new Pose2d(-59, -59    , Math.toRadians(45)), Math.PI/2)
                .stopAndAdd(
                        new SequentialAction(
                                robot.armDown(),
                                robot.clawClose(),
                                new SleepAction(.5),
                                robot.setVertTarget(-2700),
                                new ParallelAction(
                                        robot.armUp(),
                                        robot.wristUp()
                                ),
                                new SleepAction(1),
                                robot.clawOpen(),
                                new SleepAction(.7),
                                robot.armWait(),
                                robot.wristDown(),
                                robot.setVertTarget(0),
                                new InstantAction(()-> robot.hold.setPosition(0.75))
                        )

                )
                .stopAndAdd(
                        new SequentialAction(
                                robot.intakeBack(),
                                new SleepAction(1),
                                robot.stopIntake()


                        )
                )
                .setTangent(Math.toRadians(90))
                // add the deposit action for the sample it holds
                .strafeToLinearHeading(new Vector2d(-58.5, -40), Math.toRadians(90))
                // add the intake for the middle sample
                .stopAndAdd(
                        new SequentialAction(
                                new SleepAction(.5),
                                robot.setExtTarget(-160),
                                new SleepAction(1),
                                robot.checkColorRed()

                        )
                )
//                                .splineToConstantHeading(new Vector2d(-55, -44), Math.toRadians(95))
                .stopAndAdd(
                        new SequentialAction(
                                robot.intakeIn(),
                                new SleepAction(0.5),
                                robot.stopIntake()

                        )
                )
//                                .setTangent(0)

                .stopAndAdd(
                        new ParallelAction(
                                new InstantAction(() -> robot.hold.setPosition(.3)),
                                robot.setExtTarget(100)
                        )
                )
                .splineToLinearHeading(new Pose2d(-59, -59, Math.toRadians(45)), -Math.PI)
                .stopAndAdd(
                        new SequentialAction(
                                robot.armDown(),
                                robot.clawClose(),
                                new SleepAction(.5),
                                robot.setVertTarget(-2700),
                                new SleepAction(0.5),
                                new ParallelAction(
                                        robot.armUp(),
                                        robot.wristUp()
                                ),
                                new SleepAction(2),
                                robot.clawOpen(),
                                new SleepAction(.5),
                                robot.armWait(),
                                robot.wristDown(),
                                robot.setVertTarget(0)
                        )

                )
                .setTangent(Math.toRadians(60))
//                                .splineToLinearHeading(new Pose2d(-45, -20, Math.toRadians(90)), Math.toRadians(90))
//                                .setTangent(Math.toRadians(90))

                .splineToLinearHeading(new Pose2d(-54, -52, Math.toRadians(45)), Math.toRadians(180))

                .build();
    }

    private static void generateMiddleTrajectory() {
        targetSampleTrajectory = drive.actionBuilder(pose)
                .strafeToLinearHeading(new Vector2d(-52, -43), Math.toRadians(74))
                .stopAndAdd(
                        new SequentialAction(
                                robot.stopIntake(),
                                robot.setExtTarget(-210),
                                new SleepAction(1),
                                robot.checkColorRed())

                )

                .stopAndAdd(
                        new SequentialAction(

                                robot.intakeIn(),
                                new SleepAction(0.7),
                                robot.stopIntake()
                        )
                )
                .setTangent(0)
                .stopAndAdd(
                        new ParallelAction(
                                new InstantAction(() -> robot.hold.setPosition(.3)),
                                robot.setExtTarget(120)
                        )
                )
                .splineToLinearHeading(new Pose2d(-58.5, -58.5, Math.toRadians(45)), -Math.PI)
                .stopAndAdd(
                        new SequentialAction(
                                robot.armDown(),
                                robot.clawClose(),
                                new SleepAction(.5),
                                robot.setVertTarget(-2700),
                                new ParallelAction(
                                        robot.armUp(),
                                        robot.wristUp()
                                ),
                                new SleepAction(1.5),
                                robot.clawOpen(),
                                new SleepAction(.7),
                                robot.armWait(),
                                robot.wristDown(),
                                robot.setVertTarget(0),
                                new InstantAction(()-> robot.hold.setPosition(0.75))
                        )

                )
                .stopAndAdd(
                        new SequentialAction(
                                robot.intakeBack(),
                                new SleepAction(1),
                                robot.stopIntake()


                        )
                )
                .setTangent((Math.PI - Math.atan((18/14.5))))
                .splineToLinearHeading(new Pose2d(-53.5, -39, Math.toRadians(133)), (Math.PI - Math.atan((18/14.5))))
                .stopAndAdd(
                        new SequentialAction(
                                new SleepAction(.5),
                                robot.setExtTarget(-310),
                                new SleepAction(1),
                                robot.checkColorRed()

                        )
                )
//                                .splineToConstantHeading(new Vector2d(-55, -44), Math.toRadians(95))
                .stopAndAdd(
                        new SequentialAction(
                                robot.intakeIn(),
                                new SleepAction(0.5),
                                robot.stopIntake()

                        )
                )
//                                .setTangent(0)

                .stopAndAdd(
                        new ParallelAction(
                                new InstantAction(() -> robot.hold.setPosition(.3)),
                                robot.setExtTarget(100)
                        )
                )
                .setTangent(Math.toRadians(180 + 120))
                .splineToLinearHeading(new Pose2d(-59, -59, Math.toRadians(45)), Math.toRadians(180 + 120))
                // deposit the sample that is with the robot
                .stopAndAdd(
                        new SequentialAction(
                                robot.armDown(),
                                robot.clawClose(),
                                new SleepAction(.5),
                                robot.setVertTarget(-2700),
                                new SleepAction(0.5),
                                new ParallelAction(
                                        robot.armUp(),
                                        robot.wristUp()
                                ),
                                new SleepAction(2),
                                robot.clawOpen(),
                                new SleepAction(.5),
                                robot.armWait(),
                                robot.wristDown(),
                                robot.setVertTarget(0)
                        )
                )
//                                        .setTangent(Math.toRadians(0))
//                                        .splineToLinearHeading(new Pose2d(-45, -20, Math.toRadians(90)), Math.toRadians(90))
//                                        .setTangent(Math.toRadians(90))
//                                        .splineToLinearHeading(new Pose2d(-28, -9, Math.toRadians(0)), Math.toRadians(0))
//                                        // add the intake from the submersible
//                                        .setReversed(true)
//                                        .splineToLinearHeading(new Pose2d(-45, -20, Math.toRadians(90)), Math.toRadians(-90))
//                                        .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-40, -55, Math.toRadians(45)), Math.toRadians(180))
                // deposit the sample that it has.
                .build();
    }

    private static void generateRightTrajectory() {
        targetSampleTrajectory = drive.actionBuilder(pose)
                .strafeToLinearHeading(new Vector2d(-52, -43), Math.toRadians(107))                // intake sample
                .stopAndAdd(
                        new SequentialAction(
                                robot.stopIntake(),
                                robot.setExtTarget(-260),
                                new SleepAction(1),
                                robot.checkColorRed())

                )
                .stopAndAdd(
                        new SequentialAction(

                                robot.intakeIn(),
                                new SleepAction(0.3),
                                robot.stopIntake()
                        )
                )
                .setTangent(Math.toRadians(180 + 120))

                .stopAndAdd(
                        new ParallelAction(
                                new InstantAction(() -> robot.hold.setPosition(.3)),
                                robot.setExtTarget(100)
                        )
                )
                .splineToLinearHeading(new Pose2d(-59, -59, Math.toRadians(45)), Math.toRadians(180 + 120))
                .stopAndAdd(
                        new SequentialAction(
                                robot.armDown(),
                                robot.clawClose(),
                                new SleepAction(.5),
                                robot.setVertTarget(-2700),
                                new ParallelAction(
                                        robot.armUp(),
                                        robot.wristUp()
                                ),
                                new SleepAction(1.5),
                                robot.clawOpen(),
                                new SleepAction(1),
                                robot.armWait(),
                                robot.wristDown(),
                                robot.setVertTarget(0),
                                new InstantAction(()-> robot.hold.setPosition(0.75))
                        )

                )
                .stopAndAdd(
                        new SequentialAction(
                                robot.intakeBack(),
                                new SleepAction(1),
                                robot.stopIntake()


                        )
                )
                .setTangent((Math.PI - Math.atan((18/14.5))))
                .splineToLinearHeading(new Pose2d(-53, -39, Math.toRadians(134)), (Math.PI - Math.atan((18/14.5))))
                .stopAndAdd(
                        new SequentialAction(
                                new SleepAction(.5),
                                robot.setExtTarget(-350),
                                new SleepAction(1),
                                robot.checkColorRed()

                        )
                )
//                                .splineToConstantHeading(new Vector2d(-55, -44), Math.toRadians(95))
                .stopAndAdd(
                        new SequentialAction(
                                robot.intakeIn(),
                                new SleepAction(0.5),
                                robot.stopIntake()

                        )
                )
//                                .setTangent(0)

                .stopAndAdd(
                        new ParallelAction(
                                new InstantAction(() -> robot.hold.setPosition(.3)),
                                robot.setExtTarget(100)
                        )
                )
                .setTangent(Math.toRadians(180 + 120))
                .splineToLinearHeading(new Pose2d(-59, -59, Math.toRadians(45)), Math.toRadians(180 + 120))
                // deposit the sample that is with the robot
                .stopAndAdd(
                        new SequentialAction(
                                robot.armDown(),
                                robot.clawClose(),
                                new SleepAction(.5),
                                robot.setVertTarget(-2700),
                                new SleepAction(0.5),
                                new ParallelAction(
                                        robot.armUp(),
                                        robot.wristUp()
                                ),
                                new SleepAction(2),
                                robot.clawOpen(),
                                new SleepAction(.5),
                                robot.armWait(),
                                robot.wristDown(),
                                robot.setVertTarget(0)
                        )

                )
//                                .setTangent(Math.toRadians(180))
//                                .splineToSplineHeading(new Pose2d(-59, -50, Math.toRadians(90)), Math.toRadians(180))
//                                .setTangent(Math.toRadians(90))
//                                .splineToConstantHeading(new Vector2d(-47, -6), Math.toRadians(0))
//                                .splineToLinearHeading(new Pose2d(-30, -9, Math.toRadians(0)), Math.toRadians(0))
//                                // intake sample from the sub
//                                .setTangent(Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(-47, -6, Math.toRadians(90)), Math.toRadians(180))
//                                .setTangent(Math.toRadians(180))
//                                .splineToConstantHeading(new Vector2d(-59, -50), Math.toRadians(270))
//                                .setTangent(0)
//                                .splineToLinearHeading(new Pose2d(-52, -52, Math.toRadians(45)), Math.toRadians(270))
//                                // deposit the sample that is with the robot
//                                .setTangent(Math.toRadians(0))
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