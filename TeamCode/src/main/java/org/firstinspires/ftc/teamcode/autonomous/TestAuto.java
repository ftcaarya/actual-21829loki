package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.EnhancedColorDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.ArrayList;
import java.util.List;

@Config
@Autonomous(name = "Vision Test Auto")
public class TestAuto extends LinearOpMode {
    // FTC Dashboard configurable variables
    public static double MIN_AREA = 1000;
    public static double LEFT_DIVIDER = 200;
    public static double RIGHT_DIVIDER = 600;

    public static double START_X = -35;
    public static double START_Y = -63;
    public static double START_HEADING = 90; // in degrees

    public static double TEST_X = -54;
    public static double TEST_Y = -45;
    public static double TEST_HEADING = 90; // in degrees

    private MecanumDrive drive;
    private VisionPortal visionPortal;
    private EnhancedColorDetectionProcessor visionProcessor;
    private FtcDashboard dashboard;
    private boolean atTestPosition = false;

    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(START_X, START_Y, Math.toRadians(START_HEADING));
        drive = new MecanumDrive(hardwareMap, startPose);

        visionProcessor = new EnhancedColorDetectionProcessor(
                () -> MIN_AREA,
                () -> LEFT_DIVIDER,
                () -> RIGHT_DIVIDER,
                EnhancedColorDetectionProcessor.StartPositions.SPECIMEN
        );

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(visionProcessor)
                .build();

        dashboard = FtcDashboard.getInstance();

        // Wait for start
        telemetry.addData("Status", "Ready to start");
        telemetry.addData("Instructions", "Use gamepad controls:");
        telemetry.addLine("A - Move to test position");
        telemetry.addLine("B - Return to start position");
        telemetry.addLine("Vision runs continuously");
        telemetry.update();

        waitForStart();

        runningActions.add(
                new ParallelAction(
                        updateVisionTelemetry(),
                        updateDashboard()
                )


        );

        // Main loop
        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();


            if (gamepad1.a) {
                runningActions.add(
                        moveToTestPosition().build()
                );
            }

            if (gamepad1.b) {
                runningActions.add(
                        returnToStartPosition().build()
                );
            }


            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;

            dash.sendTelemetryPacket(packet);
        }

        // Cleanup
        if (visionPortal != null) {
            visionPortal.close();
        }
    }


    private TrajectoryActionBuilder moveToTestPosition() {
        drive.localizer.update();
        drive.updatePoseEstimate();
        Pose2d currentPose = new Pose2d(drive.pose.position.x, drive.pose.position.y, drive.pose.heading.real);

        TrajectoryActionBuilder moveAction = drive.actionBuilder(currentPose)
                .strafeToLinearHeading(new Vector2d(TEST_X, TEST_Y), Math.toRadians(90));

        telemetry.addData("Status", "Moved to test position");
        telemetry.update();

        return moveAction;

    }

    private TrajectoryActionBuilder returnToStartPosition() {
        drive.localizer.update();
        drive.updatePoseEstimate();
        Pose2d currentPose = new Pose2d(drive.pose.position.x, drive.pose.position.y, drive.pose.heading.real);
        Pose2d startPose = new Pose2d(START_X, START_Y, Math.toRadians(START_HEADING));

        TrajectoryActionBuilder returnAction = drive.actionBuilder(currentPose)
                .splineToLinearHeading(startPose, Math.toRadians(110));


        telemetry.addData("Status", "Returned to start position");
        telemetry.update();

        return returnAction;
    }

    Action updateVisionTelemetry() {
        return p -> {
            EnhancedColorDetectionProcessor.PropPositions propPosition =
                    visionProcessor.getRecordedPropPosition();

            telemetry.addData("=== VISION DATA ===", "");
            telemetry.addData("Prop Position", propPosition.toString());
            telemetry.addData("Contour X", "%.2f", visionProcessor.getMostSaturatedContourX());
            telemetry.addData("Contour Y", "%.2f", visionProcessor.getMostSaturatedContourY());
            telemetry.addData("Contour Area", "%.2f", visionProcessor.getMostSaturatedContourArea());
            telemetry.addData("Contour Saturation", "%.2f", visionProcessor.getMostSaturatedContourSaturation());

            telemetry.addData("=== CONFIGURABLE VALUES ===", "");
            telemetry.addData("Min Area", MIN_AREA);
            telemetry.addData("Left Divider", LEFT_DIVIDER);
            telemetry.addData("Right Divider", RIGHT_DIVIDER);

            telemetry.addData("=== ROBOT STATUS ===", "");
//        telemetry.addData("Current Position", atTestPosition ? "Test Position" : "Start Position");
//        telemetry.addData("Robot X", "%.2f", drive.pose.position.x);
//        telemetry.addData("Robot Y", "%.2f", drive.pose.position.y);
//        telemetry.addData("Robot Heading", "%.2f°", Math.toDegrees(drive.pose.heading.log()));

            telemetry.addData("=== CONTROLS ===", "");
            telemetry.addData("A Button", "Move to test position");
            telemetry.addData("B Button", "Return to start");

            telemetry.update();

            return true;
        };
    }



    private Action updateDashboard() {
        return p -> {
            TelemetryPacket packet = new TelemetryPacket();

            // Add vision data to dashboard
            packet.put("Prop Position", visionProcessor.getRecordedPropPosition().toString());
            packet.put("Contour X", visionProcessor.getMostSaturatedContourX());
            packet.put("Contour Y", visionProcessor.getMostSaturatedContourY());
            packet.put("Contour Area", visionProcessor.getMostSaturatedContourArea());
            packet.put("Contour Saturation", visionProcessor.getMostSaturatedContourSaturation());

            // Add configurable values
            packet.put("Min Area", MIN_AREA);
            packet.put("Left Divider", LEFT_DIVIDER);
            packet.put("Right Divider", RIGHT_DIVIDER);

            // Add robot position
            drive.updatePoseEstimate();

            packet.put("Robot X", drive.pose.position.x);
            packet.put("Robot Y", drive.pose.position.y);
            packet.put("Robot Heading (deg)", Math.toDegrees(drive.pose.heading.log()));
            packet.put("At Test Position", atTestPosition);

            dashboard.sendTelemetryPacket(packet);

            return true;
        };

    }
}