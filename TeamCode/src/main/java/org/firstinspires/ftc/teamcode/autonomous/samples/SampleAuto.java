package org.firstinspires.ftc.teamcode.autonomous.samples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.extraneous.AllMechs;
import org.firstinspires.ftc.teamcode.extraneous.determineBarnacle.DetermineBarnacleSample;
import org.firstinspires.ftc.teamcode.vision.ColourMassDetectionProcessor;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Sample Side Auto", group = "robot")
public class SampleAuto extends OpMode {
//    ActionSchedular actionSchedular;
    DetermineBarnacleSample determineBarnacle;
    PinpointDrive drive;
    private final FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
//    Pose2d startPose = new Pose2d(-35, -63, Math.toRadians(90));
    Pose2d passPose = new Pose2d(-50, -45, Math.toRadians(90));
    MultipleTelemetry mTelemetry;
    AllMechs robot;
    Action builder;

    ColourMassDetectionProcessor processor;

    @Override
    public void init() {
//        actionSchedular = new ActionSchedular();

        Pose2d startPose = new Pose2d(-35, -63, Math.toRadians(90));
        Pose2d initialPose = new Pose2d(-35, -63, Math.toRadians(90));
        drive = new PinpointDrive(hardwareMap, new Pose2d(-35, -63, Math.toRadians(90)));

        Pose2d passPose = new Pose2d(-50, -43, Math.toRadians(90));

        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new AllMechs(hardwareMap, 200, 400, gamepad1, gamepad2);

        determineBarnacle = new DetermineBarnacleSample(700, 100, 360, passPose, hardwareMap, gamepad1, gamepad2, drive, robot);

//        onInit();

        telemetry.addData("Testing Init:", "running");

        builder = drive.actionBuilder(new Pose2d(-35, -63, Math.toRadians(90)))
//                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-52, -52 , Math.toRadians(45)), -Math.PI)
                .build();
    }

    @Override
    public void start() {

        TrajectoryActionBuilder builder = drive.actionBuilder(new Pose2d(-35, -63, Math.toRadians(90)));
//        Action action = wholeSequence().build();

            runningActions.add(
                new ParallelAction(
                        new SequentialAction(drive.actionBuilder(new Pose2d(-35, -63, Math.toRadians(90)))
                                .stopAndAdd( new ParallelAction(
                                                new InstantAction(() -> robot.hold.setPosition(.3) ),
                                                robot.rotateHor(),
                                                robot.clawClose(),
                                        robot.setExtTarget(100)
                                        )


                                )
                                .setReversed(false)
                                .stopAndAdd(
                                        new SequentialAction(
                                                robot.setVertTarget(-2700),
                                                new ParallelAction(
                                                        robot.rotateHor(),
                                                        robot.clawClose(),
                                                        robot.armUp(),
                                                        robot.wristUp()
                                                )
                                        )
                                )

                                .splineToLinearHeading(new Pose2d(-59, -59 , Math.toRadians(45)), -Math.PI)
                                .stopAndAdd(
                                        new SequentialAction(
                                                new SleepAction(0.6),
                                                robot.clawOpen(),
                                                new SleepAction(0.6),
                                                robot.armWait(),
                                                robot.wristDown(),
                                                robot.setVertTarget(0),
                                                new InstantAction(()-> robot.hold.setPosition(0.8))

                                        )
                                )
                                .strafeToLinearHeading(new Vector2d(-48, -43), Math.toRadians(90))
                                .stopAndAdd(
                                        new SequentialAction(
                                                determineBarnacle.detectTarget(),
                                                new InstantAction(() -> {
                                                    ColourMassDetectionProcessor.PropPositions detected = DetermineBarnacleSample.getRecordedPosition();

                                                    telemetry.addData("=== DETECTION PHASE ===", "");
                                                    telemetry.addData("Detected Position", detected != null ? detected.toString() : "NULL");
                                                    telemetry.update();

                                                    telemetry.addData("=== GENERATION PHASE ===", "");
                                                    DetermineBarnacleSample.generateTargetTrajectory();

                                                    boolean hasTrajectory = DetermineBarnacleSample.hasTrajectory();
                                                    telemetry.addData("Trajectory Generated", hasTrajectory ? "YES" : "NO");

                                                    if (hasTrajectory) {
                                                        Action traj = DetermineBarnacleSample.getTargetSampleTrajectory();
                                                        if (traj != null) {
                                                            telemetry.addData("Adding trajectory to runningActions", "SUCCESS");
                                                            telemetry.addData("Current runningActions size", runningActions.size());

                                                            runningActions.add(traj);

                                                            telemetry.addData("New runningActions size", runningActions.size());
                                                            telemetry.addData("Trajectory added for", detected.toString());
                                                        } else {
                                                            telemetry.addData("ERROR", "getTargetSampleTrajectory returned null");
                                                        }
                                                    } else {
                                                        telemetry.addData("ERROR", "No trajectory was generated");
                                                    }

                                                    telemetry.update();
                                                }
                                                )
                                        )
                                )


                                .build()),
                        robot.updateExtPID(),
                        robot.updateVertPID()
                )
        );
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        telemetry.addData("=== MAIN LOOP DEBUG ===", "");
        telemetry.addData("Total Actions", runningActions.size());

        List<Action> newActions = new ArrayList<>();
        for (int i = 0; i < runningActions.size(); i++) {
            Action action = runningActions.get(i);
            telemetry.addData("Running Action " + i, action.getClass().getSimpleName());

            action.preview(packet.fieldOverlay());
            boolean stillRunning = action.run(packet);

            if (stillRunning) {
                newActions.add(action);
                telemetry.addData("Action " + i + " Status", "RUNNING");
            } else {
                telemetry.addData("Action " + i + " Status", "COMPLETED");
            }
        }

        runningActions = newActions;
        telemetry.addData("Actions After Cleanup", runningActions.size());
        telemetry.update();
        dash.sendTelemetryPacket(packet);
    }


}
