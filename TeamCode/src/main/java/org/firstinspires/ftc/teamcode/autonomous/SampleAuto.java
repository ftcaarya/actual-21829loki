package org.firstinspires.ftc.teamcode.autonomous;

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
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.extraneous.ActionSchedular;
import org.firstinspires.ftc.teamcode.extraneous.AllMechs;
import org.firstinspires.ftc.teamcode.extraneous.DetermineBarnacle;
import org.firstinspires.ftc.teamcode.vision.ColourMassDetectionProcessor;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Sample Side Auto", group = "robot")
public class SampleAuto extends OpMode {
//    ActionSchedular actionSchedular;
    DetermineBarnacle determineBarnacle;
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

        Pose2d passPose = new Pose2d(-54, -45, Math.toRadians(90));

        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new AllMechs(hardwareMap, 200, 400, gamepad1, gamepad2);

        determineBarnacle = new DetermineBarnacle(700, 100, 330, passPose, hardwareMap, gamepad1, gamepad2, drive);

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
                                .stopAndAdd(
                                        new InstantAction(() -> robot.hold.setPosition(.75))

                                )
                                .setReversed(false)
                                .splineToLinearHeading(new Pose2d(-60, -60 , Math.toRadians(45)), -Math.PI)
                                .stopAndAdd(
                                        new SequentialAction(
                                                robot.setVertTarget(-2700),
                                                new ParallelAction(
                                                        robot.clawClose(),
                                                        robot.armUp(),
                                                        robot.wristUp()
                                                ),
                                                new SleepAction(2),
                                                robot.clawOpen(),
                                                new SleepAction(.5),
                                                robot.armWait(),
                                                robot.wristDown(),
                                                robot.setVertTarget(0),
                                                robot.intakeUp()
                                        )


                                )
                                .strafeToLinearHeading(new Vector2d(-50, -45), Math.toRadians(90))
                                .stopAndAdd(
                                        new SequentialAction(
                                                determineBarnacle.detectTarget(),
                                                new InstantAction(() -> {
                                                    ColourMassDetectionProcessor.PropPositions detected = DetermineBarnacle.getRecordedPosition();

                                                    telemetry.addData("=== DETECTION PHASE ===", "");
                                                    telemetry.addData("Detected Position", detected != null ? detected.toString() : "NULL");
                                                    telemetry.update();

                                                    telemetry.addData("=== GENERATION PHASE ===", "");
                                                    DetermineBarnacle.generateTargetTrajectory();

                                                    boolean hasTrajectory = DetermineBarnacle.hasTrajectory();
                                                    telemetry.addData("Trajectory Generated", hasTrajectory ? "YES" : "NO");

                                                    if (hasTrajectory) {
                                                        Action traj = DetermineBarnacle.getTargetSampleTrajectory();
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



    private TrajectoryActionBuilder wholeSequence() {
         TrajectoryActionBuilder builder = drive.actionBuilder(new Pose2d(-35, -63, Math.toRadians(90)))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-52, -52 , Math.toRadians(45)), -Math.PI)
                .stopAndAdd(
                        new SequentialAction(
                                new ParallelAction(
                                        robot.clawClose(),
                                        robot.armUp(),
                                        robot.setVertTarget(-2700)
                                ),
                                new SleepAction(2),
                                robot.clawOpen(),
                                robot.armWait(),
                                robot.setVertTarget(0)
                        )


                )
                .strafeToLinearHeading(new Vector2d(-54, -45), Math.toRadians(90))
//                 Add the Anonymous Action for the Vision.
                .stopAndAdd(
                        new SequentialAction(
                                determineBarnacle.detectTarget(),
                                new InstantAction(DetermineBarnacle::generateTargetTrajectoryLeft),
//                                    actionSchedular.run();
                                    DetermineBarnacle.getTargetSampleTrajectory()

                        )
                );

        return builder;
    }


}
