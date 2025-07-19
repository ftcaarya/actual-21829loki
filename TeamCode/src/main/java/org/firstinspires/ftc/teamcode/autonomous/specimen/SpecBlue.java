package org.firstinspires.ftc.teamcode.autonomous.specimen;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.extraneous.AllMechs;
import org.firstinspires.ftc.teamcode.extraneous.determineBarnacle.DetermineBarnacleBlueSpec;
import org.firstinspires.ftc.teamcode.extraneous.determineBarnacle.DetermineBarnacleSpec;
import org.firstinspires.ftc.teamcode.vision.ColourMassDetectionProcessor;
import org.firstinspires.ftc.teamcode.vision.ColourMassDetectionProcessorSpec;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "specimen blue")
public class SpecBlue extends OpMode {
    DetermineBarnacleBlueSpec determineBarnacle;
    PinpointDrive drive;
    private final FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    MultipleTelemetry mTelemetry;
    AllMechs robot;


    @Override
    public void init() {
        drive = new PinpointDrive(hardwareMap, new Pose2d(10, -63, Math.toRadians(90)));

        Pose2d passPose = new Pose2d(-73, -45, Math.toRadians(90));

        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new AllMechs(hardwareMap, 600, 600, gamepad1, gamepad2);

        determineBarnacle = new DetermineBarnacleBlueSpec(700, 600, 600, passPose, hardwareMap, gamepad1, gamepad2, drive, robot);

        telemetry.addData("Testing Init:", "running");
    }

    @Override
    public void start() {
        runningActions.add(
                new ParallelAction(
                        new SequentialAction(
                                new ParallelAction(
                                        robot.intakeUp(),
                                        robot.setExtTarget(100),
                                        new SequentialAction(
                                                robot.putSpec(),
                                                new SleepAction(1)
                                        )
                                ),
                                drive.actionBuilder(new Pose2d(6, -63, Math.toRadians(270)))
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
                                        .stopAndAdd(
                                                new SequentialAction(
                                                        determineBarnacle.detectTarget(),
                                                        new InstantAction(() -> {
                                                            ColourMassDetectionProcessorSpec.PropPositions detected = DetermineBarnacleBlueSpec.getRecordedPosition();

                                                            telemetry.addData("=== DETECTION PHASE ===", "");
                                                            telemetry.addData("Detected Position", detected != null ? detected.toString() : "NULL");
                                                            telemetry.update();

                                                            telemetry.addData("=== GENERATION PHASE ===", "");
                                                            DetermineBarnacleBlueSpec.generateTargetTrajectory();

                                                            boolean hasTrajectory = DetermineBarnacleBlueSpec.hasTrajectory();
                                                            telemetry.addData("Trajectory Generated", hasTrajectory ? "YES" : "NO");

                                                            if (hasTrajectory) {
                                                                Action traj = DetermineBarnacleBlueSpec.getTargetSampleTrajectory();
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


                                        .build()
                        ),
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
