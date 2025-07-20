package org.firstinspires.ftc.teamcode.autonomous.specimen;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.extraneous.AllMechs;
import org.firstinspires.ftc.teamcode.extraneous.determineBarnacle.DetermineBarnacleSpec;

@Autonomous(name = "Spec side right")
public class SpecAutoright extends OpMode {
    DetermineBarnacleSpec determineBarnacle;
    PinpointDrive drive;
    MultipleTelemetry mTelemetry;
    AllMechs robot;

    @Override
    public void init() {
        drive = new PinpointDrive(hardwareMap, new Pose2d(10, -63, Math.toRadians(270)));

        Pose2d passPose = new Pose2d(58, -45, Math.toRadians(90));

        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new AllMechs(hardwareMap, 200, 800, gamepad1, gamepad2);
        determineBarnacle = new DetermineBarnacleSpec(1000, 200, 800, passPose, hardwareMap, gamepad1, gamepad2, drive, robot);

        telemetry.addData("Testing init: ", "running");
    }

    @Override
    public void start() {
        Actions.runBlocking(
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
                                        // now after vision processing it starts
//
                                        .stopAndAdd(
                                                new SequentialAction(
                                                        new InstantAction(() -> robot.hold.setPosition(.78)),
                                                        robot.setExtTarget(-200),
                                                        new SleepAction(1),
                                                        robot.checkColorRed()
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
                                                        robot.setExtTarget(-200),
                                                        new SleepAction(1),
                                                        robot.checkColorRed()
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
                                                                robot.getSpec()
                                                        )
                                                )
                                        )
                                        .setTangent(Math.toRadians(270))
                                        .splineToConstantHeading(new Vector2d(48, -62), Math.toRadians(270))
//                                        .stopAndAdd(
//                                                new SequentialAction(
//
//                                                        new SleepAction(0.5),
//                                                        robot.clawClose(),
//                                                        new SleepAction(0.5),
//                                                        robot.putSpec()
//                                                )
//                                        )
//                                        .setTangent(Math.toRadians(140))
//                                        .splineToLinearHeading(new Pose2d(-5, -32, Math.toRadians(270)), Math.toRadians(100))
//                                        .stopAndAdd(
//                                                new SequentialAction(
//                                                        new SleepAction(0.5),
//                                                        robot.clawOpen(),
//                                                        robot.getSpec(),
//                                                        robot.rotateHor()
//                                                )
//
//                                        )
//                                        .setTangent(Math.toRadians(320))
//                                        .splineToConstantHeading(new Vector2d(40, -62), Math.toRadians(270))
//                                        .stopAndAdd(
//                                                new SequentialAction(
//                                                        new SequentialAction(
//                                                                robot.getSpec(),
//                                                                new SleepAction(0.5),
//                                                                robot.clawClose(),
//                                                                new SleepAction(0.5),
//                                                                robot.putSpec()
//                                                        )
//                                                )
//                                        )
//                                        .setTangent(Math.toRadians(320-180))
//                                        .splineToConstantHeading(new Vector2d(3, -35), Math.toRadians(320-180))
//                                        .stopAndAdd(
//                                                new SequentialAction(
//                                                        // score it on the chamber
//                                                )
//                                        )
//                                        .setTangent(Math.toRadians(320))
//                                        .splineToConstantHeading(new Vector2d(40, -62), Math.toRadians(270))
//                                        .stopAndAdd(
//                                                new SequentialAction(
//                                                        new SequentialAction(
//                                                                robot.getSpec(),
//                                                                new SleepAction(0.5),
//                                                                robot.clawClose(),
//                                                                new SleepAction(0.5),
//                                                                robot.putSpec()
//                                                        )
//                                                )
//                                        )
//                                        .setTangent(Math.toRadians(150))
//                                        .splineToConstantHeading(new Vector2d(-3, -35), Math.toRadians(150))
//                                        .stopAndAdd(
//                                                new SequentialAction(
//                                                        // score it on the chamber
//                                                )
//                                        )

                                        .build()
                        ),
                        robot.updateVertPID(),
                        robot.updateExtPID()
                )
        );
    }

    @Override
    public void loop() {

    }
}
