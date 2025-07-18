package org.firstinspires.ftc.teamcode.autonomous.specimen;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

        robot = new AllMechs(hardwareMap, 200, 400, gamepad1, gamepad2);
        determineBarnacle = new DetermineBarnacleSpec(1000, 200, 400, passPose, hardwareMap, gamepad1, gamepad2, drive, robot);

        telemetry.addData("Testing init: ", "running");
    }

    @Override
    public void start() {
        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                drive.actionBuilder(new Pose2d(10, -63, Math.toRadians(270)))
                                        .stopAndAdd(
                                                new ParallelAction(
                                                        // add the init methods claw close arm in score position
                                                )
                                        )
                                        .setTangent(Math.toRadians(110))
                                        .splineToConstantHeading(new Vector2d(0, -35), Math.toRadians(110))
                                        // score the actual specimen
                                        .setTangent(Math.toRadians(300))
                                        .splineToLinearHeading(new Pose2d(58, -45, Math.toRadians(90)), Math.toRadians(45))
                                        // now after vision processing it starts

                                        .stopAndAdd(
                                                new SequentialAction(
                                                        robot.setExtTarget(-225),
                                                        new SleepAction(1),
                                                        robot.checkColorRed()
                                                )
                                        )
                                        .setTangent(180)
                                        .splineToLinearHeading(new Pose2d(48, -45, Math.toRadians(-70)), Math.toRadians(180))
                                        .stopAndAdd(
                                                new SequentialAction(
                                                        // spit it out
                                                )
                                        )
                                        .turnTo(Math.toRadians(90))
                                        .stopAndAdd(
                                                new SequentialAction(
                                                        robot.setExtTarget(-150),
                                                        new SleepAction(1),
                                                        robot.checkColorRed()
                                                )
                                        )
                                        .turnTo(Math.toRadians(270))
                                        .stopAndAdd(
                                                new SequentialAction(
                                                        // spit it out thanks
                                                )
                                        )
                                        .setTangent(Math.toRadians(270))
                                        .splineToConstantHeading(new Vector2d(48, -63), Math.toRadians(270))
                                        .stopAndAdd(
                                                new SequentialAction(
                                                        // intake off the wall
                                                )
                                        )
                                        .setTangent(Math.toRadians(140))
                                        .splineToLinearHeading(new Pose2d(-5, -35, Math.toRadians(270)), Math.toRadians(100))
                                        .stopAndAdd(
                                                new SequentialAction(
                                                        // score it on the chamber
                                                )
                                        )
                                        .setTangent(Math.toRadians(320))
                                        .splineToConstantHeading(new Vector2d(40, -62), Math.toRadians(270))
                                        .stopAndAdd(
                                                new SequentialAction(
                                                        // intake off the wall
                                                )
                                        )
                                        .setTangent(Math.toRadians(320-180))
                                        .splineToConstantHeading(new Vector2d(3, -35), Math.toRadians(320-180))
                                        .stopAndAdd(
                                                new SequentialAction(
                                                        // score it on the chamber
                                                )
                                        )
                                        .setTangent(Math.toRadians(320))
                                        .splineToConstantHeading(new Vector2d(40, -62), Math.toRadians(270))
                                        .stopAndAdd(
                                                new SequentialAction(
                                                        // intake off the wall
                                                )
                                        )
                                        .setTangent(Math.toRadians(150))
                                        .splineToConstantHeading(new Vector2d(-3, -35), Math.toRadians(150))
                                        .stopAndAdd(
                                                new SequentialAction(
                                                        // score it on the chamber
                                                )
                                        )

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
