package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.extraneous.ActionSchedular;
import org.firstinspires.ftc.teamcode.extraneous.AllMechs;
import org.firstinspires.ftc.teamcode.extraneous.DetermineBarnacle;

@Autonomous(name = "Sample Side Auto middle", group = "robot")
public class SampleAutomiddle extends OpMode {
    ActionSchedular actionSchedular;
    DetermineBarnacle determineBarnacle;
    PinpointDrive drive;
//    Pose2d startPose = new Pose2d(-35, -63, Math.toRadians(90));
    Pose2d passPose = new Pose2d(-54, -45, Math.toRadians(90));
    MultipleTelemetry mTelemetry;
    AllMechs robot;
    Action builder;

    @Override
    public void init() {
        actionSchedular = new ActionSchedular();

        Pose2d startPose = new Pose2d(-35, -63, Math.toRadians(90));
        Pose2d initialPose = new Pose2d(-35, -63, Math.toRadians(90));
        drive = new PinpointDrive(hardwareMap, new Pose2d(-35, -63, Math.toRadians(90)));

        Pose2d passPose = new Pose2d(-54, -45, Math.toRadians(90));

        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new AllMechs(hardwareMap, 200, 400, gamepad1, gamepad2);

        determineBarnacle = new DetermineBarnacle(1000, 100, 200, passPose, hardwareMap, gamepad1, gamepad2);

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

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(drive.actionBuilder(new Pose2d(-35, -63, Math.toRadians(90)))
                                .stopAndAdd(
                                        new InstantAction(() -> robot.hold.setPosition(.75))

                                )
                                .setReversed(false)
                                .stopAndAdd(
                                        new SequentialAction(
                                                robot.setVertTarget(-2700),
                                                new ParallelAction(
                                                        robot.clawClose(),
                                                        robot.armUp(),
                                                        robot.wristUp()
                                                )
                                        )
                                )

                                .splineToLinearHeading(new Pose2d(-60, -62 , Math.toRadians(45)), -Math.PI)
                                .stopAndAdd(
                                        new SequentialAction(
                                                new SleepAction(1),
                                                robot.clawOpen(),
                                                robot.armWait(),
                                                robot.wristDown(),
                                                robot.setVertTarget(0)
                                        )
                                )
                                .strafeToLinearHeading(new Vector2d(-54, -45), Math.toRadians(90))
                                .turnTo(Math.toRadians(68))
                        .stopAndAdd(
                                new ParallelAction(
                                        robot.setExtTarget(-200),
                                        robot.checkColorRed()


                                )
                        )

                                .stopAndAdd(
                                        new SequentialAction(

                                                robot.intakeIn(),
                                                new SleepAction(1),
                                                robot.stopIntake()


                                        )
                                )
                                .setTangent(0)

                        .stopAndAdd(
                                new ParallelAction(
                                        new InstantAction(() -> robot.hold.setPosition(.75)),
                                        robot.setExtTarget(100)
                                )
                        )
                                .splineToLinearHeading(new Pose2d(-60, -60, Math.toRadians(45)), -Math.PI)
                                .stopAndAdd(
                                        new SequentialAction(
                                                robot.armDown(),
                                                robot.clawClose(),
                                                new SleepAction(1.5),
                                                robot.setVertTarget(-2700),
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
                                // add the deposit action for the sample it holds
                                .setTangent(0)
                                .splineToLinearHeading(new Pose2d(-58, -44, Math.toRadians(95)), -Math.toRadians(180))
                                // add the intake for the middle sample
                                .stopAndAdd(
                                        new ParallelAction(
                                                robot.setExtTarget(-200),
                                                robot.checkColorRed()
                                        )
                                )
                                .setTangent(0)

                                .stopAndAdd(
                                        new ParallelAction(
                                                new InstantAction(() -> robot.hold.setPosition(.75)),
                                                robot.setExtTarget(100)
                                        )
                                )
                                .splineToLinearHeading(new Pose2d(-60, -60, Math.toRadians(45)), -Math.PI)
                                .stopAndAdd(
                                        new SequentialAction(
                                                robot.armDown(),
                                                robot.clawClose(),
                                                robot.setVertTarget(-2790),
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

                                .build()),
                        robot.updateExtPID(),
                        robot.updateVertPID()
                )
        );
    }

    @Override
    public void loop() {

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
                                        robot.setVertTarget(2790)
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
