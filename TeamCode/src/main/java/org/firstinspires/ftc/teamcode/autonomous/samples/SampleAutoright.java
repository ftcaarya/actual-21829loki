package org.firstinspires.ftc.teamcode.autonomous.samples;

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
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.extraneous.ActionSchedular;
import org.firstinspires.ftc.teamcode.extraneous.AllMechs;
import org.firstinspires.ftc.teamcode.extraneous.determineBarnacle.DetermineBarnacleSample;

@Autonomous(name = "Sample Side Auto right", group = "robot")
public class SampleAutoright extends OpMode {
    ActionSchedular actionSchedular;
    DetermineBarnacleSample determineBarnacle;
    PinpointDrive drive;
    MultipleTelemetry mTelemetry;
    AllMechs robot;
    Action builder;
    static int xPos = -28;
    static int yPos = -9;

    Gamepad currentGamepad1;
    Gamepad previousGamepad1;

    @Override
    public void init() {
        actionSchedular = new ActionSchedular();

        Pose2d startPose = new Pose2d(-35, -63, Math.toRadians(90));
        Pose2d initialPose = new Pose2d(-35, -63, Math.toRadians(90));
        drive = new PinpointDrive(hardwareMap, new Pose2d(-35, -63, Math.toRadians(90)));

        Pose2d passPose = new Pose2d(-54, -45, Math.toRadians(90));

        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new AllMechs(hardwareMap, 200, 400, gamepad1, gamepad2);

        determineBarnacle = new DetermineBarnacleSample(1000, 100, 200, passPose, hardwareMap, gamepad1, gamepad2, drive, robot);

//        onInit();

        telemetry.addData("Testing Init:", "running");

        builder = drive.actionBuilder(new Pose2d(-35, -63, Math.toRadians(90)))
//                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-52, -52 , Math.toRadians(45)), -Math.PI)
                .build();

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();
    }

    @Override
    public void start() {

        TrajectoryActionBuilder builder = drive.actionBuilder(new Pose2d(-35, -63, Math.toRadians(90)));
//        Action action = wholeSequence().build();

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                drive.actionBuilder(new Pose2d(-35, -63, Math.toRadians(90)))
                                        .stopAndAdd( new ParallelAction(
                                                        new InstantAction(() -> robot.hold.setPosition(.3) ),
                                                        robot.rotateHor(),
                                                        robot.clawClose()
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
                                                        new InstantAction(()-> robot.hold.setPosition(0.7))

                                                )
                                        )
                                        .strafeToLinearHeading(new Vector2d(-52, -43), Math.toRadians(90))
                                .turnTo(Math.toRadians(108))
                                // intake sample
                                .stopAndAdd(
                                        new SequentialAction(
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
                                                new InstantAction(()-> robot.hold.setPosition(0.7))
                                        )

                                )
                                .setTangent((Math.PI - Math.atan((18/14.5))))
                                .splineToLinearHeading(new Pose2d(-52, -36, Math.toRadians(138)), (Math.PI - Math.atan((18/14.5))))
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
                                new InstantAction(DetermineBarnacleSample::generateTargetTrajectoryLeft),
//                                    actionSchedular.run();
                                DetermineBarnacleSample.getTargetSampleTrajectory()

                        )
                );

        return builder;
    }


}
