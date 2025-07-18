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
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.extraneous.ActionSchedular;
import org.firstinspires.ftc.teamcode.extraneous.AllMechs;
import org.firstinspires.ftc.teamcode.extraneous.DetermineBarnacle;
import org.opencv.core.Mat;

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
    static int xPos = -28;
    static int yPos = -9;

    Gamepad currentGamepad1;
    Gamepad previousGamepad1;

    @Override
    public void init() {
//        actionSchedular = new ActionSchedular();

        Pose2d startPose = new Pose2d(-35, -63, Math.toRadians(90));
        Pose2d initialPose = new Pose2d(-35, -63, Math.toRadians(90));
        drive = new PinpointDrive(hardwareMap, new Pose2d(-35, -63, Math.toRadians(90)));

        Pose2d passPose = new Pose2d(-54, -45, Math.toRadians(90));

        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new AllMechs(hardwareMap, 200, 400, gamepad1, gamepad2);

        determineBarnacle = new DetermineBarnacle(1000, 100, 200, passPose, hardwareMap, gamepad1, gamepad2, drive, robot);

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
    public void init_loop() {
        previousGamepad1.copy(currentGamepad1);

        telemetry.addData("x pos: ", xPos);
        telemetry.addData("y pos: ", yPos);

        if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
            xPos -= 1;
        }

        if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
            xPos += 1;
        }

        if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
            yPos += 1;
        }

        if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
            yPos -= 1;
        }

        telemetry.update();
        currentGamepad1.copy(gamepad1);
    }

    @Override
    public void start() {

        TrajectoryActionBuilder builder = drive.actionBuilder(new Pose2d(-35, -63, Math.toRadians(90)));
//        Action action = wholeSequence().build();

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(drive.actionBuilder(new Pose2d(-35, -63, Math.toRadians(90)))
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
                                                new InstantAction(()-> robot.hold.setPosition(0.8))

                                        )
                                )
                                .strafeToLinearHeading(new Vector2d(-52, -43), Math.toRadians(90))
                                .turnTo(Math.toRadians(79))
                        .stopAndAdd(
                                new SequentialAction(
                                        robot.setExtTarget(-250),
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
                                .setTangent(Math.toRadians(90))
                                // add the deposit action for the sample it holds
                                .splineToLinearHeading(new Pose2d(-59, -40, Math.toRadians(90)), Math.toRadians(90))
                                // add the intake for the middle sample
                                .stopAndAdd(
                                        new SequentialAction(
                                                new SleepAction(.5),
                                                robot.setExtTarget(-150),
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
                                .splineToLinearHeading(new Pose2d(xPos, yPos, Math.toRadians(0)), Math.toRadians(0))
                                .stopAndAdd(
                                        new SequentialAction(
                                                new ParallelAction(
                                                        robot.setExtTarget(-180),
                                                        robot.intakeUp()
                                                ),
                                                new SleepAction(.5),
                                                robot.subIntakeCheck(),
                                                robot.setExtTarget(100)
                                        )


                                )
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
