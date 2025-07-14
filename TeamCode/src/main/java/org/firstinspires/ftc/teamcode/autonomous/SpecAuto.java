package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.extraneous.ActionSchedular;
import org.firstinspires.ftc.teamcode.extraneous.DetermineBarnacle;

@Autonomous(name = "specimen auto")
public class SpecAuto extends MasterAuto {
    ActionSchedular actionSchedular;

    SpecAuto() {
        actionSchedular = new ActionSchedular();
    }

    @Override
    protected Action onRun() {
        TrajectoryActionBuilder builder = drive.actionBuilder(new Pose2d(10, -63, Math.toRadians(270)));

        builder = wholeSequence(builder);

        return builder.build();
    }

    private TrajectoryActionBuilder wholeSequence(TrajectoryActionBuilder builder) {
        builder = builder
                .setTangent(Math.toRadians(110))
                .splineToConstantHeading(new Vector2d(0, -35), Math.toRadians(110))
                // score the specimen that is with the robot
                .setTangent(Math.toRadians(300))
                .splineToLinearHeading(new Pose2d(55, -45, Math.toRadians(90)), Math.toRadians(45))
                // Add the Anonymous Action for the Vision.
                .stopAndAdd(
                        new SequentialAction(
                                new InstantAction(DetermineBarnacle::generateTargetTrajectoryRight),
                                telemetryPacket -> {
                                    actionSchedular.run();
                                    return  DetermineBarnacle.getTargetSampleTrajectory().run(telemetryPacket);
                                }
                        )

                );
        return builder;
    }
}
