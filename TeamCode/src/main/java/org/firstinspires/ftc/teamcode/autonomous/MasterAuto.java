package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.extraneous.AllMechs;
import org.firstinspires.ftc.teamcode.extraneous.DetermineBarnacle;

public abstract class MasterAuto extends LinearOpMode {
    MecanumDrive drive;
    MultipleTelemetry mTelemetry;
    AllMechs robot;

    DetermineBarnacle determineBarnacle;


    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-35, -63, Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap, initialPose);

        Pose2d passPose = new Pose2d(-54, -45, Math.toRadians(90));

        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new AllMechs(hardwareMap, 200, 400, gamepad1, gamepad2);

        determineBarnacle = new DetermineBarnacle(1000, 100, 200, passPose, hardwareMap, gamepad1, gamepad2);

//        onInit();

        telemetry.addData("Testing Init:", "running");

        Action action = onRun();

        waitForStart();

        resetRuntime();

            Actions.runBlocking(
                    new ParallelAction(
                            action,
                            robot.updateExtPID(),
                            robot.updateVertPID()
                    )
            );


    }

    protected abstract Action onRun();
//    protected void onInit() {};
}
