package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Boolean.FALSE;

import android.app.Notification;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.extraneous.ActionSchedular;
import org.firstinspires.ftc.teamcode.extraneous.AllMechs;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="TeleOp Testing")
public class TeleOpTesting extends OpMode {
    AllMechs robot;
    ActionSchedular actionSchedular;
    MecanumDrive drive;
    Gamepad currentGamepad1;
    Gamepad previousGamepad1;

    boolean horToggle = false;
    boolean intakeInToggle = false;
    boolean intakeOutToggle = false;


    private final FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    @Override
    public void init() {
        robot = new AllMechs(hardwareMap, 0, 0, gamepad1, gamepad2);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        actionSchedular = new ActionSchedular();
        previousGamepad1 = new Gamepad();
        currentGamepad1 = new Gamepad();

    }

    @Override
    public void start() {
        runningActions.add(
                new ParallelAction(
                        robot.updateVertPID(),
                        robot.updateExtPID(),
                        robot.rotateHor()
                )

        );


    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();


        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        .484038 * Math.tan(1.12 * -gamepad1.left_stick_y),
                        .484038 * Math.tan(1.12 * -gamepad1.left_stick_x)
                ),
                -gamepad1.right_stick_x
        ));
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

//        if (gamepad1.dpad_up) {
//            runningActions.add(
//                    new ParallelAction(
//                            robot.setVertTarget(2790),
//                            robot.armUp(),
//                            robot.wristUp()
//                    )
//            );
//        }
        if (gamepad2.dpad_up) {
            runningActions.add(
                    robot.transfer()
                    );
        }

        if (gamepad2.dpad_down) {
            runningActions.add(
                    new ParallelAction(
                            robot.setVertTarget(0),
                            robot.armWait(),
                            robot.wristDown(),
                            robot.rotateHor()
                    )
            );
        }


        if (gamepad2.dpad_right) {
            runningActions.add(

                    robot.setExtTarget(-400)
            );
            horToggle = true;
        }

        if (gamepad2.dpad_left && !horToggle) {
            runningActions.add(
                    new ParallelAction(
                            robot.setExtTarget(-20),
                            robot.intakeUp(),
                            robot.stopIntake()
                    )

            );
        }

        if (gamepad2.dpad_left && horToggle) {

            runningActions.add(
                    new ParallelAction(
                    robot.setExtTarget(100),
                    robot.intakeUp(),
                    robot.stopIntake()
                    )
            );
        }
        if (gamepad2.left_bumper) {
            runningActions.add(

                    robot.clawOpen()
            );
        }
        if (gamepad2.right_bumper) {
            runningActions.add(
                    robot.clawClose()
            );
        }


        if (gamepad2.right_stick_button) {
            runningActions.add(new ParallelAction(
                    robot.intakeDown(),
                    robot.checkColorRed(),

                    robot.armWait()
            ));
        }


        if (gamepad2.circle) {
            runningActions.add(

                    new ParallelAction(
                            robot.intakeUp(),
                            robot.stopIntake()
                    )
            );
        }

        if (gamepad2.left_stick_button) {
            runningActions.add(
                    robot.armDown()
            );
        }
        if(gamepad2.cross) {
            runningActions.add(
                    robot.intakeIn()
            );
        }
        if(gamepad2.triangle){
            runningActions.add(
                    robot.intakeBack()
            );
        }
        if (gamepad2.left_trigger>0.2){
            runningActions.add(
                    robot.getSpec()
            );

        }
        if (gamepad2.right_trigger>0.2){
            runningActions.add(
                    robot.putSpec()
            );

        }
        if (gamepad2.square) {
            runningActions.add(
                    robot.stopIntake()
            );
        }
//






        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        dash.sendTelemetryPacket(packet);
    }
}
