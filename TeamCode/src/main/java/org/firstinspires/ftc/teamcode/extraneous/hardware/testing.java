package org.firstinspires.ftc.teamcode.extraneous.hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name = "intake testing")
public class testing extends OpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    private ColorSensor colorSensor;
    private DcMotor intake;
    private Servo pooper;
    private Servo hold;

    public static final double POOPER_BLOCK = 1;
    public static final double POOPER_PASS = .4;

    public static double INTAKE_DOWN = 0.7;
    public static double INTAKE_UP = 0;
    public static double pos = 0.7;


    @Override
    public void init() {
        colorSensor = hardwareMap.get(ColorSensor.class, "color sensor");

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        pooper = hardwareMap.get(Servo.class, "pooper");
        hold = hardwareMap.get(Servo.class, "hold");
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        telemetry.addData("Red: ", colorSensor.red());
        telemetry.addData("Green: ", colorSensor.green());
        telemetry.addData("Blue: ", colorSensor.blue());
        telemetry.update();

        runningActions.add(
                new ParallelAction(
                        checkColorRed(),
                        setHold(pos)
                )
        );


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

    public Action setHold(double pos) {
        return new InstantAction(() -> hold.setPosition(pos));
    }

    public class CheckColorRed implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (colorSensor.red() > colorSensor.green() + 50 && colorSensor.red() > colorSensor.blue() + 50) {
                pooper.setPosition(POOPER_BLOCK);
                intake.setPower(0);
                return false;
            } else if ((colorSensor.green() > colorSensor.blue()) && (colorSensor.red() > colorSensor.blue())) {
                pooper.setPosition(POOPER_BLOCK);
                intake.setPower(0);
                return false;
            } else if (colorSensor.blue() > colorSensor.green() + 50 && colorSensor.blue() > colorSensor.red() + 50) {
                pooper.setPosition(POOPER_PASS);
                intake.setPower(.65);
                return true;
            } else {
                pooper.setPosition(POOPER_BLOCK);
                intake.setPower(.6);
                return true;
            }

        }
    }

    public Action checkColorRed() {
        return new CheckColorRed();
    }
}
