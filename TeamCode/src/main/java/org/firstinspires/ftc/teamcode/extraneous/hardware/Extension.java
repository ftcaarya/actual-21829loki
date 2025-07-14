package org.firstinspires.ftc.teamcode.extraneous.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "PID Tuning for extension")
public class Extension extends OpMode {
    private PIDController controller_extension;



    public static double p = 0.05, i = 0, d = 0;

    public static double pe = 0, ie = 0, de = 0;
    public static double f = 0;


    public static int target = 0;

    private DcMotorEx extension;





    @Override
    public void init () {
        controller_extension = new PIDController(pe, ie, de);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        extension = hardwareMap.get(DcMotorEx.class, "extension");
        extension.setDirection(DcMotorSimple.Direction.REVERSE);


        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



    }

    @Override
    public void loop () {
        controller_extension.setPID(pe, ie, de);


        int Pos = extension.getCurrentPosition();
        double power = controller_extension.calculate(Pos, (target));




        extension.setPower(power);

        telemetry.addData("Extension Pos", Pos);
        telemetry.addData("Target", target);

        telemetry.update();

    }

}
