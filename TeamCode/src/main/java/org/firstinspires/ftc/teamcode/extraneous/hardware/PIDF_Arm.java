package org.firstinspires.ftc.teamcode.extraneous.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name = "PID Tuning for elevators")
public class PIDF_Arm extends OpMode {
    private PIDController controller_left;
    private PIDController controller_right;

    public static double p = 0.02, i = 0, d = 0.00065;
    public static double f = 0.05;

    public static int target = 0;

    private final double ticks_in_degree = 700/180.0;
    private DcMotorEx elevator_left;
    private DcMotorEx elevator_right;




    @Override
    public void init () {
        controller_left = new PIDController(p, i, d);
        controller_right = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        elevator_left = hardwareMap.get(DcMotorEx.class, "left elevator");
        elevator_right = hardwareMap.get(DcMotorEx.class, "right elevator");

        elevator_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



    }

    @Override
    public void loop () {
        controller_right.setPID(p, i, d);

//        int leftPos = elevator_left.getCurrentPosition();
        int rightPos = elevator_right.getCurrentPosition();
//        double pid_left = controller_left.calculate(leftPos, target);
        double pid_right = controller_right.calculate(rightPos, (-1 * target));

        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

//        double power_left = pid_left + ff;
        double power_right = pid_right + ff;

        elevator_left.setPower(power_right);
        elevator_right.setPower(power_right);

        telemetry.addData("Right Pos", rightPos);
//        telemetry.addData("Left Pos", leftPos);
        telemetry.addData("Target", target);
//        telemetry.addData("Left Power", power_left);
        telemetry.addData("Right Power", power_right);
        telemetry.update();



    }

}
