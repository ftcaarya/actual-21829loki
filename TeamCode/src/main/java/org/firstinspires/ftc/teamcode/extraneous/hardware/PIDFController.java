package org.firstinspires.ftc.teamcode.extraneous.hardware;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class PIDFController extends LinearOpMode {

    public DcMotorEx vert_left, vert_right;

    double integralSum = 0;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;
    double Kf = 0;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    int target = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        vert_left = hardwareMap.get(DcMotorEx.class, "vert left");
        vert_right = hardwareMap.get(DcMotorEx.class, "vert right");

        vert_right.setDirection(DcMotorSimple.Direction.FORWARD);
        vert_left.setDirection(DcMotorSimple.Direction.REVERSE);

        vert_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vert_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        vert_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vert_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while (opModeIsActive()) {
            double power = PIDFControl(target, vert_left.getVelocity());
            vert_left.setPower(power);
            vert_right.setPower(power);
        }
    }

    public double PIDFControl(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        return (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (reference * Kf);
    }
}
