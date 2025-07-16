package org.firstinspires.ftc.teamcode.extraneous.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="Hardware testing", group = "exercises")
public class HardwareTesting extends OpMode {



    public Servo claw, rotate, wrist_left, wrist_right, arm_left, arm_right, hor_left, hor_right, intake_left, intake_right, pooper;

    public DcMotor intake;

    public DcMotor frontRight, rearRight, frontLeft, rearLeft;
    public DcMotorEx rightVert, leftVert;

    public DcMotor test;

    public ColorSensor colorSensor;

    public static double hor_left_extend = 0.6;
    public static double hor_left_retract = 0.32;

    public static double hor_right_extend = 0.5;
    public static double hor_right_retract = .7;

    public static final double POOPER_BLOCK = 1;
    public static final double POOPER_PASS = .4;

    public static final double CLAW_OPEN = 1;
    public static final double CLAW_CLOSE = 0.25;

    public static double wrist_left_down = 1;
    public static double wrist_left_up = 0;

    public static double wrist_right_down = 0;
    public static double wrist_right_up = 1;
    public static double intake_left_down = 0;
    public static double intake_left_up = 1;

    public static double intake_right_down = 0;
    public static double intake_right_up = 1;

    public static double arm_left_up = .59;
    public static double arm_left_down = 0.32;

    public static double arm_right_up = 0.2;
    public static double arm_right_down = 0.68
            ;

    public static final double rotate_hor = 0.22;
    public static final double rotate_vert = 0.55;

    public static double intake_up = 1;



@Override
    public void init() {

//        claw = hardwareMap.get(Servo.class, "claw");
//        rotate = hardwareMap.get(Servo.class, "rotate");
//
        wrist_left = hardwareMap.get(Servo.class, "wrist left");
        wrist_right = hardwareMap.get(Servo.class, "wrist right");

        arm_left = hardwareMap.get(Servo.class, "arm left");
        arm_right = hardwareMap.get(Servo.class, "arm right");

//        hor_left = hardwareMap.get(Servo.class, "hor left");
//        hor_right = hardwareMap.get(Servo.class, "hor right");

//        pooper = hardwareMap.get(Servo.class, "pooper");
//
//        intake = hardwareMap.get(DcMotor.class, "intake");
//
//        frontRight = hardwareMap.get(DcMotor.class, "front right");
//        rearRight = hardwareMap.get(DcMotor.class, "rear right");
//        frontLeft = hardwareMap.get(DcMotor.class, "front left");
//        rearLeft = hardwareMap.get(DcMotor.class, "rear left");
//        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        leftVert = hardwareMap.get(DcMotorEx.class, "left elevator");
//        leftVert.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightVert = hardwareMap.get(DcMotorEx.class, "right elevator");
//        rightVert.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        test = hardwareMap.get(DcMotor.class, "test");



//    colorSensor = hardwareMap.get(ColorSensor.class, "color sensor");
//
//    intake = hardwareMap.get(DcMotor.class, "intake motor");
//    intake.setDirection(DcMotorSimple.Direction.FORWARD);


}
@Override
public void loop() {

    if (gamepad1.dpad_up) {
        arm_left.setPosition(arm_left_up);
        arm_right.setPosition(arm_right_up);
    }

    if (gamepad1.dpad_down) {
        arm_left.setPosition(arm_left_down);
        arm_right.setPosition(arm_right_down);
    }
    if (gamepad1.dpad_right) {
        wrist_left.setPosition(wrist_left_up);
        wrist_right.setPosition(wrist_right_up);
    }
    if (gamepad1.dpad_left) {
        wrist_left.setPosition(wrist_left_down);
        wrist_right.setPosition(wrist_right_down);
    }

}

}

