package org.firstinspires.ftc.teamcode.extraneous.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "1st java OpMode")
public class Test extends OpMode {
    DcMotor leftMotor;
    DcMotorEx rightMotor;

    @Override
    public void init() {
       leftMotor = hardwareMap.get(DcMotor.class, "leftbasemotor");
       rightMotor = hardwareMap.get(DcMotorEx.class, "rightbasemotor");
    }

    @Override
    public void loop() {
        leftMotor.setPower(-gamepad1.left_stick_y);
        rightMotor.setPower(gamepad1.right_stick_y);

    }
}
