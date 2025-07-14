package org.firstinspires.ftc.teamcode.extraneous.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "lift testing")
public class MotorTesting extends OpMode {
    public DcMotor leftVert, rightVert;

    @Override
    public void init() {
        leftVert = hardwareMap.get(DcMotor.class, "left elevator");
        rightVert = hardwareMap.get(DcMotor.class, "right elevator");
        leftVert.setDirection(DcMotorSimple.Direction.REVERSE);
        rightVert.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        while (gamepad1.cross) {
            leftVert.setPower(1);
            rightVert.setPower(1);
        }
    }
}
