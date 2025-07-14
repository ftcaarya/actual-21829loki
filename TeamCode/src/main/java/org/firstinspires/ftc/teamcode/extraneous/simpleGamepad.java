package org.firstinspires.ftc.teamcode.extraneous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Testing Motor Configs", group = "exercise")
public class simpleGamepad extends OpMode {
    private final int speedCap = 1;

    public DcMotor frontLeft, rearLeft, rearRight, frontRight;


    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "front left");
        rearLeft = hardwareMap.get(DcMotor.class, "rear left");
        rearRight = hardwareMap.get(DcMotor.class, "rear right");
        frontRight = hardwareMap.get(DcMotor.class, "front right");

        // Change this
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop() {


        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed22
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double frontLeftPower = (y + x + rx);
        double backLeftPower = (y - x + rx);
        double frontRightPower = (y - x - rx);
        double backRightPower = (y + x - rx);

        frontLeftPower = Range.clip(frontLeftPower, -speedCap, speedCap);
        backRightPower = Range.clip(backRightPower, -speedCap, speedCap);
        backLeftPower = Range.clip(backLeftPower, -speedCap, speedCap);
        frontRightPower = Range.clip(frontRightPower, -speedCap, speedCap);

        frontLeft.setPower(frontLeftPower);
        rearLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        rearRight.setPower(backRightPower);

        telemetry.update();
    }
}
