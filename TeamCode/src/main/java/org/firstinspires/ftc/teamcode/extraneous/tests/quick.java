package org.firstinspires.ftc.teamcode.extraneous.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Linakge Testing")
public class quick extends OpMode {

    public Servo linkage;
    @Override
    public void init() {
        linkage = hardwareMap.get(Servo.class, "linkage");
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_left) {
            linkage.setPosition(0.2);
        } else if (gamepad1.dpad_right) {
            linkage.setPosition(0.415);
        }
    }
}
