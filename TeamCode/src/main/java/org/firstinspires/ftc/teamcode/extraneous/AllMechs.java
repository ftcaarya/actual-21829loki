package org.firstinspires.ftc.teamcode.extraneous;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.opmode.BlocksClassFilter;
import org.firstinspires.ftc.teamcode.extraneous.hardware.testing;
import org.firstinspires.ftc.teamcode.vision.EnhancedColorDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

public class AllMechs {
    public PIDController controller_right;

    public PIDController controller_extension;


    public MultipleTelemetry telemetry;

    public Servo claw, rotate, wrist_left, wrist_right, arm_left, arm_right, hold, pooper;


    public static final double POOPER_BLOCK = 1;
    public static final double POOPER_PASS = .4;

    public static final double CLAW_OPEN = 1;
    public static final double CLAW_CLOSE = 0.05;

    public static double wrist_left_down = 1;
    public static double wrist_left_up = 0;

    public static double wrist_right_down = 0;
    public static double wrist_right_up = 1;

    public static double arm_left_up = .85;
    public static double arm_left_down = 0.32;

    public static double arm_right_up = 0.15;
    public static double arm_right_down = 0.68;

    public static double arm_left_wait = 0.52;

    public static double arm_right_wait = 0.48;


    public static final double rotate_hor = 0.22;
    public static final double rotate_vert = 0.55;

    public static double intake_up = .3;
    public static double intake_down = .8;


    public static double p = 0.02, i = 0, d = 0.00065;
    public static double f = 0.01;


    public static int target = 0;
    public final double ticks_in_degree = 700/180.0;

    public static double pe = 0.04, ie = 0, de = 0.0007;

    public static int hor_target = 0;

    public DcMotorEx extension;

    public DcMotor frontLeft, rearLeft, rearRight, frontRight;

    public ColorSensor colorSensor;
    public Gamepad testGamepad;
    public IMU imu;

    public Gamepad gamepad1;
    public Gamepad gamepad2;

    public DcMotorEx vert_left, vert_right;
    public ElapsedTime vertTimer;

    public DcMotor intake;

    public VisionPortal visionPortal;
    public EnhancedColorDetectionProcessor colourMassDetectionProcessor;

    public OpenCvCamera camera;
    // 640, 360
    public static final int CAMERA_WIDTH = 1280, CAMERA_HEIGHT = 360;

    //change this later
    public static final double objectWidthRealWorld = 3.5;
    public static final double focalLength = 200 * 8.5 / 3.5;

    double cX = 0;
    double cY = 0;
    static double width = 0;

    public static double output;

    public static int vertTarget;
    public static int extTarget;



    public AllMechs(HardwareMap hardwareMap, int left, int right, Gamepad gamepad1, Gamepad gamepad2) {
        claw = hardwareMap.get(Servo.class, "claw");
        rotate = hardwareMap.get(Servo.class, "rotate");

        wrist_left = hardwareMap.get(Servo.class, "wrist left");
        wrist_right = hardwareMap.get(Servo.class, "wrist right");

        arm_left = hardwareMap.get(Servo.class, "arm left");
        arm_right = hardwareMap.get(Servo.class, "arm right");

        pooper = hardwareMap.get(Servo.class, "pooper");

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        hold = hardwareMap.get(Servo.class, "hold");

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        controller_extension = new PIDController(pe, ie, de);

        extension = hardwareMap.get(DcMotorEx.class, "extension");
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        frontRight = hardwareMap.get(DcMotor.class, "front right");
        rearRight = hardwareMap.get(DcMotor.class, "rear right");
        frontLeft = hardwareMap.get(DcMotor.class, "front left");
        rearLeft = hardwareMap.get(DcMotor.class, "rear left");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        vert_left = hardwareMap.get(DcMotorEx.class, "left elevator");
        vert_right = hardwareMap.get(DcMotorEx.class, "right elevator");

        vert_right.setDirection(DcMotorSimple.Direction.REVERSE);
        vert_left.setDirection(DcMotorSimple.Direction.REVERSE);

        vert_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vert_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        vert_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vert_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        controller_extension = new PIDController(pe, ie, de);

        extension = hardwareMap.get(DcMotorEx.class, "extension");


        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        colorSensor = hardwareMap.get(ColorSensor.class, "color sensor");

        vertTimer = new ElapsedTime();

//        colorSensor = hardwareMap.get(ColorSensor.class, "color sensor");

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        pooper = hardwareMap.get(Servo.class, "pooper");


//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()
//        );
//
//        camera = OpenCvCameraFactory.getInstance().createWebcam(
//                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId
//        );
        controller_right = new PIDController(p, i, d);

//        IMU imu = hardwareMap.get(IMU.class, "imu");
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
//
//        imu.initialize(parameters);

        double lowerH = 150; // the lower hsv threshold for your detection
        double upperH = 180; // the upper hsv threshold for your detection
        double minArea = 100; // the minimum area for the detection to consider for your prop

//        colourMassDetectionProcessor = new EnhancedColorDetectionProcessor(
//
//                () -> minArea,
//                () -> left, // the left dividing line, in this case the left third of the frame
//                () -> right, // the left dividing line, in this case the right third of the frame
//                EnhancedColorDetectionProcessor.StartPositions.SAMPLE
//        );

//        visionPortal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .build();
    }

    public Action setVertTarget(int target) {
        return new InstantAction(() -> vertTarget = target);
    }
    public Action transfer() {
        return new SequentialAction(
                new ParallelAction(
                        armWait(),
                        clawOpen()
                ),
                new SleepAction(0.5),
                setExtTarget(100),
                armDown(),
                clawClose(),
                new SleepAction(1),
                new ParallelAction(
                setVertTarget(-2700),
                armUp(),
                wristUp()
        )

        );
    }

    public Action updateVertPID() {
        return packet -> {
            controller_right.setPID(p, i, d);

            int rightPos = vert_right.getCurrentPosition();
            double pid_right = controller_right.calculate(rightPos, vertTarget);

            double ff = Math.cos(Math.toRadians(vertTarget / ticks_in_degree)) * f;

            double power_right = pid_right + ff;

            vert_left.setPower(power_right);
            vert_right.setPower(power_right);

            return true;
        };
    }
    public Action updateExtPID() {
        return packet -> {
            controller_extension.setPID(pe, ie, de);


            int Pos = extension.getCurrentPosition();
            double pid = controller_extension.calculate(Pos, (extTarget));

            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

            double power = pid + ff;


            extension.setPower(power);

            return true;
        };
    }
    public Action setExtTarget(int extarget) {
        return new InstantAction(() -> extTarget = extarget);
    }

    public Action intakeIn(){
        return new InstantAction(() -> intake.setPower(-0.5));
    }

    public Action intakeBack(){
        return new InstantAction(() -> intake.setPower(0.5));
    }


    public Action vertSample(){
        return new InstantAction(() -> vertTarget = 2700);
    }

    public Action vertDown() {
        return new InstantAction(() -> vertTarget = 50);
    }

    public Action setHorTarget(int target) {
        return new InstantAction(() -> hor_target = target);
    }

    public Action horExtend() {
        return new InstantAction(() -> hor_target = 350);
    }

    public Action horRetract() {
        return new InstantAction(() -> hor_target = 20);
    }


    public Action intakeDown() {

        return new InstantAction(() -> hold.setPosition(.75));


    }

    public Action intakeUp() {
        return new InstantAction(() -> hold.setPosition(.3));
    }

    public Action checkColorRed() {
        return p -> {
            hold.setPosition(.8);

            if (colorSensor.red() > colorSensor.green() + 50 && colorSensor.red() > colorSensor.blue() + 50) {
                pooper.setPosition(POOPER_BLOCK);
                gamepad1.rumbleBlips(1);
                gamepad1.setLedColor(255, 0, 0, 5000);
                intake.setPower(0);
                hold.setPosition(.3);
                setHorTarget(20);
                return false;
            } else if ((colorSensor.green() > colorSensor.blue()) && (colorSensor.red() > colorSensor.blue())) {
                pooper.setPosition(POOPER_BLOCK);
                gamepad1.setLedColor(230, 230, 0, 5000);
                gamepad1.rumbleBlips(1);
                intake.setPower(0);
                hold.setPosition(.3);
                setHorTarget(20);
                return false;
            } else if (colorSensor.blue() > colorSensor.green() + 50 && colorSensor.blue() > colorSensor.red() + 50) {
                pooper.setPosition(POOPER_PASS);
                gamepad1.setLedColor(0, 0, 225, 5000);
                gamepad1.rumbleBlips(1);
                intake.setPower(-.7);
                return true;
            } else if (gamepad1.square) {
                pooper.setPosition(POOPER_BLOCK);
                hold.setPosition(.3);
                intake.setPower(0);
                return false;
            } else {
                pooper.setPosition(POOPER_BLOCK);
                intake.setPower(-.65);
                return true;
            }
        };
    }

    public Action stopIntake() {
        return new ParallelAction(
                new InstantAction(() -> pooper.setPosition(POOPER_BLOCK)),
                new InstantAction(() -> intake.setPower(0))
        );
    }

    public Action armUp() {
        return new SequentialAction(
                new InstantAction(() -> arm_left.setPosition(arm_left_up)),
                new InstantAction(() -> arm_right.setPosition(arm_right_up))
        );
    }
    public Action armDown() {
        return new SequentialAction(
                new InstantAction(() -> arm_left.setPosition(arm_left_down)),
                new InstantAction(() -> arm_right.setPosition(arm_right_down))
        );
    }
    public Action wristUp() {
        return new ParallelAction(
                new InstantAction(() -> wrist_left.setPosition(wrist_left_up)),
                new InstantAction(() -> wrist_right.setPosition(wrist_right_up))
        );
    }
    public Action wristDown() {
        return new ParallelAction(
                new InstantAction(() -> wrist_left.setPosition(wrist_left_down)),
                new InstantAction(() -> wrist_right.setPosition(wrist_right_down))
        );
    }

    public Action clawClose() {
        return new InstantAction(() -> claw.setPosition(CLAW_CLOSE));

    }
    public Action clawOpen() {
        return new InstantAction(() -> claw.setPosition(CLAW_OPEN));

    }
    public Action rotateHor() {
        return new InstantAction(() -> rotate.setPosition(rotate_hor));

    }
    public Action rotateVert() {
        return new InstantAction(() -> rotate.setPosition(rotate_vert));

    }
    public Action armWait(){
        return new ParallelAction(
                new InstantAction(()-> arm_right.setPosition(arm_right_wait)),
                new InstantAction(()-> arm_left.setPosition(arm_left_wait))
        );
    }

}