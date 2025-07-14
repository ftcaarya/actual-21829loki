package org.firstinspires.ftc.teamcode.vision;//package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.extraneous.AllMechs.CAMERA_HEIGHT;
import static org.firstinspires.ftc.teamcode.extraneous.AllMechs.CAMERA_WIDTH;
import static org.firstinspires.ftc.teamcode.vision.OpenCVPipeline.width;
import static org.firstinspires.ftc.teamcode.vision.OpenCVPipeline.cX;
import static org.firstinspires.ftc.teamcode.vision.OpenCVPipeline.cY;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.extraneous.AllMechs;
import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "OpenCV Testing")
public class OpenCVOpMode extends LinearOpMode {
    AllMechs robot;
    private VisionPortal visionPortal;
    private EnhancedColorDetectionProcessor colourMassDetectionProcessor;

    public OpenCvCamera camera;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new AllMechs(hardwareMap, 200, 500, gamepad1, gamepad2);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()
        );

        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId
        );

        OpenCVPipeline YellowPipeline = new OpenCVPipeline();
//        ErodedOpenCVPipeline TestingPipeline = new ErodedOpenCVPipeline();
//        TejasGivenPipeline TejasPipeline = new TejasGivenPipeline();

        camera.setPipeline(YellowPipeline);
//        robot.camera.setPipeline(TestingPipeline);
//        robot.camera.setPipeline(TejasPipeline);

        camera.openCameraDevice();
        camera.startStreaming(1280, 360, OpenCvCameraRotation.UPRIGHT);

//        colourMassDetectionProcessor = new EnhancedColorDetectionProcessor(
//                () -> 100, // these are lambda methods, in case we want to change them while the match is running, for us to tune them or something
//                () -> 213, // the left dividing line, in this case the left third of the frame
//                () -> 426,
//                EnhancedColorDetectionProcessor.StartPositions.SPECIMEN// the left dividing line, in this case the right third of the frame
//
//        );

//        visionPortal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // the camera on your robot is named "Webcam 1" by default
//                .addProcessor(colourMassDetectionProcessor)
//                .build();



        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(camera, 30);

        waitForStart();

        while (opModeIsActive()) {
            FtcDashboard.getInstance().startCameraStream(camera, 60);

            camera.setPipeline(YellowPipeline);

            telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
//            telemetry.addData("Distance in Inch", (getDistance(width)));
            telemetry.addData("Width: ", width);

            telemetry.update();

        }

    }
}
