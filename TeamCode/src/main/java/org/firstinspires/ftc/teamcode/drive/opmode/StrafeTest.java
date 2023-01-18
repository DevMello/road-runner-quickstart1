package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.SleeveDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class StrafeTest extends LinearOpMode {
    public static double DISTANCE = 60; // in
    protected static DcMotorEx railRight;
    protected static DcMotorEx railLeft;

    private SleeveDetection sleeveDetection;

    private SleeveDetection.ParkingPosition pos;
    private OpenCvCamera camera;
    private String webcamName = "Webcam 1";
    @Override
    public void runOpMode() throws InterruptedException {

        railRight = (DcMotorEx) hardwareMap.dcMotor.get("railRight");
        railLeft = (DcMotorEx) hardwareMap.dcMotor.get("railLeft");
        railLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        railRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        railRight.setDirection(DcMotorSimple.Direction.REVERSE);
        railLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        railRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        railRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        railRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        railRight.setTargetPosition(0);
        railLeft.setTargetPosition(0);
        railLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        railRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .addDisplacementMarker(() -> {
//                    Thread obj = new Thread(new func());
//                    obj.start();
                    railRight.setTargetPosition(300);
                    railLeft.setTargetPosition(300);
                    railLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    railRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .strafeRight(DISTANCE)
                .build();

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName));
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName));
        sleeveDetection = new SleeveDetection();
        camera.setPipeline(sleeveDetection);

        //lets us see stream from driver hub (not really necessary)
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        while (!isStarted()) {
            pos = sleeveDetection.getPosition();
            telemetry.addData("Position: ", pos);
            telemetry.update();
        }

        if (isStopRequested()) return;


        //drive.followTrajectory(trajectory);



        while (!isStopRequested() && opModeIsActive()) ;
    }
}
