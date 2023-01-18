package org.firstinspires.ftc.teamcode.drive.opmode;

import java.util.Date;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;

import com.qualcomm.robotcore.hardware.DcMotorEx;
//import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.subsystem.Slides;


/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class StraightTest extends LinearOpMode {
//    DcMotorEx v4bright = (DcMotorEx) hardwareMap.dcMotor.get("railRight");
//    DcMotorEx v4bleft = (DcMotorEx) hardwareMap.dcMotor.get("railLeft");
//    Robot robot;
    public static double STRAFE = 20;
    protected static DcMotorEx v4bright;
    protected static DcMotorEx v4bleft;
    //public static Slides slides;
    public static double DISTANCE1 = 40;
    public static double DISTANCE2 = 70; // in
    public static double DISTANCE3 = 110; // in
    public static double rotation = 230;
    public static double rotation1 = -230;

    public static double rotation2 = -230;

    //Pose2d startPose = new Pose2d(-36, 58, Math.toRadians(-90));

    @Override
    public void runOpMode() throws InterruptedException {
        v4bright = (DcMotorEx) hardwareMap.dcMotor.get("railRight");
        v4bleft = (DcMotorEx) hardwareMap.dcMotor.get("railLeft");
        v4bleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        v4bright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        v4bright.setDirection(DcMotorSimple.Direction.REVERSE);
        v4bleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        v4bright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        v4bright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        v4bright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        v4bright.setTargetPosition(0);
        v4bleft.setTargetPosition(0);
        v4bleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        v4bright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                .addDisplacementMarker(() -> {
//                    Thread obj = new Thread(new func());
//                    obj.start();
                    v4bright.setTargetPosition(300);
                    v4bleft.setTargetPosition(300);
                    v4bleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    v4bright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                //.forward(DISTANCE1)
                .build();

        Trajectory trajectory2 = drive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE2)
                .build();

        Trajectory trajectory3 = drive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE3)
                .build();


        waitForStart();

        if (isStopRequested()) return;
//        runSlides(300);N
        //robot.slides.runToPreset(Levels.HIGH);
        //movev4b(Junction.HIGH);
        drive.followTrajectory(trajectory1);


//        drive.turn(Math.toRadians(rotation));
//        drive.followTrajectory(trajectory2);
//        drive.turn(Math.toRadians(rotation1));
//        drive.followTrajectory(trajectory3);
//        drive.turn(Math.toRadians(rotation2));


        while (!isStopRequested() && opModeIsActive()) ;
    }

}
