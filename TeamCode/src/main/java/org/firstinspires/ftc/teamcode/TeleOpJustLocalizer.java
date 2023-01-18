package org.firstinspires.ftc.teamcode;

import android.util.Log;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
//import org.firstinspires.ftc.teamcode.subsystem.Slides;

import static org.firstinspires.ftc.teamcode.DevsTestBot.*;
import static org.firstinspires.ftc.teamcode.DevsTestBot.HIGH;

/**
 * This opmode assumes you have your own robot class and simply wish to utilize Road Runner's
 * packaged localizer tools.
 */
@TeleOp(group = "advanced")
public class TeleOpJustLocalizer extends LinearOpMode {

//    public static Slides slides;
    public static double DISTANCE1 = 40;
    public static double DISTANCE2 = 70; // in
    public static double DISTANCE3 = 110; // in
    public static double rotation = 230;
    public static double rotation1 = -230;

    public static double rotation2 = -230;

//    private void movev4b(DevsTestBot.Junction junction){
//        switch(junction){
//            case GROUND:
//                v4bleft.setTargetPosition(GROUND);
//                v4bright.setTargetPosition(GROUND);
//                v4bright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                v4bleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                break;
//            case LOW:
//                v4bleft.setTargetPosition(LOW);
//                v4bright.setTargetPosition(LOW);
//                v4bright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                v4bleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                break;
//            case MEDIUM:
//                v4bleft.setTargetPosition(MEDIUM);
//                v4bright.setTargetPosition(MEDIUM);
//                v4bright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                v4bleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                break;
//            case HIGH:
//                v4bleft.setTargetPosition(HIGH); // tune later
//                v4bright.setTargetPosition(HIGH);
//                v4bright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                v4bleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                Log.d("Test", "high");
//                break;
//        }
//    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize your own robot class
        Robot robot = new Robot(hardwareMap);

        // This is assuming you are using StandardTrackingWheelLocalizer.java
        // Switch this class to something else (Like TwoWheeTrackingLocalizer.java) if your
        // configuration differs

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);

        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
//                .strafeLeft(STRAFE)
                .forward(DISTANCE1)
                .build();

        Trajectory trajectory2 = drive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE2)
                .build();

        Trajectory trajectory3 = drive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE3)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Make sure to call myLocalizer.update() on *every* loop
            // Increasing loop time by utilizing bulk reads and minimizing writes will increase your
            // odometry accuracy
            drive.update();

            // Retrieve your pose
            Pose2d myPose = drive.getPoseEstimate();
            drive.followTrajectory(trajectory1);
            //movev4b(DevsTestBot.Junction.HIGH);

            // Print your pose to telemetry
            telemetry.addData("x", myPose.getX());
            telemetry.addData("y", myPose.getY());
            telemetry.addData("heading", myPose.getHeading());
            telemetry.update();

//            // Teleop driving part
//            // Mecanum example code from gm0
//            // https://gm0.org/en/stable/docs/software/mecanum-drive.html
//            double x = gamepad1.left_stick_x;
//            double y = -gamepad1.left_stick_y;
//            double rx = gamepad1.right_stick_x;
//
//            // Set drive power
//            robot.setDrivePower(x, y, rx);
        }

    }

    // Simple custom robot class
    // Holds the hardware for a basic mecanum drive
    class Robot {
        private DcMotorEx leftFront, leftRear, rightRear, rightFront;

        public Robot(HardwareMap hardwareMap) {
            // Initialize motors
            leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
            leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
            rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
            rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

            // Reverse right side motor directions
            // This may need to be flipped to the left side depending on your motor rotation direction
            rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
            rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

            // Turn on bulk reads to help optimize loop times
            for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }
        }

        // Mecanum example code from gm0
        // https://gm0.org/en/stable/docs/software/mecanum-drive.html
        public void setDrivePower(double x, double y, double rx) {
            double powerFrontLeft = y + x + rx;
            double powerFrontRight = y - x - rx;
            double powerBackLeft = y - x + rx;
            double powerBackRight = y + x - rx;

            if (Math.abs(powerFrontLeft) > 1 || Math.abs(powerBackLeft) > 1 ||
                    Math.abs(powerFrontRight) > 1 || Math.abs(powerBackRight) > 1) {
                // Find the largest power
                double max;
                max = Math.max(Math.abs(powerFrontLeft), Math.abs(powerBackLeft));
                max = Math.max(Math.abs(powerFrontRight), max);
                max = Math.max(Math.abs(powerBackRight), max);

                // Divide everything by max (it's positive so we don't need to worry
                // about signs)
                powerFrontLeft /= max;
                powerBackLeft /= max;
                powerFrontRight /= max;
                powerBackRight /= max;
            }

            leftFront.setPower(powerFrontLeft);
            rightFront.setPower(powerFrontRight);
            leftRear.setPower(powerBackLeft);
            rightRear.setPower(powerBackRight);
        }
    }
}
class PoseStorage {
    public static Pose2d currentPose = new Pose2d();
}