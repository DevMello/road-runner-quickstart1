package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
//import org.firstinspires.ftc.teamcode.subsystem.Slides;


@TeleOp(name = "slides")
public class SlidesAndDrive extends LinearOpMode {
    public DcMotorEx v4bright;
    public DcMotorEx v4bleft;
    protected DcMotorEx motorFrontLeft;
    protected DcMotorEx motorBackLeft;
    protected DcMotorEx motorFrontRight;
    protected DcMotorEx motorBackRight;

    @Override
    public void runOpMode() throws InterruptedException {


        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();
        motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("leftFront");
        motorBackLeft = (DcMotorEx) hardwareMap.dcMotor.get("leftRear");
        motorFrontRight = (DcMotorEx) hardwareMap.dcMotor.get("rightFront");
        motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get ("rightRear");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

//        Motor slides1 = (Motor) components[4];
//        Motor slides2 = (Motor) components[5];
//        slides1.setTarget(1);
//        slides1.motor.setTargetPosition(1);
//        slides2.setTarget(1);
//        slides2.motor.setTargetPosition(1);
//        slides1.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        slides2.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        slides1.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        //Slides slides = new Slides(v4bright, v4bleft, voltageSensor);

        double x;
        double y;
        double rx;

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            //DRIVE
            if (gamepad1.right_bumper){
                x = gamepad1.right_stick_x*0.25;
                y = -gamepad1.left_stick_y*0.25;
                rx = gamepad1.left_stick_x*0.25;

            } else{
                x = gamepad1.right_stick_x;
                y = -gamepad1.left_stick_y;
                rx = gamepad1.left_stick_x;
            }


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
            DcMotorEx backLeft = motorBackLeft;
            DcMotorEx backRight = motorBackRight;
            DcMotorEx frontLeft = motorFrontLeft;
            DcMotorEx frontRight = motorFrontRight;
            frontLeft.setPower((float)powerFrontLeft);
            frontRight.setPower((float)powerFrontRight);
            backLeft.setPower(-(float)powerBackLeft);
            backRight.setPower(-(float)powerBackRight);


//            //SLIDES
//            if (gamepad1.dpad_down)
//                slides.runToPreset(Levels.GROUND);
//            else if (gamepad1.dpad_left)
//                slides.runToPreset(Levels.LOW);
//            else if (gamepad1.dpad_right)
//                slides.runToPreset(Levels.MEDIUM);
//            else if (gamepad1.left_bumper)
//                slides.runToPreset(Levels.HIGH);
//            slides.update();
////            if (gamepad1.left_trigger > 0.5) {
////                slides1.setSpeed(-1);
////                slides2.setSpeed(1);
////            } else if (gamepad1.left_trigger < 0.5) {
////                slides1.setSpeed(0);
////                slides2.setSpeed(0);
////            }
//
//            telemetry.addData("slides target ", slides.target);
//            telemetry.addData("slides pos1", slides.v4bright.getCurrentPosition());
//            telemetry.addData("slides pos2", slides.v4bleft.getCurrentPosition());
//            telemetry.addData("slides power 1", slides.power1);
//            telemetry.addData("slides power 2", slides.power2);
//            telemetry.update();

            previousGamepad1.copy(gamepad1);
            previousGamepad2.copy(gamepad2);
        }
    }
}