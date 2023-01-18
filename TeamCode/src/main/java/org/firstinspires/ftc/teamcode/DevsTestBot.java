package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DcMotorWrapper;
import org.firstinspires.ftc.teamcode.math.TimeManager;
import org.firstinspires.ftc.teamcode.subsystem.GamepadWrapper;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;

@Config
@TeleOp(name="DevMello Test Bot" )
public class DevsTestBot extends LinearOpMode {

    private DcMotorWrapper linSlideLeft;
    private DcMotorWrapper linSlideRight;
    private TimeManager timeManager;

    private final int linSlideLowerBound = 0;
    private final int linSlideUpperBound = 2500;
    private final double linSlidePower = 1;

    private final double[] linSlidePositions = { 0.0, 0.45, 0.6, 0.88,};
    private int linSlidePosition = 0;
    private boolean linSlideUp = false;
    private double linSlideOffset = 0;

    public enum Junction {
        GROUND,
        LOW,
        MEDIUM,
        HIGH
    }

    public static int LOW = -350;
    public static int MEDIUM = -850;
    public static int HIGH = -1750;
    public static int GROUND = -100;
    public static double speedL = 0.01;
    public static double speedR = 0.01;

    protected boolean clawState;

    public static double maxSpeedAxon = 0.14, maxSpeedTorque = 0.25, maxSpeedNormal = 0.11;
    public static double deadZone = 0.1;

    protected CRServoImplEx v4bMoveRight;
    public static double v4bMoveRightStart = -1, v4bMoveRightEnd = 0.2;

    protected CRServoImplEx v4bMoveLeft;
    public static double v4bMoveLeftStart = -1, v4bMoveLeftEnd = 0.2;

    protected CRServoImplEx clawPitch;
    public static double clawPitchStart = -1, clawPitchEnd = 0.5;

    protected CRServoImplEx clawOpen;
    public static double clawOpenStart = 0, clawOpenEnd = 1;

    protected CRServoImplEx clawRot;
    public static double clawRotStart = -.7, clawRotEnd = 1;


    protected DcMotorEx motorFrontLeft;
    protected DcMotorEx motorBackLeft;
    protected DcMotorEx motorFrontRight;
    protected DcMotorEx motorBackRight;



    protected DcMotorEx[] motors = {motorBackLeft, motorBackRight, motorFrontRight, motorFrontLeft};
    protected double x,y,rx;
//    VoltageSensor voltageSensor;
//    Slides slides = new Slides(voltageSensor);
    static int target =0;
    static int target1 = 0;
    boolean slides = false;
    private GamepadWrapper gamepad1Wrapper;
    private GamepadWrapper gamepad2Wrapper;

    @Override
    public void runOpMode() throws InterruptedException {
        this.gamepad2Wrapper = new GamepadWrapper()
                .setGamepad(gamepad2);

        GamepadEx gamepadLib = new GamepadEx(gamepad2);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        this.timeManager = new TimeManager()
                .setOpMode(this);
        DcMotorWrapper.setTimeManager(this.timeManager);

        motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("leftFront");
        motorBackLeft = (DcMotorEx) hardwareMap.dcMotor.get("leftRear");
        motorFrontRight = (DcMotorEx) hardwareMap.dcMotor.get("rightFront");
        motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get ("rightRear");

        DcMotorEx v4bright = (DcMotorEx) hardwareMap.dcMotor.get( "rightSlide");
        DcMotorEx v4bleft = (DcMotorEx) hardwareMap.dcMotor.get("leftSlide");

        this.linSlideLeft = new DcMotorWrapper()
                .setDcMotor(hardwareMap.dcMotor.get("leftSlide"))
                .setLowerBound(this.linSlideLowerBound)
                .setUpperBound(-this.linSlideUpperBound)
                .setPower(-this.linSlidePower);
        this.linSlideRight = new DcMotorWrapper()
                .setDcMotor(hardwareMap.dcMotor.get("rightSlide"))
                .setLowerBound(this.linSlideLowerBound)
                .setUpperBound(this.linSlideUpperBound)
                .setPower(this.linSlidePower);

        DcMotor[] slides = {v4bleft, v4bright};
        for(DcMotor slide : slides) {
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        v4bright.setDirection(DcMotorSimple.Direction.REVERSE);
//        v4bright.setRunMode(Motor.RunMode.PositionControl);
//        v4bleft.setRunMode(Motor.RunMode.PositionControl);
//        double kPright = v4bright.getPositionCoefficient();
//        double kPleft = v4bleft.getPositionCoefficient();
//        v4bleft.setPositionTolerance(CONFIG.TOLERANCE);
//        v4bright.setPositionTolerance(CONFIG.TOLERANCE);


        v4bMoveLeft = (CRServoImplEx) hardwareMap.crservo.get("leftArm");
        v4bMoveRight = (CRServoImplEx) hardwareMap.crservo.get("rightArm");
        clawRot = (CRServoImplEx) hardwareMap.crservo.get("turret");
        clawPitch = (CRServoImplEx) hardwareMap.crservo.get("pitch");
        clawOpen = (CRServoImplEx) hardwareMap.crservo.get("claw");


        boolean rotButtonPrevState = gamepad2.x, clawButtonPrevState = gamepad2.a, v4bButtonPrevState = gamepad2.b;
        boolean rotButtonState = false, clawButtonState = false, v4bButtonState = false;

        boolean slidesButtonPrevState = gamepad2.dpad_up;

        waitForStart();
        if (isStopRequested()) return;
        long time = System.currentTimeMillis();
        long prevTime = System.currentTimeMillis();
        while (opModeIsActive()) {

            double y = -gamepad1.right_stick_x; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.left_stick_y;
//            //rx is actully x
//            //y is actaully  y
//            //x is rx
//            y = -y;
//            double mid = rx;
//            rx = x
//            x = mid;


            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

//Servo code ----------------
//            double rotWriteState = 0, clawWriteState = 0, pitchWriteState = 0;

            if(gamepad2.x && !rotButtonPrevState){
                rotButtonState = !rotButtonState;
                clawRot.setPower(rotButtonState ? clawRotStart : clawRotEnd);
                clawPitch.setPower(rotButtonState ? clawPitchStart : clawPitchEnd);
            }
//            else if()

            if(gamepad2.a && !clawButtonPrevState) {
                clawButtonState = !clawButtonState;
                clawOpen.setPower(clawButtonState ? clawOpenStart : clawOpenEnd);
            }

            //ARM


            if(gamepad2.b && !v4bButtonPrevState) {
                v4bButtonState = !v4bButtonState;
                v4bMoveLeft.setPower(v4bButtonState ? v4bMoveLeftStart : v4bMoveLeftEnd);
                v4bMoveRight.setPower(v4bButtonState ? v4bMoveRightStart : v4bMoveRightEnd);
            }
//            v4bMoveLeft.setPower(-gamepad2.right_stick_x);
//            v4bMoveRight.setPower(-gamepad2.right_stick_x);
            rotButtonPrevState = gamepad2.x;
            clawButtonPrevState = gamepad2.a;
            v4bButtonPrevState = gamepad2.b;
//--------------------------------
            Telemetry dashboardTelemetry = dashboard.getTelemetry();

            //Rails
            v4bleft.setPower(-gamepad2.left_stick_y);
            v4bright.setPower(-gamepad2.left_stick_y);


//            if (gamepad2.dpad_up && !slidesButtonPrevState) {
//                dashboardTelemetry.addLine("Worked");
//                this.linSlidePosition = 1;
//                moveLinSlide();
//            }
            gamepad2Wrapper.subscribeYPressedEvent(() -> {
                this.linSlidePosition = 1;
                moveLinSlide();
                return false;
            });

            if(gamepadLib.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                this.linSlidePosition = 1;
                moveLinSlide();
            }
            slidesButtonPrevState = gamepad2.dpad_up;
//            if(gamepad2.a) {
//                clawOpen.setPower(1);
//            } else if (gamepad2.b) {
//                clawOpen.setPower(0);
//            }


            telemetry.addData("RRM Ticks", v4bright.getCurrentPosition());
            telemetry.addData("LRM Ticks", v4bleft.getCurrentPosition());
            telemetry.addData("Claw Power", clawOpen.getPower());
            dashboardTelemetry.addData("RRM Ticks", v4bright.getCurrentPosition());
            dashboardTelemetry.addData("LRM Ticks", v4bleft.getCurrentPosition());
            dashboardTelemetry.addData("Claw Power", clawOpen.getPower());
            dashboardTelemetry.addData("DPAD", slidesButtonPrevState);
            dashboardTelemetry.update();
            telemetry.update();
            prevTime = System.currentTimeMillis();
        }


    }

    private void moveLinSlide(){
        this.linSlideLeft.setPosition(this.linSlidePositions[linSlidePosition] + linSlideOffset);
        this.linSlideRight.setPosition(this.linSlidePositions[linSlidePosition] + linSlideOffset);
    }


}
@Config
class CONFIG {
    public static int GROUND = 0;
    public static int LOW = -800;
    public static int MEDIUM = -1600;
    public static int HIGH = -2500;
    public static Double TOLERANCE = 13.6;
}