package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.*;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.subsystem.Levels;
import org.firstinspires.ftc.teamcode.subsystem.Slides;

public class Robot {

    // SUBSYSTEM DECLARATIONS

    //public Claw claw;
    public Slides slides;
    //public V4B v4b;

    // STATE VARS
    // example: clawToggled = false;
    Levels currentPosition = Levels.GROUND;
    DcMotorEx slideLeft, slideRight;

    // POSE
//    public Pose robotPose = new Pose(
//        Math.PI/2, Math.PI/2, 0.0,
//        7.41830709, -7.41830709, 0.5748031,
//        0.5748031, 0.5748031, 3.75,
//        0.6968503935
//    );

    // AUTON CONSTANTS
    public SampleMecanumDrive drive;
    boolean auton;

    public enum StarterStack {
        FIVE,
        FOUR,
        THREE,
        TWO,
        ONE
    }

    public Robot(HardwareMap map, boolean auton, DcMotorEx slideLeft, DcMotorEx slideRight){
        this.auton = auton;

        this.slideLeft=slideLeft;
        this.slideRight=slideRight;

        VoltageSensor voltageSensor = map.voltageSensor.iterator().next();

        // INIT SUBSYSTEMS
        //this.claw = new Claw((StepperServo) components[11], (StepperServo) components[8], (StepperServo) components[9], (StepperServo) components[10]);
        this.slides = new Slides(slideLeft, slideRight, voltageSensor);
        //this.v4b = new V4B((StepperServo) components[6], (StepperServo) components[7]);
    }
    public Robot(HardwareMap map, boolean auton){
        this.auton = auton;
        VoltageSensor voltageSensor = map.voltageSensor.iterator().next();

        // INIT SUBSYSTEMS
        //this.claw = new Claw((StepperServo) components[11], (StepperServo) components[8], (StepperServo) components[9], (StepperServo) components[10]);
        //this.slides = new Slides(slideLeft, slideRight, voltageSensor);
        //this.v4b = new V4B((StepperServo) components[6], (StepperServo) components[7]);
    }


    // CONTROL FUNCTIONS

    //CLAW
//    public void toggleClaw(boolean x) {
//        this.claw.toggle();
//    }
//
//    public void advancedToggleClaw(boolean x) {
//        if (currentPosition == Levels.GROUND) {
//            this.claw.toggle();
//        } else if (currentPosition != Levels.ZERO) {
//            this.claw.toggle();
////            sleep(500);
//            groundPreset(false);
//        }
//    }
//
//    public void startClawX(boolean direction) {
//        this.claw.startXRotation(direction);
//    }
//
//    public void startClawY(boolean direction) {
//        this.claw.startYRotation(direction);
//    }

//    public void resetClawRotation(boolean b) {
//        this.claw.resetRotation(Axis.ALL);
//    }


    // SLIDES + V4B + CLAW PRESETS
    public void groundPreset(boolean pad_down) {
        this.slides.runToPreset(Levels.GROUND);
    }

    public void lowPreset(boolean pad_left) {
        this.slides.runToPreset(Levels.LOW);

    }

    public void mediumPreset(boolean pad_right) {
        this.slides.runToPreset(Levels.MEDIUM);

    }

    public void highPreset(boolean pad_up) {
        this.slides.runToPreset(Levels.HIGH);

    }

//    public void starterStackPreset(StarterStack level) {
//        switch (level) {
//            case FIVE:
//                this.slides.runToPosition(-50);
//
//            case FOUR:
//                this.slides.runToPosition(-50);
//
//            case THREE:
//                this.slides.runToPosition(-50);
//                this.v4b.runToPreset(Levels.HIGH);
//                sleep(500);
//                this.claw.clawY.servo.setPosition(90);
//                this.claw.setXRotation(170);
//            case TWO:
//                this.slides.runToPosition(-50);
//                this.v4b.runToPreset(Levels.HIGH);
//                sleep(500);
//                this.claw.clawY.servo.setPosition(90);
//                this.claw.setXRotation(170);
//            case ONE:
//                this.slides.runToPosition(-50);
//                this.v4b.runToPreset(Levels.HIGH);
//                sleep(500);
//                this.claw.clawY.servo.setPosition(90);
//                this.claw.setXRotation(170);
//        }
//    }

    public void robotOff(boolean pad_left, int ticks) {
        this.slides.runToPosition(ticks);

    }



    //DRIVE

    public void resetEncoders() {
        slides.resetAllEncoders();
    }

    private void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (Exception e) {}
    }
}