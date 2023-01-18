package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.robocol.Command;
import org.firstinspires.ftc.teamcode.lib.Motor;

public class DevFtcLib extends CommandOpMode {
    private GamepadEx gamepadEx1, gamepadEx2;
    protected MotorEx motorFrontLeft;
    protected MotorEx motorBackLeft;
    protected MotorEx motorFrontRight;
    protected MotorEx motorBackRight;

    @Override
    public void initialize() {
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);
        motorFrontLeft = new MotorEx(hardwareMap, "leftFront");
        motorBackLeft = new MotorEx(hardwareMap, "leftRear");
        motorFrontRight = new MotorEx(hardwareMap, "rightFront");
        motorBackRight = new MotorEx(hardwareMap, "rightRear");
    }






}
