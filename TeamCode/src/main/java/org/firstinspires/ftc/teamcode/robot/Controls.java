/*
This is where you put all methods regarding TeleOp controls. Gamepads shouldn't be called anywhere else.
 */

package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Controls {
    private Gamepad gamepad1;
    private Gamepad gamepad2;

    public Controls (Gamepad g1, Gamepad g2) {
        gamepad1 = g1;
        gamepad2 = g2;
    }

    public double forwardPower() {
        return -gamepad1.left_stick_x;
    }
    public double turnPower() {
        return -gamepad1.left_stick_y;
    }
    public double armPower() {
        return gamepad1.right_stick_y;
    }
    public double turretPower() {
        return gamepad1.right_stick_x;
    }
    public boolean toggleSlowMode() {
        return gamepad1.left_trigger > 0.5;
    }
    public boolean stateForward() {
        return gamepad1.left_bumper;
    }
    public boolean stateBack() {
        return gamepad1.right_bumper;
    }
    public boolean toggleWrist() {
        return gamepad1.right_trigger > 0.5;
    }
}
