package org.firstinspires.ftc.teamcode.robot;

@TeleOp
public class TeleOp extends Robot {
    @Override
    public void runOpMode() throws InterruptedException {
        init(Subsystem.alliance, false);

        waitForStart();

        teleOp();
    }
}