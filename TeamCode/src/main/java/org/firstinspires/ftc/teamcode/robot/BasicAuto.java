package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class BasicAuto extends LinearOpMode {
    @Override
    public void runOpMode() {

        Drivetrain dt = new Drivetrain(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);

        waitForStart();
        if (opModeIsActive()) {
            outtake.setIdle();
            outtake.closeClaw();
            dt.moveForward(90,0.6);
//            sleep(10000);
            dt.waitForConclusion(12000);
            dt.turn(90, Drivetrain.Direction.LEFT, 0.4);
            sleep(5000);
//            dt.waitForConclusion(7000);
            dt.moveForward(50, 0.6);
//            sleep(1000);
            outtake.setScore();
            dt.waitForConclusion(4000);
            sleep(3000);
            sleep(500);
            outtake.openClaw();
            sleep(1000);
            dt.moveForward(-10, 0.6);
            sleep(1000);
            outtake.setIdle();
            sleep(1000);
        }
    }
}
