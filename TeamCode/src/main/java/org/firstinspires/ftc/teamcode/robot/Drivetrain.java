package org.firstinspires.ftc.teamcode.robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.util.Encoder;
public class Drivetrain extends Subsystem{

    //Declare Motors/Encoders
    private DcMotorEx left, right;
    private Encoder leftEncoder, rightEncoder;

    public Drivetrain(){
        //Initializing and setting up motors
        left = hardwareMap.get(DcMotorEx.class, "driveLeft");
        right = hardwareMap.get(DcMotorEx.class, "driveRight");
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "driveLeft"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "driveRight"));
    }

    @Override
    public void onStart() {

    }
    public void manualControl() {
    }

}
