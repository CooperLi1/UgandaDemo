package org.firstinspires.ftc.teamcode.robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.util.Encoder;
public class Drivetrain extends Subsystem{

    private DcMotorEx left, right;
    private Encoder leftEncoder, rightEncoder;

    private final double slowSpeed = 0.4;

    private final double fastSpeed = 1.0;
    private double multiplier = fastSpeed;
    private double forward, turn;

    private double leftPower, rightPower;

    private final double CM_TO_TICKS = 17.8286693538;
    private final double DEGREES_TO_TICKS = 3.0494453208;
    private ElapsedTime waitTimer = new ElapsedTime();

    public enum Direction{
        LEFT, RIGHT;
    }

    public Drivetrain(){
        left = hardwareMap.get(DcMotorEx.class, "driveLeft");
        right = hardwareMap.get(DcMotorEx.class, "driveRight");
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "driveLeft"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "driveRight"));
    }

    public Drivetrain(HardwareMap hardwareMap){
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
        if(controls.toggleSlowMode()){
            multiplier = slowSpeed;
        }else{
            multiplier = fastSpeed;
        }

        forward = controls.forwardPower();
        turn = controls.turnPower();

        leftPower = forward + turn;
        rightPower = forward - turn;

        // Normalize
        double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (maxPower > 1.0) {
            leftPower /= maxPower;
            rightPower /= maxPower;
        }

        left.setPower(leftPower*multiplier);
        right.setPower(rightPower*multiplier);
    }

    public void moveForward(double dist, double pow){
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int ticks = (int) (dist*CM_TO_TICKS);
        left.setTargetPosition(ticks);
        right.setTargetPosition(-ticks);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setPower(pow);
        right.setPower(pow);
    }

    public void turn(double deg, Direction dir, double pow){
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int ticks = (int) (deg*DEGREES_TO_TICKS);
        if(dir == Direction.LEFT) {
            left.setTargetPosition(ticks);
            right.setTargetPosition(ticks);
        }else{
            left.setTargetPosition(-ticks);
            right.setTargetPosition(-ticks);
        }
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setPower(pow);
        right.setPower(pow);
    }

    public void waitForConclusion(double maxTime){
        waitTimer.reset();
        while(left.isBusy() || right.isBusy()){
            if(waitTimer.milliseconds()>maxTime){
                return;
            }
        }
    }
}
