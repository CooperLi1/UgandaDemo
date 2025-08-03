package org.firstinspires.ftc.teamcode.robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.robot.util.Encoder;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.config.Config;


@Config
public class Outtake extends Subsystem{
    private final DcMotorEx arm, turret;
    private final Servo pivot, wrist, claw;
    private enum State{
        INTAKE, GRAB, INTERMEDIATE, IDLE, SCORE, SCORED, POSTSCORE
    }
    private State state = State.IDLE;
    private State lastState = State.INTAKE;
    private Encoder armEncoder, turretEncoder;

    private double turretAngleToTicks = 1425.1/360*2;
    private double armTarget = 0;
    private double turretTarget = 0;
    private final ElapsedTime timer = new ElapsedTime();

    private final double wait = 200;
    private final double intermediateWait = 700;

    private double clawClose = 0.8;
    private double clawOpen = 0.2;

    private double wristCenter = 0.39;
    private enum WristOffset {
        LEFT(45),
        CENTER(0),
        RIGHT(-45),
        SUPER_RIGHT(-90);

        private final int angle;

        WristOffset(int angle) {
            this.angle = angle;
        }

        public int getAngle() {
            return angle;
        }

        public WristOffset next() {
            return values()[(ordinal() + 1) % values().length];
        }
    }
    WristOffset wristOffset = WristOffset.CENTER;
    private final ElapsedTime wristTimer = new ElapsedTime();

    private double pivotScore = 0.25;
    private double pivotIntake = 0.6;
    private double pivotFold = 0.9;

    private final double grabDelay = 50;

    public Outtake(){
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "arm"));
        turretEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "turret"));
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pivot = hardwareMap.get(Servo.class, "pivot");
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");
    }

    public Outtake(HardwareMap hardwareMap){
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "arm"));
        turretEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "turret"));
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pivot = hardwareMap.get(Servo.class, "pivot");
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");
    }

    @Override
    public void onStart() {
        setIdle();
    }

    @Override
    public void manualControl() {

        armTarget+=controls.armPower()*0.6;
        turretTarget+=controls.turretPower()*0.5;
        telemetry();

        if ((state == State.GRAB || state == State.INTAKE) && controls.toggleWrist() && wristTimer.milliseconds()>250){
            wristOffset = wristOffset.next();
            wristTimer.reset();
        }

        switch(state){
            case INTAKE:
                wrist.setPosition(getWristPos());
                manualOn();
                if(timer.milliseconds()>400){
                    setArmPos(-1630);
                }
                if(waitOver() && controls.stateForward()){
                    lastState = State.INTAKE;
                    timer.reset();
                    state = State.GRAB;
                }
                if(waitOver() && controls.stateBack()){
                    lastState = State.INTAKE;
                    timer.reset();
                    setPostScore();
                    state=State.POSTSCORE;
                }
                break;
            case GRAB:
                setArmPos(-1700);
                wrist.setPosition(getWristPos());
                if(timer.milliseconds()>grabDelay) {
                    claw.setPosition(clawClose);
                }
                if(timer.milliseconds()>grabDelay+wait && controls.stateForward()){
                    lastState = State.GRAB;
                    timer.reset();
                    setIntermediate();
                    state = State.INTERMEDIATE;
                }
                if(timer.milliseconds()>grabDelay+wait && controls.stateBack()){
                    lastState = State.GRAB;
                    timer.reset();
                    setIntake();
                    state = State.INTAKE;
                }
                break;
            case INTERMEDIATE:
                if(timer.milliseconds()>intermediateWait && lastState == State.GRAB){
                    lastState = State.INTERMEDIATE;
                    timer.reset();
                    setIdle();
                    state = State.IDLE;
                }
                if(timer.milliseconds()>intermediateWait && lastState == State.IDLE){
                    lastState = State.INTAKE;
                    timer.reset();
                    setIdle();
                    state = State.IDLE;
                }
                break;
            case IDLE:
                if(waitOver() && controls.stateForward()){
                    lastState = State.IDLE;
                    timer.reset();
                    setScore();
                    state = State.SCORE;
                }
                if(waitOver() && controls.stateBack()){
                    lastState = State.IDLE;
                    timer.reset();
                    setIntermediate();
                    state=State.INTERMEDIATE;
                }
                break;
            case SCORE:
                manualOn();
                if(waitOver() && controls.stateForward()){
                    lastState = State.SCORE;
                    timer.reset();
                    claw.setPosition(clawOpen);
                    state = State.SCORED;
                }
                if(waitOver() && controls.stateBack()){
                    lastState = State.SCORE;
                    timer.reset();
                    setIdle();
                    state=State.IDLE;
                }
                break;
            case SCORED:
                if(waitOver() && controls.stateForward()){
                    lastState = State.SCORED;
                    timer.reset();
                    setPostScore();
                    state = State.POSTSCORE;
                }
                if(waitOver() && controls.stateBack()){
                    lastState = State.SCORED;
                    timer.reset();
                    setIdle();
                    state=State.IDLE;
                }
                break;
            case POSTSCORE:
                if(timer.milliseconds()>400){
                    setArmPos(0);
                }
                if(waitOver() && controls.stateForward()){
                    lastState = State.POSTSCORE;
                    timer.reset();
                    setIntake();
                    state = State.INTAKE;
                }
                break;
        }
    }

    private void manualOn(){
        setArmPos(armTarget);
        setTurretPos(turretTarget);
    }
    private boolean waitOver(){
        return timer.milliseconds()>wait;
    }
    public void setIntake(){
        setArmPos(-1000);
        setTurretPos(0);
        pivot.setPosition(pivotIntake);
        claw.setPosition(clawOpen);
        wrist.setPosition(wristCenter);
    }
    public void setIntermediate(){
        setArmPos(-270);
        setTurretPos(0);
        pivot.setPosition(pivotIntake);
        wrist.setPosition(wristCenter);
    }

    public void setIdle(){
        setArmPos(0);
        setTurretPos(0);
        pivot.setPosition(pivotFold);
        wrist.setPosition(wristCenter);
    }
    public void setScore(){
        setArmPos(-800);
        setTurretPos(0);
        pivot.setPosition(pivotScore);
        wrist.setPosition(wristCenter);
    }

    public void setPostScore(){
        setArmPos(-270);
        setTurretPos(0);
        pivot.setPosition(pivotFold);
        wrist.setPosition(wristCenter);
    }

    public void openClaw(){
        claw.setPosition(clawOpen);
    }

    public void closeClaw(){
        claw.setPosition(clawClose);
    }

    private void setArmPos(double pos){
        armTarget = pos;
        arm.setTargetPosition((int) (pos));
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.65);
    }

    private void setTurretPos(double angle){
        turretTarget = angle;
        turret.setTargetPosition((int) (angle*turretAngleToTicks));
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(1);
    }

    private double getWristPos(){
        double angle = -turretTarget+wristOffset.getAngle();
        return wristCenter+angle*0.00333333333;
    }

    public void telemetry() {
        telemetry.addData("State: ", state);
        telemetry.addData("Timer", timer);
        telemetry.addData("Arm Pos ", arm.getCurrentPosition());
        telemetry.addData("Turret Pos", turret.getCurrentPosition());
    }
}
