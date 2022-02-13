package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import static java.lang.Thread.sleep;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot
{
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;
    public DcMotorEx turret;
    public DcMotorEx lift;
    public DcMotorEx extention;
    public DcMotor collector;
    public I2cDeviceSynch pixyCam;
    public DigitalChannel expanLimit;
    public DigitalChannel cLimit;
    public DigitalChannel frontLimit;
    public CRServo swod;
    public Servo linearActuator;
    public Object drive;


    public enum states {
        LIFTING_UP,  LIFTING_DOWN, STOPPED_L,  LIFT_MIDO, , E_OUT, , E_IN, , STOPPED_E,  TURRET_LEFT, , TURRET_RIGHT, , STOPPED_T, TURRET_MID, FINE_ADJ_L, FINE_ADJ_R, LIFT_FAU, LIFT_FAD,
    }


    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor.RunMode currentDrivetrainMode;
    public states state = states.STOPPED_L;
    private Telemetry telemetry;
    private OpMode opMode;
    public int nt;
    public int newLeftTarget;
    public int ttarget;
    public int attarget;
    public int at;


    public Robot(HardwareMap hwMap, Telemetry telemetry, LinearOpMode opMode){
        this.telemetry = telemetry;
        this.opMode = opMode;

        frontLeft = (DcMotorEx) hwMap.dcMotor.get("FL");
        frontRight = (DcMotorEx) hwMap.dcMotor.get("FR");
        backLeft = (DcMotorEx) hwMap.dcMotor.get("BL");
        backRight = (DcMotorEx) hwMap.dcMotor.get("BR");
        turret = (DcMotorEx) hwMap.dcMotor.get("Turret");
        lift = (DcMotorEx) hwMap.dcMotor.get("Lift-Up");
        extention = (DcMotorEx) hwMap.dcMotor.get("Lift-E");
        collector = hwMap.dcMotor.get("Collector");
        expanLimit = hwMap.digitalChannel.get("E-Limit");
        cLimit = hwMap.digitalChannel.get("C-Limit");
        frontLimit = hwMap.digitalChannel.get("F-Limit");
        swod = hwMap.crservo.get("SWOD");
        pixyCam = hwMap.i2cDeviceSynch.get("Pixy-Cam");
        linearActuator = hwMap.servo.get("TLA");


        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extention.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        setMotorModes(currentDrivetrainMode);

        telemetry.addData("Robot", " Is Ready");
        telemetry.update();
    }

    public void update() throws InterruptedException {
        switch(state) {
            case STOPPED_L:
                lift.setPower(0);
                break;

            case STOPPED_E:
                extention.setPower(0);
                break;

            case STOPPED_T:
                turret.setPower(0);
                break;

            case LIFTING_DOWN:
                sleep(50);
                lift.setTargetPosition((0));
                lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                runtime.reset();
                lift.setPower(-0.25);
                if((runtime.seconds() > 4) || (!lift.isBusy())) {
                    state = states.STOPPED_L;
                    telemetry.addData("status", "complete");
                    telemetry.update();
                }
                break;

            case LIFTING_UP:
                sleep(50);
                ;
                lift.setTargetPosition((1420));
                lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                runtime.reset();
                lift.setPower((1));
                if((runtime.seconds() > 4) || (!lift.isBusy())) {
                    telemetry.addData("status", "complete");
                    state = states.STOPPED_L;
                }
                break;
            case LIFT_MIDO:
                sleep(50);

                lift.setTargetPosition((400));
                lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                runtime.reset();
                lift.setPower((0.45));
                if((runtime.seconds() > 2) || (!lift.isBusy())) {
                    telemetry.addData("status", "complete");
                    state = states.STOPPED_L;
                }
                break;

            case LIFT_FAD:
                sleep(50);
                lift.setTargetPosition((lift.getCurrentPosition() - 75));
                lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                runtime.reset();
                lift.setPower((0.3));

                if((runtime.seconds() > 1) || (!lift.isBusy())) {
                    telemetry.addData("status", "complete");
                    state = states.STOPPED_L;
                }
                break;
            case LIFT_FAU:
                sleep(50);
                lift.setTargetPosition((lift.getCurrentPosition() + 75));
                lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                runtime.reset();
                lift.setPower((0.3));

                if((runtime.seconds() > 1) || (!lift.isBusy())) {
                    telemetry.addData("status", "complete");
                    state = states.STOPPED_L;
                    telemetry.update();
                }
                break;

            case E_IN:
                linearActuator.setPosition(0.7936507924);
                extention.setPower(0.8);
                sleep(750);
                extention.setPower(0);
                extention.setPower(0.15);
                if((runtime.seconds() > 2) || (!frontLimit.getState())) {
                    state = states.STOPPED_E;
                }
                break;
            case E_OUT:
                extention.setPower(-0.8);
                sleep(50);
                extention.setPower(0);
                state = states.STOPPED_E;
                break;

            case TURRET_LEFT:
                turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                turret.setTargetPosition((660));
                turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                runtime.reset();
                turret.setPower(0.3);
                telemetry.addData("Turret Left to +330", turret.getCurrentPosition());
                if ((runtime.seconds() > 3) || !turret.isBusy()) {
                    state = states.STOPPED_T;
                    telemetry.addData("Status : ", "Complete");
                }
                telemetry.update();
                break;

            case TURRET_RIGHT:
                turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                turret.setTargetPosition((0));
                turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                runtime.reset();
                turret.setPower(-0.25);
                if(runtime.seconds() > 3 || !turret.isBusy()) {
                    telemetry.addData("Turret Stat : ", "complete");
                    state = states.STOPPED_T;
                }
                break;

            case TURRET_MID:
                turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                turret.setTargetPosition((330));
                turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                runtime.reset();
                if (turret.getCurrentPosition() > 330){
                    turret.setPower(-0.25);
                }

                if (turret.getCurrentPosition() < 330){
                    turret.setPower(0.25);
                }
                if(runtime.seconds() > 1.5 || !turret.isBusy()) {
                    telemetry.addData("Turret Stat : ", "complete");
                    state = states.STOPPED_T;
                }
                break;
            case FINE_ADJ_L:
                turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                turret.setTargetPosition((turret.getCurrentPosition() + 50));
                turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                runtime.reset();
                if (turret.getCurrentPosition() > turret.getTargetPosition()){
                    turret.setPower(-0.2);
                }

                if (turret.getCurrentPosition() < turret.getTargetPosition()){
                    turret.setPower(0.2);
                }

                if(runtime.seconds() > 1 || !turret.isBusy()) {
                    telemetry.addData("Turret Stat : ", "complete");
                    state = states.STOPPED_T;
                }
                break;
            case FINE_ADJ_R:
                turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                turret.setTargetPosition((turret.getCurrentPosition() - 50));
                turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                runtime.reset();
                if (turret.getCurrentPosition() > turret.getTargetPosition()){
                    turret.setPower(-0.6);
                }

                if (turret.getCurrentPosition() < turret.getTargetPosition()){
                    turret.setPower(0.6);
                }

                if(runtime.seconds() > 1 || !turret.isBusy()) {
                    telemetry.addData("Turret Stat : ", "complete");
                    state = states.STOPPED_T;
                    telemetry.update();
                }
                break;
        }

    }



    public states getState() {
        return state;
    }

    private void setMotorModes(DcMotor.RunMode mode) {
        if(mode != currentDrivetrainMode) {
            frontLeft.setMode(currentDrivetrainMode);
            frontRight.setMode(currentDrivetrainMode);
            backLeft.setMode(currentDrivetrainMode);
            backRight.setMode(currentDrivetrainMode);
            currentDrivetrainMode = mode;
        }
    }

    public void setMotorPowers(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    public void stop() {
        setMotorPowers(0,0,0,0);

    }

    public void Collector (double power) { collector.setPower(power); }

    public void SWOD (double power) { swod.setPower(power);}


}