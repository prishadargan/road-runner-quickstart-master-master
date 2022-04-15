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
    public DcMotorEx collector;
    public I2cDeviceSynch pixyCam;
    public DigitalChannel expanLimit;
    public DigitalChannel cLimit;
    public DigitalChannel frontLimit;
    public CRServo swod;
    public Servo linearActuator;
    public Servo cServo;
    public Servo cap;
    public Object drive;

    public enum states {
        STOPPED_L,STOPPED_T,TURRET_SHARED,TURRET_SHARED_ACT, LIFT_UP, LIFT_UP_ACT,
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



    public Robot(HardwareMap hwMap, Telemetry telemetry, LinearOpMode opMode){
        this.telemetry = telemetry;
        this.opMode = opMode;

        frontLeft = (DcMotorEx) hwMap.dcMotor.get("FL"); //
        frontRight = (DcMotorEx) hwMap.dcMotor.get("FR"); //
        backLeft = (DcMotorEx) hwMap.dcMotor.get("BL"); //
        backRight = (DcMotorEx) hwMap.dcMotor.get("BR"); //
        turret = (DcMotorEx) hwMap.dcMotor.get("Turret"); //
        lift = (DcMotorEx) hwMap.dcMotor.get("Lift-Up"); //
        extention = (DcMotorEx) hwMap.dcMotor.get("Lift-E"); //
        collector = (DcMotorEx) hwMap.dcMotor.get("Collector"); //
        expanLimit = hwMap.digitalChannel.get("E-Limit"); //
        cLimit = hwMap.digitalChannel.get("C-Limit"); //
//      frontLimit = hwMap.digitalChannel.get("F-Limit");
        swod = hwMap.crservo.get("SWOD"); //
        pixyCam = hwMap.i2cDeviceSynch.get("Pixy-Cam"); //
        linearActuator = hwMap.servo.get("TLA"); //
        cap = hwMap.servo.get("YesCap");


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

            case STOPPED_T:
                turret.setPower(0);
                break;

            case TURRET_SHARED:
                turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                turret.setTargetPosition(340);
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                runtime.reset();
                turret.setPower(1);
                state = states.TURRET_SHARED_ACT;
                break;

            case TURRET_SHARED_ACT:
                if ((runtime.seconds() > 3) || (turret.getCurrentPosition() > 330)){
                    state = states.STOPPED_T;
                }
                break;

            case LIFT_UP:
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lift.setTargetPosition(640);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                runtime.reset();
                lift.setPower(1);
                state = states.LIFT_UP_ACT;
                break;

            case LIFT_UP_ACT:
                if ((runtime.seconds() > 3) || (lift.getCurrentPosition() > 635)){
                    state = states.STOPPED_T;
                }
                break;
        }

    }



    public states getState() { return state;}

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

    public void lift_top(){
        lift.setTargetPosition((1780));
        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        runtime.reset();
        lift.setPower((1));
        while ((lift.isBusy())){
            telemetry.addData("Target Position", lift.getTargetPosition());
            telemetry.addData("Current Position", lift.getCurrentPosition());
        }
        if((runtime.seconds() > 5) || (!lift.isBusy())) {
            telemetry.addData("status", "complete");
            lift.setPower(0);
        }
    }

    public void Collector (double power) { collector.setPower(power); }

    public void SWOD (double power) { swod.setPower(power);}

    public void LAdown() { linearActuator.setPosition((0.79682527)); }

    public void LAup() { linearActuator.setPosition((0.39682527)); }

    public void LetsCap(int position)  {cap.setPosition(position/2521);}

}