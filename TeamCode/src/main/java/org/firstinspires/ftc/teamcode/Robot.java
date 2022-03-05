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
    public Servo cServo;
    public Object drive;

    public enum states {
        STOPPED_L, G2X, G2X_ACT,GX2_ACT_2,G2B, G2B_ACT, G2B_ACT_2,  G2A, G2A_ACT,  G2Y, G2Y_ACT,  G2LB, G2LB_ACT,  G2DL, G2DL_ACT, G2DR, G2DR_ACT
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
        collector = hwMap.dcMotor.get("Collector"); //
        expanLimit = hwMap.digitalChannel.get("E-Limit"); //
        cLimit = hwMap.digitalChannel.get("C-Limit"); //
//      frontLimit = hwMap.digitalChannel.get("F-Limit");
        swod = hwMap.crservo.get("SWOD"); //
        pixyCam = hwMap.i2cDeviceSynch.get("Pixy-Cam"); //
        linearActuator = hwMap.servo.get("TLA"); //


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
            case G2X:
                turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                turret.setTargetPosition(670);
                telemetry.addLine("1");
                telemetry.update();
                sleep(500);
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                telemetry.addLine("2");
                telemetry.update();
                sleep(500);
                turret.setPower(-0.6);
                telemetry.addLine("3");
                telemetry.update();
                sleep(500);
                state = states.G2X_ACT;
                break;
            case G2X_ACT:
                runtime.reset();
                if (runtime.seconds() > 1.5 && turret.getCurrentPosition() > 670) {
                    telemetry.addLine("4");

                    sleep(500);
                    lift.setTargetPosition(0);
                    telemetry.addLine("5");

                    sleep(500);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    telemetry.addLine("6");

                    sleep(500);
                    lift.setPower(-0.6);
                    telemetry.update();
                }
                break;
            case G2B_ACT_2:

                break;
            case G2B:
                turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                turret.setTargetPosition(0);
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                runtime.reset();
                turret.setPower(0.6);
                while (runtime.seconds() < 1.5 && turret.getCurrentPosition() > 45);
                lift.setTargetPosition(0);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                runtime.reset();
                lift.setPower(-0.6);
                while (runtime.seconds() < 1.75 && lift.isBusy());
                break;
            case G2A:
                runtime.reset();
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lift.setTargetPosition(650);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                runtime.reset();
                lift.setPower(0.5);
                while (runtime.seconds() < 0.87 && lift.getCurrentPosition() < 600);
                turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                turret.setTargetPosition(340);
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPower(.85);
                while (runtime.seconds() < 3 && lift.isBusy());
                break;
            case G2Y:
                lift.setTargetPosition(1584);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                runtime.reset();
                lift.setPower(1);
                while (runtime.seconds() < 0.85 && lift.getCurrentPosition() < 1400);
                turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                turret.setTargetPosition(340);
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPower(.85);
                while (runtime.seconds() < 4 && turret.isBusy() && lift.isBusy());
                break;
            case G2LB:
                extention.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                extention.setTargetPosition(0);
                extention.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extention.setPower(1);
                while (extention.getCurrentPosition() < -65);
                extention.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                extention.setPower(.05);
                break;
            case G2DL:
                turret.setTargetPosition(turret.getCurrentPosition() - (50));
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                runtime.reset();
                turret.setPower(0.5);
                while(runtime.seconds() < 0.25);
                break;
            case G2DR:
                turret.setTargetPosition(turret.getCurrentPosition() + (50));
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                runtime.reset();
                turret.setPower(0.5);
                while(runtime.seconds() < 0.25);
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

    public void CapUp() {cServo.setPosition(0.9999999);}

    public void CapDown() {cServo.setPosition(0.20618);}

    public void LAdown() { linearActuator.setPosition((0.79682527)); }

    public void LAup() { linearActuator.setPosition((0.39682527)); }

}