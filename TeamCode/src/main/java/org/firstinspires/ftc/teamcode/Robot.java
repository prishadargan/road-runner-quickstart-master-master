package org.firstinspires.ftc.teamcode;

    /* Copyright (c) 2017 FIRST. All rights reserved.
     *
     * Redistribution and use in source and binary forms, with or without modification,
     * are permitted (subject to the limitations in the disclaimer below) provided that
     * the following conditions are met:
     *
     * Redistributions of source code must retain the above copyright notice, this list
     * of conditions and the following disclaimer.
     *
     * Redistributions in binary form must reproduce the above copyright notice, this
     * list of conditions and the following disclaimer in the documentation and/or
     * other materials provided with the distribution.
     *
     * Neither the name of FIRST nor the names of its contributors may be used to endorse or
     * promote products derived from this software without specific prior written permission.
     *
     * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
     * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
     * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
     * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
     * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
     * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
     * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
     * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
     * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
     * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
     * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
     */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor.RunMode currentDrivetrainMode;
    public DcMotor DepositorT;
    public DcMotor DepositorL;
    public DcMotor collector;
    public Telemetry telemetry;
    public OpMode opMode;
    public DcMotor swod;
    public Servo DepositorS;
    public Servo DepositorM;
    public Servo ElderWand;
    public CRServo Vex393L;
    public CRServo Vex393U;


    public Robot(HardwareMap hwMap, Telemetry telemetry, LinearOpMode opMode) {
        this.telemetry = telemetry;
        this.opMode = opMode;

        frontLeft = hwMap.dcMotor.get("FL");
        frontRight = hwMap.dcMotor.get("FR");
        backLeft = hwMap.dcMotor.get("BL");
        backRight = hwMap.dcMotor.get("BR");
        swod = hwMap.dcMotor.get("SWOD");
        collector = hwMap.dcMotor.get("collector");
        DepositorT = hwMap.dcMotor.get("Depositor-Turret");
        DepositorL = hwMap.dcMotor.get("Depositor-Lift");
        DepositorS = hwMap.servo.get("Depositor-S");
        DepositorM = hwMap.servo.get("Depositor-Stretch");
        ElderWand = hwMap.servo.get("Elder-Wand");
        Vex393L = hwMap.crservo.get("V-393-L");
        Vex393U = hwMap.crservo.get("V-393-U");




        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        currentDrivetrainMode = DcMotor.RunMode.RUN_USING_ENCODER;
        setMotorModes(currentDrivetrainMode);
        setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Robot", " Is Ready");
        telemetry.update();
    }

    private void setMotorModes(DcMotor.RunMode mode) {
        if (mode != currentDrivetrainMode) {
            frontLeft.setMode(currentDrivetrainMode);
            frontRight.setMode(currentDrivetrainMode);
            backLeft.setMode(currentDrivetrainMode);
            backRight.setMode(currentDrivetrainMode);
            currentDrivetrainMode = mode;
        }
    }

    public void Encoders() {
        currentDrivetrainMode = DcMotor.RunMode.RUN_USING_ENCODER;
        setMotorModes(currentDrivetrainMode);
        setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DepositorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DepositorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DepositorT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DepositorT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void spinningWheelofDeath(double power) {

        swod.setPower(-power);
    }

    public void Collector(double power) {

        collector.setPower(power);
    }



    public void Dlift(double power) {
        DepositorL.setPower(power);
    }

    public void Dturret(double power) {
        DepositorT.setPower(power);
    }
   public void DServo(double position) {
       DepositorS.setPosition(position);
   }
   public void DStretch( double position) {DepositorM.setPosition(position);}
   public void ElderWand(double position) {ElderWand.setPosition(position);}
   // public void ElderWandPos() {ElderWand.getPosition() += 10;}




    public void setMotorPowers(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void fineTuneTurret(double turretFineTuneSpeed) {
        DepositorT.setPower(turretFineTuneSpeed);
    }

    public void extendDepositor(double extension) {
        DepositorM.setPosition(extension);
    }

    public void Vex393U(double power) {Vex393U.setPower(power);}
    public void Vex393L(double power) {Vex393L.setPower(power);}
}



