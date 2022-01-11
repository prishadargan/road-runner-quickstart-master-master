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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;




import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;
    public DcMotor.RunMode currentDrivetrainMode;
    public DcMotorEx DepositorT;
    public DcMotorEx DepositorL;
    public DcMotor collector;
    public Telemetry telemetry;
    public OpMode opMode;
    public DcMotor swod;
    public CRServo DepositorS;
    public Servo DepositorM;
    public Servo ElderWand;
    public I2cDeviceSynch pixyCam;
    public DigitalChannel limit;




    public Robot(HardwareMap hwMap, Telemetry telemetry, LinearOpMode opMode) {
        this.telemetry = telemetry;
        this.opMode = opMode;


        frontLeft = (DcMotorEx) hwMap.dcMotor.get("FL");
        frontRight = (DcMotorEx) hwMap.dcMotor.get("FR");
        backLeft = (DcMotorEx) hwMap.dcMotor.get("BL");
        backRight = (DcMotorEx) hwMap.dcMotor.get("BR");
        swod = hwMap.dcMotor.get("SWOD");
        collector = hwMap.dcMotor.get("collector");
        DepositorT = (DcMotorEx) hwMap.dcMotor.get("Depositor-Turret");
        DepositorL = (DcMotorEx) hwMap.dcMotor.get("Depositor-Lift");
        DepositorS = hwMap.crservo.get("Depositor-S"); // depositing servo
        DepositorM = hwMap.servo.get("Depositor-Extend");
        ElderWand = hwMap.servo.get("Elder-Wand");
        pixyCam = hwMap.i2cDeviceSynch.get("Pixy-Cam");
        limit = hwMap.digitalChannel.get("Limit-Switch");



        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
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
        currentDrivetrainMode = DcMotorEx.RunMode.RUN_USING_ENCODER;
        setMotorModes(currentDrivetrainMode);
        setMotorModes(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        DepositorL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        DepositorL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        DepositorT.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        DepositorT.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void spinningWheelofDeath(double power) {

        swod.setPower(-power);
    }

    public void Collector(double power) {

        collector.setPower(power);
    }



    public void Dlift(double power) { DepositorL.setPower(power);}

    public void Dturret(double power) {DepositorT.setPower(power);}

    public void DServo(double position) {
        DepositorS.setPower(position);
    }

    public void DStretch(double position) {
        DepositorM.setPosition(position);
    }

    public void ElderWand(double position) {
        ElderWand.setPosition(position/2520);
    }


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

    public void fineTuneTurret(double turretFineTuneSpeed) { DepositorT.setPower(turretFineTuneSpeed); }

    public void extendDepositor(double extension) { DepositorM.setPosition(extension/2522); }

    public boolean getLimitState() {
        return limit.getState();
    }


    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        // Ensure that the opmode is still active
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        // Determine new target position, and pass to motor controller
        newLeftTarget = ((frontLeft.getCurrentPosition())  + (int)(leftInches));
        newRightTarget = ((backRight.getCurrentPosition()) + (int)(rightInches));
        frontLeft.setTargetPosition((newLeftTarget));
        backLeft.setTargetPosition((newLeftTarget ));
        frontRight.setTargetPosition(newRightTarget);
        backRight.setTargetPosition(newRightTarget);
        // Turn On RUN_TO_POSITION
        frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        // reset the timeout time and start motion.
        frontRight.setPower(Math.abs(speed));
        backRight.setPower(Math.abs(speed));
        frontLeft.setPower((Math.abs(speed)));
        backLeft.setPower((Math.abs(speed)));

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.


        // Display it for the driver.
        telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
        telemetry.addData("front encoders",  "Running at Left:Right %7d :%7d",
                frontLeft.getCurrentPosition(),
                frontRight.getCurrentPosition());
        telemetry.addData("back encoders",  "Running at Left:Right %7d :%7d",
                backLeft.getCurrentPosition(),
                backRight.getCurrentPosition());
        telemetry.update();

        // Stop all motion;
        stop();

        // Turn off RUN_TO_POSITION
        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

    }
}