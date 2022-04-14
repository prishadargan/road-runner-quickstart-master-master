package org.firstinspires.ftc.teamcode;

/*
Copyright (c) 2016 Robert Atkinson
All rights reserved.
Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:
Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.
Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

this policy is nice

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Thread.sleep;

import android.util.Log;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp(name="TeleOp", group="Freight-Frenzy")
public class TeleOpCode extends LinearOpMode {
    //Initializes joystick storage variables 2t
    private double leftStickX, leftStickY, rightStickX;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime opmodetime = new ElapsedTime();
    public  double SWODpower;
    public double SWODrampup;
    private static final double threshold = 0.2;
    public String mode = "shared";
    public String color = "color";
    private boolean liftingUp = false;
    private boolean gotoleft = false;
    private boolean gotoright = false;
    private boolean goingallup = false;
    private boolean extensionin = false;
    //private double lessSpeed = 1;


    @Override

    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, telemetry, this);
        StickyButton depositButton = new StickyButton();
        StickyButton controlButton = new StickyButton();
        StickyButton expansionButton = new StickyButton();
        StickyButton gamepad_y = new StickyButton();
        StickyButton extendin = new StickyButton();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetry.addLine("Waiting for start");
        telemetry.update();


        while (!opModeIsActive() && !isStopRequested()){
            telemetry.addLine("PRESS X IF PLAYING BLUE ");
            telemetry.addLine("PRESS B IF PLAYING RED ");
            robot.linearActuator.setPosition(0.7936507924);
            if (gamepad1.x){
                SWODpower = 0.15;
                SWODrampup = 0.09;
                color = "blue";
                telemetry.clearAll();
                telemetry.addLine("Alliance Color:  Blue");
            }

            if (gamepad1.b) {
                SWODpower = -0.15;
                SWODrampup = -0.09;
                color = "red";
                telemetry.clearAll();
                telemetry.addLine("Alliance Color:  Red");
            }

            telemetry.update();

            telemetry.update();

        }



        waitForStart();
        opmodetime.reset();

        telemetry.addLine("Starting...");
        telemetry.update();


        //reset encoders
        robot.extention.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);





        while (opModeIsActive()) {

            telemetry.addLine("Running...");
            telemetry.addData("Turret Encoder - ", robot.turret.getCurrentPosition());
            telemetry.addData("Lift Encoder - ", robot.lift.getCurrentPosition());
            telemetry.addData("Mode: ", mode);
            telemetry.addData("Color: ", color);
            telemetry.addData("Lift Status :", robot.lift.isBusy());
            telemetry.addData("Time (seconds) :", opmodetime.seconds());


            leftStickX = gamepad1.left_stick_x * -1;
            leftStickY = gamepad1.left_stick_y * -1;

           /* if (Math.abs(gamepad1.right_stick_x) > threshold) {
                if (gamepad1.right_stick_x < 0) {
                    rightStickX = -gamepad1.right_stick_x * gamepad1.right_stick_x * -1 * (4.0 / 5.0) - (1.0 / 5.0);
                } else {
                    rightStickX = -gamepad1.right_stick_x * gamepad1.right_stick_x * 1 * (4.0 / 5.0) + (1.0 / 5.0);
                }
            }

            if ((Math.abs(gamepad1.left_stick_y) > threshold) || (Math.abs(gamepad1.left_stick_x) > threshold) || Math.abs(gamepad1.right_stick_x) > threshold) {
                //Calculate formula for mecanum drive function
                double addValue = (double) (Math.round((50 * (leftStickY * Math.abs(leftStickY) + leftStickX * Math.abs(leftStickX))))) / 50;
                double subtractValue = (double) (Math.round((50 * (leftStickY * Math.abs(leftStickY) - leftStickX * Math.abs(leftStickX))))) / 50;
                //Set motor speed variables
                robot.setMotorPowers(addValue + rightStickX, subtractValue - rightStickX, subtractValue + rightStickX, addValue - rightStickX);
            }

          else {
              robot.stop();
            }

            */
            // drive-train


            drive.setWeightedDrivePower(
                    new Pose2d(
                            -(gamepad1.left_stick_y),
                            -(gamepad1.left_stick_x),
                            -(gamepad1.right_stick_x/2)

                    )
            );



            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();




            // reset encoder
            if (gamepad1.x){
                SWODpower = 0.15;
                SWODrampup = 0.09;
                color = "blue";
                telemetry.addLine("Blue");
            }

            if (gamepad1.b){
                SWODpower = -0.15;
                SWODrampup = -0.09;
                color = "red";
                telemetry.addLine("Red");
            }


            if (gamepad1.y){
                robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.extention.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            // swod
            if (gamepad1.right_bumper) {
                telemetry.addData("SWOD : ", "On");
                telemetry.update();

                for (int i = 1; i < 3; i++) {
                    robot.SWOD(SWODpower);
                    SWODpower += SWODrampup;
                    sleep(500);
                }
            }

            if (gamepad1.left_bumper) {
                robot.SWOD(0);
                telemetry.addData("SWOD : ", "Off");
                telemetry.update();
            }
                if(gamepad1.dpad_right) {
                robot.LetsCap(1200);
            }

            // capping
            if (gamepad1.dpad_up) {
                robot.linearActuator.setPosition(robot.linearActuator.getPosition() - 0.005);
            }

            if (gamepad1.dpad_down) {
                robot.linearActuator.setPosition(robot.linearActuator.getPosition() + 0.005);
            }







            // collector
            if  (gamepad2.right_trigger > 0.2) {
                robot.Collector(-1); // In
            }

            if  (gamepad2.left_trigger > 0.2) {
                robot.Collector(0.6); // Out
            }

            if ((gamepad2.left_trigger > 0.2) && (gamepad2.right_trigger > 0.2)) {
                robot.Collector(0);
            }

            if  (gamepad2.right_bumper) {
                robot.Collector(1); // out overide
            }





            // extension
            if (gamepad2.left_stick_y < -0.1 || gamepad2.left_stick_y > 0.1) {
                robot.extention.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.extention.setPower((gamepad2.left_stick_y));
            }

            if(gamepad2.left_stick_y < 0.1 && gamepad2.left_stick_y > -0.1) {
                robot.extention.setPower(0);
            }

/*
            extendin.update(gamepad2.left_bumper);
            if(extendin.getState()) {
                robot.extention.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.extention.setTargetPosition(0);
                robot.extention.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                runtime.reset();
                robot.extention.setPower(1);
                extensionin = true;
            }
            if (goingallup && (robot.extention.getCurrentPosition() < 50 || runtime.seconds() > 5)) {
                robot.extention.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.extention.setPower(0.07);
                extensionin = false;
            }


 */







            // khadified functions

            controlButton.update(gamepad2.x);
            if(controlButton.getState()) {
                robot.turret.setTargetPosition(680);
                robot.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                runtime.reset();
                robot.turret.setPower(0.3);
                gotoleft = true;
            }


            if (gotoleft && (robot.turret.getCurrentPosition() > 650 || runtime.seconds() > 2)) {
                robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.lift.setTargetPosition(0);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift.setPower(0.5);
                gotoleft = false;
                //lessSpeed = 1;

            }
            if(!gotoleft && robot.lift.getCurrentPosition() < 3) {
                robot.lift.setPower(0);
            }
            else if(!gotoleft) {
                robot.lift.setPower(.5);
            }
            expansionButton.update(gamepad2.b);
            if(expansionButton.getState()) {
                robot.turret.setTargetPosition(0);
                robot.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                runtime.reset();
                robot.turret.setPower(0.3);
                gotoright = true;
            }
            if (gotoright && (robot.turret.getCurrentPosition() < 50 || runtime.seconds() > 2)) {
                robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.lift.setTargetPosition(0);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift.setPower(0.5);
                gotoright = false;
            }



            //shared hub position
            if (gamepad2.right_stick_button){
                mode = "alliance";
            }
            if (gamepad2.left_stick_button){
                mode = "shared";
            }

            depositButton.update(gamepad2.a);
            if(depositButton.getState()) {
                robot.lift.setTargetPosition(680);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                runtime.reset();
                robot.lift.setPower(1);
                liftingUp = true;
            }

            if (liftingUp && (robot.lift.getCurrentPosition() > 600 || runtime.seconds() > 2)) {
                robot.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.turret.setTargetPosition(340);
                robot.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.turret.setPower(0.5);
                liftingUp = false;
                //lessSpeed = 1.5;
            }


            gamepad_y.update(gamepad2.y);
            if(gamepad_y.getState()) {
                robot.lift.setTargetPosition(1622);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                runtime.reset();
                robot.lift.setPower(0.3);
                goingallup = true;
            }

            if (goingallup && (robot.lift.getCurrentPosition() > 300 || runtime.seconds() > 2)) {
                robot.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.turret.setTargetPosition(340);
                robot.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.turret.setPower(0.5);
                goingallup = false;
            }




            // lift
            if (gamepad2.right_stick_y < -0.2 || gamepad2.right_stick_y > 0.2) {
                robot.lift.setTargetPosition((int) (robot.lift.getTargetPosition() + -gamepad2.right_stick_y * 10));
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift.setPower(1);
            }



            if(gamepad2.dpad_left) {
                robot.turret.setTargetPosition(robot.turret.getCurrentPosition() - (20));
                robot.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                runtime.reset();
                robot.turret.setPower(0.5);
            }

            if(gamepad2.dpad_right) {
                robot.turret.setTargetPosition(robot.turret.getCurrentPosition() + (20));
                robot.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                runtime.reset();
                robot.turret.setPower(0.5);
            }

            telemetry.update();
            /*

*/
        }

    }

}