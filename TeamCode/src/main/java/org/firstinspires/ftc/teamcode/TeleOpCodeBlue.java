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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.stream.Collector;
import static java.lang.Thread.sleep;


@TeleOp(name="TeleOp - Blue", group="Freight-Frenzy")
public class TeleOpCodeBlue extends LinearOpMode {
    //Initializes joystick storage variables
    private double leftStickX, leftStickY, rightStickX;
    private ElapsedTime runtime = new ElapsedTime();

    private static final double threshold = 0;

    @Override

    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, telemetry, this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addLine("Waiting for start");
        telemetry.update();
        robot.extention.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();



        // move to auto
        robot.turret.setPower(-0.1);
        sleep(250);
        robot.turret.setPower(0.0);





        robot.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addLine("Starting...");
        telemetry.update();


        while (opModeIsActive()) {

            telemetry.addLine("Running...");
            telemetry.addData("Turret Current - C", robot.turret.getCurrentPosition());


            leftStickX = gamepad1.left_stick_x * -1;
            leftStickY = gamepad1.left_stick_y * -1;


            /*
            add back
            telemetry.addData("left_stick_x", leftStickX);
            telemetry.addData("left_stick_y", leftStickY);
            telemetry.update();

             */




            /*
                D1 - Driver 1
             */


            // drive-train





                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        )
                );

                drive.update();

                Pose2d poseEstimate = drive.getPoseEstimate();
                telemetry.addData("x", poseEstimate.getX());
                telemetry.addData("y", poseEstimate.getY());
                telemetry.addData("heading", poseEstimate.getHeading());
                telemetry.update();

            /*
            if (Math.abs(gamepad1.right_stick_x) > threshold) {
                if (gamepad1.right_stick_x < 0) {
                    rightStickX = -gamepad1.right_stick_x * gamepad1.right_stick_x * -1 * (4.0 / 5.0) - (1.0 / 5.0);
                } else {
                    rightStickX = -gamepad1.right_stick_x * gamepad1.right_stick_x * 1 * (4.0 / 5.0) + (1.0 / 5.0);
                }
            } else {
                rightStickX = 0;
            }


            if ((Math.abs(gamepad1.left_stick_y) > threshold) || (Math.abs(gamepad1.left_stick_x) > threshold) || Math.abs(gamepad1.right_stick_x) > threshold) {
                //Calculate formula for mecanum drive function
                double addValue = (double) (Math.round((50 * (leftStickY * Math.abs(leftStickY) + leftStickX * Math.abs(leftStickX))))) / 50;
                double subtractValue = (double) (Math.round((50 * (leftStickY * Math.abs(leftStickY) - leftStickX * Math.abs(leftStickX))))) / 50;

                //Set motor speed variables
                robot.setMotorPowers(addValue + rightStickX, subtractValue + rightStickX, subtractValue + rightStickX, addValue - rightStickX);
            } else {
                robot.stop();
            }

             */


            // collector
            if (gamepad1.left_bumper) {
                robot.Collector(1);
            }
            if (gamepad1.right_bumper) {
                robot.Collector(-1);
            }
            if (gamepad1.right_bumper && gamepad1.left_bumper) {
                robot.Collector(0);
            }


            // swod

            if (gamepad1.x) {
                telemetry.addData("SWOD : ", "On");
                telemetry.update();
                robot.SWOD(0.15);
                sleep(650);
                robot.SWOD(0.35);
                sleep(250);
                robot.SWOD(0.85);
            }

            if (gamepad1.y) {
                robot.SWOD(0);
                telemetry.addData("SWOD : ", "Off");
                telemetry.update();
            }

            /*
                D2 - Driver 2
             */


            // main turret
            if (gamepad2.left_bumper) {
                robot.state = Robot.states.TURRET_RIGHT;
                telemetry.update();
            }

            if (gamepad2.right_bumper) {
                robot.state = Robot.states.TURRET_LEFT;
                telemetry.update();
            }

            if (gamepad2.left_trigger > 0.2) {
                robot.state = Robot.states.TURRET_MID;
            }



            // turret fine adjustment


            if (gamepad2.right_stick_x <  -0.2) {
                robot.state = Robot.states.FINE_ADJ_L;
            }

            if (gamepad2.right_stick_x > 0.2) {
                robot.state = Robot.states.FINE_ADJ_R;

            }

            /*
            if (robot.cLimit.getState()) {
                robot.state = Robot.states.STOPPED_T;
            }

            if (robot.expanLimit.getState()) {
                robot.state = Robot.states.STOPPED_T;
            }

             */


            // lift

            if (gamepad2.x) {
                telemetry.addData("Lowest Positon", "Postion 1 - Lowest");
                telemetry.addData("Target Value", robot.newLeftTarget);
                telemetry.addData("Current Position", robot.lift.getCurrentPosition());
                robot.state = Robot.states.LIFTING_UP;

            }
            if (gamepad2.a) {
                telemetry.addData("Highest Positon", "Postion 2 - Highest");
                telemetry.addData("Target Value", robot.nt);
                telemetry.addData("Current Position", robot.lift.getCurrentPosition());
                robot.state = Robot.states.LIFTING_DOWN;
            }
            if (gamepad2.b) {
                telemetry.addData("Highest Positon", "Postion 2 - Highest");
                telemetry.addData("Target Value", robot.nt);
                telemetry.addData("Current Position", robot.lift.getCurrentPosition());
                robot.state = Robot.states.LIFT_MIDO;
            }
            telemetry.update();
            robot.update();


            // extension related stuff

            if (gamepad2.left_stick_y > 0.2) {
                robot.extention.setPower(0.55);
                sleep(50);
                robot.extention.setPower(0);
            }

            if (gamepad2.left_stick_y < -0.2) {
                robot.extention.setPower(-0.75);
                sleep(50);
                robot.extention.setPower(0);
                robot.extention.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (gamepad2.left_stick_y < 0.2) {
                if (gamepad2.left_stick_y > 0.2) {
                    robot.state = Robot.states.STOPPED_E;
                }
            }

            if (gamepad2.right_trigger > 0.2) {
                robot.linearActuator.setPosition((0.79682527));
                robot.extention.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.extention.setTargetPosition((0));
                robot.extention.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                robot.extention.setPower(-0.8);
                if ((runtime.seconds() > 4) || (!robot.extention.isBusy())) {
                    robot.extention.setPower(0);
                }
                if (!robot.frontLimit.getState()) {

                    robot.extention.setPower(-0.25);
                } else {
                    robot.extention.setPower(0);
                }


            }

            if (gamepad2.dpad_down) {
                robot.linearActuator.setPosition((0.39682527));
                telemetry.addData("TLA", "Down");
            }

            telemetry.update();


        }



    }
}


