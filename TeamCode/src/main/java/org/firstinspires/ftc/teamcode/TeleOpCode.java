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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.stream.Collector;

import static java.lang.Thread.sleep;

@TeleOp(name="TeleOp - Test", group="Freight-Frenzy")
public class TeleOpCode extends LinearOpMode {
    //Initializes joystick storage variables
    private double leftStickX, leftStickY, rightStickX;
    private ElapsedTime runtime = new ElapsedTime();

    private static final double threshold = 0;
    private double pos = 1;

    @Override

    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, telemetry, this);

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        telemetry.addLine("Starting...");
        telemetry.update();

        while (opModeIsActive()) {

            telemetry.addLine("Running...");

            leftStickX = gamepad1.left_stick_x * -1;
            leftStickY = gamepad1.left_stick_y * 1;


            telemetry.addData("left_stick_x", leftStickX);
            telemetry.addData("left_stick_y", leftStickY);
            telemetry.update();



            if (Math.abs(gamepad1.right_stick_x) > threshold) {
                if (gamepad1.right_stick_x < 0) {
                    rightStickX = -gamepad1.right_stick_x * gamepad1.right_stick_x * -1 * (4.0 / 5.0) - (1.0 / 5.0);
                } else {
                    rightStickX = -gamepad1.right_stick_x * gamepad1.right_stick_x * 1 * (4.0 / 5.0) + (1.0 / 5.0);
                }
            } else {
                rightStickX = 0;
            }



            /*

            if ((Math.abs(gamepad1.left_stick_y) > threshold) || (Math.abs(gamepad1.left_stick_x) > threshold) || Math.abs(gamepad1.right_stick_x) > threshold) {
                //Calculate formula for mecanum drive function
                double addValue = (double) (Math.round((50 * (leftStickY * Math.abs(leftStickY) + leftStickX * Math.abs(leftStickX))))) / 50;
                double subtractValue = (double) (Math.round((50 * (leftStickY * Math.abs(leftStickY) - leftStickX * Math.abs(leftStickX))))) / 50;

                //Set motor speed variables
                robot.setMotorPowers(addValue + rightStickX, subtractValue - rightStickX, subtractValue + rightStickX, addValue - rightStickX);
            } else {
                robot.stop();
            }

             */


            if (gamepad1.left_bumper) {
                robot.Collector(1);
            }
            if (gamepad1.right_bumper) {
                robot.Collector(-1);
            }
            if (gamepad1.right_bumper && gamepad1.left_bumper) {
                robot.Collector(0);
            }


            if (gamepad1.a) {
                if (pos == 1) {
                    robot.lift.setPower(0.2);
                    sleep(200);
                    robot.lift.setPower(0);
                } else {
                    telemetry.addData("Lift Positon", "Postion 1 - Lowest");
                    telemetry.addData("Target Value", robot.newLeftTarget);
                    telemetry.addData("Current Position", robot.lift.getCurrentPosition());
                    robot.state = Robot.states.LIFTING_DOWN;
                    pos = 1;
                }

            }
            if (gamepad1.b) {
                if (pos == 2){
                    robot.lift.setPower(-0.1);
                    sleep(100);
                    robot.lift.setPower(0);
                } else {
                    telemetry.addData("Lift Positon", "Postion 2 - Highest");
                    telemetry.addData("Target Value", robot.nt);
                    telemetry.addData("Current Position", robot.lift.getCurrentPosition());
                    robot.state = Robot.states.LIFTING_UP;
                    pos = 2;
                }
            }

            telemetry.update();
            robot.update();


            if (gamepad1.left_trigger > 0.2) {
                int ttarget;
                ttarget = ((robot.turret.getCurrentPosition()) - (int) (323));
                robot.turret.setTargetPosition((ttarget));
                robot.turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                runtime.reset();
                if (robot.turret.getCurrentPosition() > robot.turret.getTargetPosition()) {
                    robot.turret.setPower(-0.3);
                }
            }


            if (gamepad1.right_trigger > 0.2) {
                int attarget;
                attarget = ((robot.turret.getCurrentPosition()) + (int) (323));
                robot.turret.setTargetPosition((attarget));
                robot.turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                runtime.reset();
                if (robot.turret.getCurrentPosition() < robot.turret.getTargetPosition()) {
                    robot.turret.setPower(0.3);
                }
            }



                if (gamepad1.x) {

                    telemetry.clearAll();
                    int etarget;
                    sleep(50);
                    etarget = ((robot.extention.getCurrentPosition()) + (int) (200));
                    robot.extention.setTargetPosition((etarget));
                    robot.extention.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    runtime.reset();
                    robot.extention.setPower(1);
                    telemetry.addData("Target:", robot.extention.getTargetPosition());
                    telemetry.addData("Current:", robot.extention.getCurrentPosition());
                    telemetry.update();
                    robot.extention.setPower(0);

                    /*
                    sleep(50);
                    robot.extention.setPower(0.75);
                    sleep(750);
                    robot.extention.setPower(0);

                     */


                }

                if (gamepad1.y) {
                    while (!robot.frontLimit.getState()) {
                        robot.extention.setPower(-0.5);
                    }
                    robot.extention.setPower(0);

                }


                if (gamepad2.y) {
                    robot.SWOD(0);
                    telemetry.addData("y", "is true");
                    telemetry.update();

                }

                if (gamepad2.x) {
                    robot.SWOD(1);
                    telemetry.addData("x", "is true");
                    telemetry.update();

                }



            }
        }
    }
