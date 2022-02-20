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


@TeleOp(name="TeleOp - Red", group="Freight-Frenzy")
public class TeleOpCodeRed extends LinearOpMode {
    //Initializes joystick storage variables
    private double leftStickX, leftStickY, rightStickX;
    private ElapsedTime runtime = new ElapsedTime();

    private static final double threshold = 0;

    @Override

    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, telemetry, this);
        StickyButton sb = new StickyButton();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetry.addLine("Waiting for start");
        telemetry.update();
        //robot.extention.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
                double power = 0.20;
                for (int i = 1; i < 3; i++) {
                    robot.SWOD(-power);
                    power += 0.15;
                }
            }

            if (gamepad1.y) {
                robot.SWOD(0);
                telemetry.addData("SWOD : ", "Off");
                telemetry.update();
            }


            // capping
            if (gamepad1.right_trigger > 0.2) {
                robot.CapDown();
                telemetry.clearAll();
                telemetry.addData("Cap ", " Down");
                telemetry.update();
                sleep(100);
            }


            if (gamepad1.left_trigger > 0.2) {
                robot.CapUp();
                telemetry.clearAll();
                telemetry.addData("Cap ", " Down");
                telemetry.update();
                sleep(100);
            }

            /*
                D2 - Driver 2
             */


            // main turret
            if (gamepad2.right_bumper) {
                robot.state = Robot.states.TURRET_LEFT;
                telemetry.update();
            }

            if (gamepad2.left_bumper) {
                robot.state = Robot.states.TURRET_RIGHT;
                telemetry.update();
            }

            if (gamepad2.left_trigger > 0.2) {
                robot.state = Robot.states.TURRET_MID;
            }


            // turret fine adjustment


            if (gamepad2.dpad_left) {
                robot.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                int ttarget;
                ttarget = ((robot.turret.getCurrentPosition()) - (int) (65));
                robot.turret.setTargetPosition((ttarget));
                robot.turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                runtime.reset();
                robot.turret.setPower(-0.35);
                if ((runtime.seconds() > 1)) {
                    robot.turret.setPower(0);
                    telemetry.addData("Status : ", "Complete");
                }
                telemetry.update();
                robot.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            sb.update(gamepad2.dpad_right);
            if (sb.getState()) {
                robot.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                int ttarget;
                ttarget = ((robot.turret.getCurrentPosition()) + (int) (65));
                robot.turret.setTargetPosition((ttarget));
                robot.turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                runtime.reset();
                robot.turret.setPower(0.35);
                if ((runtime.seconds() > 1)) {
                    robot.turret.setPower(0);
                    telemetry.addData("Status : ", "Complete");
                }
                telemetry.update();
                robot.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

            if (gamepad2.b) {
                telemetry.addData("Highest Positon", "Postion 2 - Highest");
                telemetry.addData("Target Value", robot.nt);
                telemetry.addData("Current Position", robot.lift.getCurrentPosition());
                robot.state = Robot.states.LIFT_MIDO;

            }
            if (gamepad2.a) {
                telemetry.addData("Highest Positon", "Postion 2 - Highest");
                telemetry.addData("Target Value", robot.nt);
                telemetry.addData("Current Position", robot.lift.getCurrentPosition());
                robot.state = Robot.states.LIFTING_DOWN;
            }

            if (gamepad2.x) {
                telemetry.addData("Highest Positon", "Postion 2 - Highest");
                telemetry.addData("Target Value", robot.nt);
                telemetry.addData("Current Position", robot.lift.getCurrentPosition());
                robot.state = Robot.states.LIFTING_UP;
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
                if ((runtime.seconds() > 4)) {
                    robot.extention.setPower(0);
                }
            }

            if (gamepad2.dpad_down) {
                robot.linearActuator.setPosition((0.39682527));
                telemetry.addData("TLA", "Down");
            }

            if (gamepad2.left_stick_button) {
                robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.extention.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            // Alliance Specific Hub


            if (gamepad2.x) {
                telemetry.clearAll();
                runtime.reset();
                robot.lift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                robot.lift.setPower(1);
                sleep(500);
                robot.LAup();
                sleep(500);

                telemetry.addLine();
                telemetry.addData("status : ", " 1");
                telemetry.update();
                sleep(500);

                while (robot.lift.getCurrentPosition() <= 1500 && runtime.milliseconds() < 1500 && opModeIsActive()) ;

                robot.turret.setTargetPosition(-300);
                robot.turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                robot.turret.setPower(0.85);
                robot.lift.setPower(0.05);
                robot.extention.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                robot.extention.setTargetPosition(-490);
                robot.extention.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                robot.extention.setPower(-0.65);
            }


            if (gamepad2.y) {
                robot.turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                robot.extention.setTargetPosition(0);
                robot.extention.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                robot.extention.setPower(1);
                robot.LAdown();
                runtime.reset();
                while (!robot.cLimit.getState() && runtime.milliseconds() < 1300) {
                    robot.turret.setPower(0.85);
                }
                robot.lift.setPower(-0.55);
                sleep(1300);
            }
        }

        telemetry.update();
    }

}

