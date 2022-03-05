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
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Thread.sleep;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp(name="TeleOp", group="Freight-Frenzy")
public class TeleOpCode extends LinearOpMode {
    //Initializes joystick storage variables    2t
    private double leftStickX, leftStickY, rightStickX;
    private ElapsedTime runtime = new ElapsedTime();

    private static final double threshold = 0;
    public  double SWODpower;
    public double SWODrampup;
    public String mode = "shared";
    public String color = "color";

    @Override

    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, telemetry, this);
        StickyButton sb = new StickyButton();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetry.addLine("Waiting for start");
        telemetry.update();

        while (!opModeIsActive()){
            robot.linearActuator.setPosition(0.7936507924);
            if (gamepad1.x){
                SWODpower = 0.2;
                SWODrampup = 0.15;
                color = "blue";
                telemetry.addLine("Blue");
            }

            if (gamepad1.b){
                SWODpower = -0.2;
                SWODrampup = -0.15;
                color = "red";
                telemetry.addLine("Red");
            }

            telemetry.update();

        }
        waitForStart();

        telemetry.addLine("Starting...");
        telemetry.update();


        //reset encoders
        robot.extention.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);




        while (opModeIsActive()) {

            telemetry.addLine("Running...");
            telemetry.addData("Turret Current - C", robot.turret.getCurrentPosition());
            telemetry.addData("Lift Current - C", robot.lift.getCurrentPosition());
            telemetry.addData("Mode:", mode);
            telemetry.addData("Color:", color);


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




            // reset encoder
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


            // capping
            if (gamepad1.dpad_up) {
                robot.linearActuator.setPosition(robot.linearActuator.getPosition() - 0.005);
            }

            if (gamepad1.dpad_down) {
                robot.linearActuator.setPosition(robot.linearActuator.getPosition() + 0.005);
            }




            /*
                         D2 - Driver 2
             */



            // collector
            if  (gamepad2.right_trigger > 0.2) {
                robot.Collector(-1); // In
            }

            if  (gamepad2.left_trigger > 0.2) {
                robot.Collector(.5); // Out
            }

            if (!(gamepad2.left_trigger > 0.2) && !(gamepad2.right_trigger > 0.2)) {
                robot.Collector(0);

            }



            // extension
            if (gamepad2.left_stick_y < -0.1 || gamepad2.left_stick_y > 0.1) {
                robot.extention.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.extention.setPower((gamepad2.left_stick_y)/2);
            }



            if(gamepad2.left_bumper) {
                robot.extention.setTargetPosition(0);
                robot.extention.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.extention.setPower(1);
                robot.extention.setPower(.05);
            }




            if(gamepad2.x) {
                robot.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.turret.setTargetPosition(670);
                robot.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.turret.setPower(-0.6);
                sleep(1000);
                robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.lift.setTargetPosition(0);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift.setPower(0.6);
                runtime.reset();
                while (runtime.seconds() < 2 && robot.lift.isBusy());


            }

            if(gamepad2.b) {
                robot.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.turret.setTargetPosition(0);
                robot.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.turret.setPower(0.6);
                sleep(1000);
                robot.lift.setTargetPosition(0);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift.setPower(-0.6);
                runtime.reset();
                while (runtime.seconds() < 2 && robot.lift.isBusy());

            }


            //shared hub position

            if (gamepad2.right_stick_button){
                mode = "alliance";
            }

            if (gamepad2.left_stick_button){
                mode = "shared";
            }


            if(gamepad2.a && mode == "shared") {
                runtime.reset();
                robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.lift.setTargetPosition(650);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift.setPower(0.5);
                sleep(500);
                robot.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.turret.setTargetPosition(340);
                robot.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.turret.setPower(.85);
                while (runtime.seconds() < 3 && robot.lift.isBusy());
            }

            if(gamepad2.a && mode == "alliance") {
                robot.lift.setTargetPosition(1585);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift.setPower(1);
                sleep(500);
                robot.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.turret.setTargetPosition(340);
                robot.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.turret.setPower(.85);

                while (runtime.seconds() < 3 && robot.lift.isBusy());
            }

            if (gamepad1.b) {
                robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.lift.setTargetPosition(2500);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift.setPower(1);
                telemetry.addData("Lift Up", "doing that now");
                sleep(2000000);
                while(runtime.seconds() < 2 && robot.lift.isBusy());
            }



            // lift
            if (gamepad2.right_stick_y < -0.2 || gamepad2.right_stick_y > 0.2) {
                robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.lift.setPower((-gamepad2.right_stick_y)/2);
            }

            if (gamepad2.right_stick_y < 0.2 && gamepad2.right_stick_y > -0.2){
                robot.lift.setPower(0);
            }


            //turret

            if(gamepad2.dpad_left) {
                robot.turret.setTargetPosition(robot.turret.getCurrentPosition() - (50));
                robot.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.turret.setPower(0.5);
                while(!robot.turret.isBusy() && runtime.seconds() > 1.5) {
                    robot.turret.setPower(0);
                }
            }

            if(gamepad2.dpad_right) {
                robot.turret.setTargetPosition(robot.turret.getCurrentPosition() + (50));
                robot.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.turret.setPower(0.5);
                while(!robot.turret.isBusy() && runtime.seconds() > 1.5) {
                    robot.turret.setPower(0);
                }
            }


        }

        telemetry.update();
    }

}
