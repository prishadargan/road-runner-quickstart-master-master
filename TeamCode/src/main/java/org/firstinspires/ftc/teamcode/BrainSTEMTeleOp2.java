package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp(name="TeleOp", group="Iterative Opmode")

public class BrainSTEMTeleOp2 extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private Robot robot;
    private int Pos = 0;

    double threshold = 0.2;


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        robot = new Robot(this.hardwareMap, this.telemetry, this);

        robot.Encoders();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.Encoders();

        while (!opModeIsActive()) ;

        while (opModeIsActive()) {
            robot.collector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.swod.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            telemetry.addData("TeleOP","Active");

            //driver 1

            double leftPower;
            double rightPower;
            double drive = (-gamepad1.left_stick_y);
            double turn = (gamepad1.right_stick_x);
            leftPower = Range.clip(drive + turn, -1.0, 1.0);
            rightPower = Range.clip(drive - turn, -1.0, 1.0);
            robot.setMotorPowers(leftPower, rightPower, leftPower, rightPower);


            /*
             dead zone that sometimes works
            if (gamepad1.left_stick_y < 0.15 && gamepad1.left_stick_y > -0.15 || gamepad1.right_stick_x < 0.15 && gamepad1.right_stick_x > -0.15) {
                robot.setMotorPowers(0,0,0,0);
            }

             */

            while (gamepad1.right_bumper) {
                robot.swod.setPower((-1));
            }

            while (gamepad1.right_trigger > threshold) {
                robot.collector.setPower((1));
            }





            if (gamepad1.b) {
                robot.ElderWand(1059); // find position of lowest position (to pick up the team marker)
            }

            if (gamepad1.x) {
                robot.ElderWand(2520); // storage position for match
            }

            while (gamepad1.dpad_up) {
                double position1 = robot.ElderWand.getPosition();
                position1 = position1 + 0.01;
                robot.ElderWand.setPosition(position1);
                sleep(50);
             }

            while (gamepad1.dpad_down) {
                double position2 = robot.ElderWand.getPosition();
                position2 = position2 - 0.01;
                robot.ElderWand.setPosition(position2);
                sleep(50);
            }


            // gamepad 1 - b is elder wand all the way down (servo) -- done
            // gamepad 1 - x is elder wand storage position for match (servo)
            // gamepad 1 - joystick right turns left and right
            // gamepad 1 - dpad up - elder wand extend out (in increments of 10) ---- need Amish
            // gamepad 2 - right joystick hold out to extend out (position of joystick dictates position of servo, default if retracted) -- done
            // gamepad 2 - left joysitck if fine adjustment on the turret -- done
            // gamepad 2 - dpad is the turret position ---- done
            // gamepad 2 x,a,b,y, is positions for lift = y - level 3, x - level 2, a - level 1, b - reset ----- done
            // gamepad 2 - right trigger open and close servo for depositor ---- done

            // fully extended depositor = 1200
            // extendor in is 2000


            // driver 2
            double DTurretAdjustment = gamepad2.left_stick_x;
            double DExtension = gamepad2.right_stick_y;

            /*
            if (gamepad2.right_stick_y == 0){
                telemetry.addData("G2- Right Stick Y", "0");
            }
            if (gamepad2.right_stick_y != 0){
                telemetry.addData("G2- Right Stick Y", "> 0.2");
            }

             */


            if (gamepad2.right_stick_y != 0) {
               robot.extendDepositor(2000/2522);
            }

            if (gamepad2.right_stick_y == 0) {
                robot.extendDepositor(2522/2522);
            }




            if (gamepad2.b) {
                telemetry.addData("G2-B", "True");
            }
            if (gamepad2.x) {
                telemetry.addData("G2-X", "True");
            }
            if (gamepad2.y) {
                telemetry.addData("G2-Y", "True");
            }


            /*
            if (gamepad2.b) {
                Pos = 1;
                telemetry.addData("Lift Positon", "Postion 1 - Lowest");
                telemetry.update();
                if (Pos == 1){
                    robot.DepositorL.setTargetPosition(0); // positon 1
                }
                if (Pos == 2){
                    robot.DepositorL.setTargetPosition(42);
                }
                if (Pos == 3){
                    robot.DepositorL.setTargetPosition(80);
                }

            }

            if (gamepad2.x) {
                Pos = 2;
                telemetry.addData("Lift Positon", "Postion 2 - Middle");
                telemetry.update();
                if (Pos == 1){
                    robot.DepositorL.setTargetPosition(-42); // positon 1
                }
                if (Pos == 2){
                    robot.DepositorL.setTargetPosition(0);
                }
                if (Pos == 3){
                    robot.DepositorL.setTargetPosition(35);
                }
            }

            if (gamepad2.y) {
                Pos = 3;
                telemetry.addData("Lift Positon", "Postion 3 - Highest");
                telemetry.update();
                if (Pos == 1){
                    robot.DepositorL.setTargetPosition(-42); // positon 1
                }
                if (Pos == 2){
                    robot.DepositorL.setTargetPosition(-35);
                }
                if (Pos == 3){
                    robot.DepositorL.setTargetPosition(0);
                }
            }

             */




            if (gamepad2.right_trigger > 0.1) {
                robot.DServo(0.75);
            } else {
               robot.DServo(0);
            }
            if (gamepad2.left_trigger > 0.1) {
                robot.DServo(-0.75);
            } else {
               robot.DServo(0);
            }

            if (gamepad2.dpad_left) {
                robot.DepositorT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                int target;
                target = ((robot.DepositorT.getCurrentPosition()) + (300));
                robot.DepositorT.setTargetPosition(target); // position 1
                robot.DepositorT.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                runtime.reset();
                robot.DepositorT.setPower(-0.3);
                while (robot.DepositorT.isBusy() && (runtime.seconds() < 2)) {
                    telemetry.addData("Target Value", target);
                    telemetry.addData("Current Position", robot.DepositorT.getCurrentPosition());
                    telemetry.update();
                }
                robot.Dturret(0);
                robot.DepositorT.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);



            }




            if (gamepad2.dpad_right) {
                robot.DepositorT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                int target2;
                target2 = ((robot.DepositorT.getCurrentPosition()) - (300));
                robot.DepositorT.setTargetPosition(target2); // position 2
                robot.DepositorT.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                runtime.reset();
                robot.DepositorT.setPower(0.3);
                while (robot.DepositorT.isBusy() && (runtime.seconds() < 2)) {
                    telemetry.addData("Target Value", target2);
                    telemetry.addData("Current Position", robot.DepositorT.getCurrentPosition());
                    telemetry.update();
                }
                robot.Dturret(0);
                robot.DepositorT.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            }







            //robot.extendDepositor(DExtension);
            //robot.fineTuneTurret(DTurretAdjustment);



            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}