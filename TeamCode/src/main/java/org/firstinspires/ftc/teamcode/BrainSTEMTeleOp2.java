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

    double threshold = 0.2;


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        robot = new Robot(this.hardwareMap, this.telemetry, this);
        robot.Dturret(-0.35);
        sleep(760);
        robot.Dturret(0);
        robot.Encoders();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.Encoders();

        while (!opModeIsActive()) ;

        while (opModeIsActive()) {


            //driver 1

            double leftPower;
            double rightPower;
            double drive = (-gamepad1.left_stick_y);
            double turn = (gamepad1.right_stick_x);
            leftPower = Range.clip(drive + turn, -1.0, 1.0);
            rightPower = Range.clip(drive - turn, -1.0, 1.0);
            robot.setMotorPowers(-leftPower, rightPower, leftPower, rightPower);


            /*
             dead zone that sometimes works
            if (gamepad1.left_stick_y < 0.15 && gamepad1.left_stick_y > -0.15 || gamepad1.right_stick_x < 0.15 && gamepad1.right_stick_x > -0.15) {
                robot.setMotorPowers(0,0,0,0);
            }

             */

            while (gamepad1.right_bumper) {
                robot.spinningWheelofDeath((0.2));
                sleep(500);
                robot.spinningWheelofDeath((0.4));
                sleep(500);
                robot.spinningWheelofDeath((0.6));
                sleep(500);
                robot.spinningWheelofDeath((0.65));
                sleep(1000);

            }
            if (gamepad1.left_bumper) {
                robot.spinningWheelofDeath((0));
            }


            if (gamepad1.right_trigger > threshold) {
                robot.Collector(1);
                robot.Vex393U(0.55);
                robot.Vex393L(0.55);

            } else {
                robot.Collector(0);
                robot.Vex393U(0);
                robot.Vex393L(0);
            }


            if (gamepad1.left_trigger > threshold) {
                robot.Collector(-1);
                robot.Vex393U(.25);
                robot.Vex393L(.25);
            } else {
                robot.Collector(0);
                robot.Vex393U(0);
                robot.Vex393L(0);
            }

            if (gamepad1.b) {
                robot.ElderWand(0.426587301587302); // find position of lowest position (to pick up the team marker)
            }

            if (gamepad1.x) {
                robot.ElderWand(1); // storage position for match
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
            double DExtension = -gamepad2.right_stick_y;
            // Range.scale(DExtension, 0, 1, 1200, 2000);

            if (DExtension == 0) {
                robot.extendDepositor(0.7936507936507937);
            }

            if (DExtension > threshold) {
                robot.extendDepositor(0.35);
            }

            if (gamepad2.a) {
                robot.DepositorL.setTargetPosition(0); // lift to position 1
            }

            if (gamepad2.b) {
                robot.DepositorL.setTargetPosition(0); // lift to reset
            }

            if (gamepad2.x) {
                robot.DepositorL.setTargetPosition(0); // lift to position 2

            }

            if (gamepad2.y) {
                robot.DepositorL.setTargetPosition(0); // lift to position 3

            }

            if (gamepad2.right_trigger > threshold) {
                robot.DServo(0.657); // figure out open position
            } else {
                robot.DServo(0.1); // figure out close position
            }



            if (gamepad2.dpad_left) {

                robot.DepositorT.setTargetPosition(0); // position 1
                robot.DepositorT.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                robot.Dturret(0.25);
                while (robot.DepositorT.isBusy()) {
                }
                robot.Dturret(0);

            }

            if (gamepad2.dpad_down) {
                /*
                robot.DepositorT.setTargetPosition(3600); // position 2
                robot.DepositorT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.Dturret(0);

                 */
                telemetry.addData("P2", "Active");
            }

            if (gamepad2.dpad_right) {
                /*
                robot.DepositorT.setTargetPosition(2400); // position 3
                robot.DepositorT.setPower(-0.5);
                robot.DepositorT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.Dturret(0);
                 */
                robot.DepositorT.setPower(-0.5);
                telemetry.addData("P3", "Active");

            }


            //robot.extendDepositor(DExtension);
            robot.fineTuneTurret(DTurretAdjustment);



            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}