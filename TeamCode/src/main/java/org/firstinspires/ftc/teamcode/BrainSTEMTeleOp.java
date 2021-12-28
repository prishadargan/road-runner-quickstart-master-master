package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp(name="tank-drive", group="Iterative Opmode")

public class BrainSTEMTeleOp extends LinearOpMode {
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
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.Encoders();


        while (!opModeIsActive()) ;

        while (opModeIsActive()) {


            //driver 1

            double leftPower;
            double rightPower;

            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            leftPower = Range.clip(drive + turn, -1.0, 1.0);
            rightPower = Range.clip(drive - turn, -1.0, 1.0);


            if (gamepad1.right_bumper) {
                robot.spinningWheelofDeath(0.65); // add ramp up and hold speed at 0.65 and stop at release
            } else {
                robot.spinningWheelofDeath(0);
            }


            if (gamepad1.right_trigger > threshold) {
                robot.Collector(1);
                robot.Vex393U(.75);
                robot.Vex393L(.75);

            } else {
                robot.Collector(0);
            }


            if (gamepad1.left_trigger > threshold) {
                robot.Collector(-1);
            } else {
                robot.Collector(0);
            }

            if (gamepad1.b = true) {
                robot.ElderWand(1000); // find position of lowest position
            }

            if (gamepad1.x = true) {
                robot.ElderWand(2435); // storage position for match
            }

            // if (gamepad1.dpad_up = true) {
            //  robot.ElderWandPos();
            // }


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
                robot.extendDepositor(2000);
            }

            if (DExtension > threshold) {
                robot.extendDepositor(1200);
            }

            if (gamepad2.a = true) {
                robot.DepositorL.setTargetPosition(0); // lift to position 1
            }

            if (gamepad2.b = true) {
                robot.DepositorL.setTargetPosition(0); // lift to reset


                if (gamepad2.x = true) {
                    robot.DepositorL.setTargetPosition(0); // lift to position 2

                }

                if (gamepad2.y = true) {
                    robot.DepositorL.setTargetPosition(0); // lift to position 3

                }

          /*  if (gamepad2.right_trigger > threshold) {
                robot.DServo(1600); // figure out open position
            } else {
                robot.DServo(700); // figure out close position
            }

           */

                if (gamepad2.dpad_left = true) {
                    robot.DepositorT.setTargetPosition(0); // position 1
                } else {
                    robot.Dturret(0);
                }

                if (gamepad2.dpad_down = true) {
                    robot.DepositorT.setTargetPosition(0); // position 2
                } else {
                    robot.Dturret(0);
                }

                if (gamepad2.dpad_right = true) {
                    robot.DepositorT.setTargetPosition(0); // position 3
                } else {
                    robot.Dturret(0);
                }


                //robot.extendDepositor(DExtension);
                robot.fineTuneTurret(DTurretAdjustment);
                robot.setMotorPowers(-leftPower, rightPower, leftPower, rightPower);


                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
                telemetry.update();
            }
        }
    }
}