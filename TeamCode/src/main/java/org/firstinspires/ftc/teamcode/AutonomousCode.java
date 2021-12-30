package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
link - https://learnroadrunner.com/quickstart-overview.html#are-you-using-drive-encoders
 */

@Autonomous(name="felipe code things", group="SummerCamp")
public class AutonomousCode extends LinearOpMode {
    Robot robot;
    private ElapsedTime runtime = new ElapsedTime();
    @Override

    public void runOpMode() throws InterruptedException {

        double[] p1 = {0, 0, 0, 0};
        double[] p2 = {0, 0, 0, 0};
        double[] p3 = {0, 0, 0, 0};

        robot = new Robot(hardwareMap, telemetry, this);
        waitForStart();
        robot.Encoders();


        encoderDrive(0.4, -1210, 1210, 100000);







    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        ElapsedTime runtime = new ElapsedTime();
        int newLeftTarget;
        int newRightTarget;
        // Ensure that the opmode is still active
        robot.frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        // Determine new target position, and pass to motor controller
        newLeftTarget = ((robot.frontLeft.getCurrentPosition())  + (int)(leftInches));
        newRightTarget = ((robot.backRight.getCurrentPosition()) + (int)(rightInches));
        robot.frontLeft.setTargetPosition((newLeftTarget));
        robot.backLeft.setTargetPosition((newLeftTarget ));
        robot.frontRight.setTargetPosition(newRightTarget);
        robot.backRight.setTargetPosition(newRightTarget);


        // Turn On RUN_TO_POSITION
        robot.frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);


        // reset the timeout time and start motion.
        runtime.reset();
        robot.frontRight.setPower(Math.abs(speed));
        robot.backRight.setPower(Math.abs(speed));
        robot.frontLeft.setPower((Math.abs(speed)));
        robot.backLeft.setPower((Math.abs(speed)));

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                        (robot.frontLeft.isBusy() && robot.frontRight.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
            telemetry.addData("front encoders",  "Running at Left:Right %7d :%7d",
                    robot.frontLeft.getCurrentPosition(),
                    robot.frontRight.getCurrentPosition());
            telemetry.addData("back encoders",  "Running at Left:Right %7d :%7d",
                    robot.backLeft.getCurrentPosition(),
                    robot.backRight.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        stop();

        // Turn off RUN_TO_POSITION
        robot.frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //  sleep(250);   // optional pause after each move
    }

    public void encoderTurn(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        ElapsedTime runtime = new ElapsedTime();
        int newLeftTarget;
        int newRightTarget;
        // Ensure that the opmode is still active
        robot.frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        // Determine new target position, and pass to motor controller
        newLeftTarget = ((robot.frontLeft.getCurrentPosition())  + (int)(leftInches));
        newRightTarget = ((robot.backRight.getCurrentPosition()) + (int)(rightInches));
        robot.frontLeft.setTargetPosition((newLeftTarget));
        robot.backLeft.setTargetPosition((newLeftTarget ));
        robot.frontRight.setTargetPosition(newRightTarget);
        robot.backRight.setTargetPosition(newRightTarget);


        // Turn On RUN_TO_POSITION
        robot.frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);


        // reset the timeout time and start motion.
        runtime.reset();
        robot.frontRight.setPower(Math.abs(speed));
        robot.backRight.setPower(Math.abs(speed));
        robot.frontLeft.setPower((-Math.abs(speed)));
        robot.backLeft.setPower((-Math.abs(speed)));

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (robot.frontLeft.isBusy() && robot.frontRight.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
            telemetry.addData("front encoders",  "Running at Left:Right %7d :%7d",
                    robot.frontLeft.getCurrentPosition(),
                    robot.frontRight.getCurrentPosition());
            telemetry.addData("back encoders",  "Running at Left:Right %7d :%7d",
                    robot.backLeft.getCurrentPosition(),
                    robot.backRight.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        stop();

        // Turn off RUN_TO_POSITION
        robot.frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //  sleep(250);   // optional pause after each move
    }


    }




