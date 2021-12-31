package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
link - https://learnroadrunner.com/quickstart-overview.html#are-you-using-drive-encoders
 */

@Autonomous(name="Red Ducks", group="SummerCamp")
public class Autonomous1 extends LinearOpMode {
    Robot robot;
    private ElapsedTime runtime = new ElapsedTime();
    @Override

    public void runOpMode() throws InterruptedException {

        // 250 ticks ~ 1 ft (11-12 inches) |  f == ((250/12) * I)
        robot = new Robot(hardwareMap, telemetry, this);
        waitForStart();
        robot.Encoders();

        robot.ElderWand((300)); // lower ElderWand
        encoderDrive(0.3, 0.3,295,295,10); // move out
        encoderTurn(0.3,-244,244,10); // turn
        encoderDrive(-0.1, -0.1,-300,-300,10); // move back and maybe get team element
        sleep(250); // stop briefly
        encoderDrive(-0.1, -0.1, 850,850,10); // move forward to the end of the tile
        sleep(250); // stop briefly
        robot.ElderWand((2521)); // raise the Elder Wand

        encoderDrive(0.2, 0.2, 150, 150, 10); // go forward again
        encoderTurn(0.3,-145,145,10); // turn to face carousel
        encoderDrive(0.3,0.3, 150,150,10); // go towards carousel

        robot.setMotorPowers(0.2,0.2,0.2,0.2); // snug up the the carousel by time
        sleep(2000);
        robot.stop();

        //
        // swod
        robot.spinningWheelofDeath((0.2));
        sleep(500);
        robot.spinningWheelofDeath((0.4));
        sleep(500);
        robot.spinningWheelofDeath((0.6));
        sleep(500);
        robot.spinningWheelofDeath((0.7));
        sleep(1000);
        robot.spinningWheelofDeath((0));
        //
        encoderDrive(-0.4, -0.4, -1000,-100,5);
        encoderDrive(-0.4, -0.4, -1470, -1470, 15); //
        encoderTurn(0.3,-310, 310, 4); // turn to go straight //
        encoderDrive(0.4, 0.4, 890, 890, 5);
        robot.DepositorM.setPosition(0.35);
        sleep(500);
        robot.DepositorS.setPosition(0.657);
        sleep(500);
        robot.DepositorS.setPosition(0.1);
        sleep(500);
        robot.DepositorM.setPosition(0.793650790794);
        encoderTurnR(0.3, 145, -145, 5);
        encoderDrive(1, 1, 930, 930, 5);
        encoderTurnR(0.5,160, -160, 5);
        encoderDrive(0.55, 0.55, 80, 80, 5);

        

    }

    public void encoderDrive(double leftspeed, double rightspeed,
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
        robot.frontRight.setPower((rightspeed));
        robot.backRight.setPower((rightspeed));
        robot.frontLeft.setPower(((leftspeed)));
        robot.backLeft.setPower(((leftspeed)));

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
        robot.stop();

        // Turn off RUN_TO_POSITION
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        robot.stop();
        // Turn off RUN_TO_POSITION
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sleep(250);
    }

    public void encoderTurnR(double speed,
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
        robot.frontRight.setPower(-Math.abs(speed));
        robot.backRight.setPower(-Math.abs(speed));
        robot.frontLeft.setPower((Math.abs(speed)));
        robot.backLeft.setPower((Math.abs(speed)));

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
        robot.stop();
        // Turn off RUN_TO_POSITION
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sleep(250);
    }


}