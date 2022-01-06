            package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

/*
link - https://learnroadrunner.com/quickstart-overview.html#are-you-using-drive-encoders
 */

@Autonomous(name="Red Ducks", group="SummerCamp")
public class Autonomous1 extends LinearOpMode {
    Robot robot;
    public double team_element_x;
    public double team_element_y;
    public int lift_height;
    private ElapsedTime runtime = new ElapsedTime();
    @Override

    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, this);

        waitForStart();
        while (!robot.getLimitState()) {
            sleep(1000);
            robot.Dturret(0.35);
        }
        robot.DepositorT.setPower(0);
        robot.DepositorT.setTargetPosition(900);
        robot.DepositorT.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        runtime.reset();
        robot.DepositorT.setPower(-0.3);

        // 250 ticks ~ 1 ft (11-12 inches) |  f == ((250/12) * I)
        robot.Encoders();

        robot.pixyCam.engage();
        team_element_x = 0xff&robot.pixyCam.read(0x51,5)[1];
        team_element_y = 0xff&robot.pixyCam.read(0x51,5)[2];


        /*
        if (team_element_y < 90){
            if (team_element_y > 16) {
                lift_height = 0;
            } else {

            }
        }
        if (team_element_y < 160){
            if (110 < team_element_y){
                lift_height = 2;
            }
        }
        if (170 < team_element_y){
            lift_height = 3;
        }

         */

        for (int i =1; i != 0; i++) {
            telemetry.addData("X Position", team_element_x);
            telemetry.addData("Y Position", team_element_y);

            telemetry.update();
        }


        /*
        robot.ElderWand((300)); // lower ElderWand
        encoderDrive(0.3, 0.3,270,270,10); // move out
        encoderTurn(0.3,-255,255,10); // turn
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
        encoderDrive(-0.4, -0.4, -1450, -1450, 15);
        encoderTurn(0.3,-290, 290, 4); // turn to go straight
        encoderDrive(0.4, 0.4, 790, 790, 5);
        robot.DepositorL.setPower(0.5);
        sleep(lift_height);
        robot.DepositorL.setPower(0);
        robot.DepositorM.setPosition(0.35);
        sleep(500);
        robot.DepositorS.setPosition(0.657);
        sleep(500);
        robot.DepositorS.setPosition(0.1);
        sleep(500);
        robot.DepositorM.setPosition(0.793650790794);
        encoderDrive(0.1, 0.1, 85, 85, 5);
        encoderTurnR(0.3, 145, -145, 5);
        encoderDrive(1, 1, 700, 700, 5);
        encoderTurnR(0.5,200, -200, 5);
        encoderDrive(0.55, 0.55, 355, 355, 5);
        encoderTurn(0.3,-256,256,2);
        encoderDrive(1,1,1300,1300,3);

         */


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