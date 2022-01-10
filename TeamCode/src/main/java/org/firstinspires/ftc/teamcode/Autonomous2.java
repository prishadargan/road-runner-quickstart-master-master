package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (name="Blue Ducks", group="Summer Camp")
public class Autonomous2 extends LinearOpMode {
    Robot robot;
    private ElapsedTime runtime = new ElapsedTime();
    public double team_element_x;
    public double team_element_y;
    double liftHeight;
    @Override

    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, this);
        waitForStart();

       while (!robot.limit.getState()) {
            robot.DepositorT.setPower(-0.35);
       }
        robot.DepositorT.setPower(0);
        robot.DepositorT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.Encoders();

        robot.pixyCam.engage();
        team_element_x = 0xff&robot.pixyCam.read(0x51,5)[1];
        team_element_y = 0xff&robot.pixyCam.read(0x51,5)[2];

        for (int i = 0; i < 10; i++) {
            sleep(100);

            if (team_element_x > 75) {

            }
            if (team_element_x > 100) {
                liftHeight = 0;
            }

            if (105 < team_element_x) {
                if (team_element_x < 180) {
                    liftHeight = 2;
                } else {
                    liftHeight = 4;
                }

            }
        }
        sleep(500);
        robot.ElderWand(100);
        encoderDrive(0.4, 0.3, 330, 330, 10);
        encoderTurn(0.4, -280, 280, 10);
        encoderDrive(0.1, 0.1, 300, 300, 10);
        sleep(250);
        encoderDrive(0.1, 0.1, -850, -850, 10);
        sleep(250);
        robot.ElderWand(2500);
        encoderTurn(0.5, -500, 500, 10);
        encoderDrive(0.2, 0.2, 165, 165, 10);
        encoderTurnR(0.3, 200, -200, 10);
        encoderDrive(0.3, 0.3, 75, 75, 10);
        robot.setMotorPowers(0.2, 0.2, 0.2, 0.2);
        sleep(1000);
        robot.stop();
        robot.spinningWheelofDeath((-0.2));
        sleep(500);
        robot.spinningWheelofDeath((-0.4));
        sleep(500);
        robot.spinningWheelofDeath((-0.6));
        sleep(500);
        robot.spinningWheelofDeath((-0.65));
        sleep(1000);
        robot.spinningWheelofDeath(0);
        encoderDrive(-0.4, -0.4, -250,-1000,5);
        encoderDrive(-0.4, -0.4, -1400, -1400, 15);
        encoderTurnR(0.3,320,-320,6);
        encoderDrive(0.4, 0.4, 750, 750, 5);
        robot.DepositorM.setPosition(0.35);
        sleep(500);
        robot.DepositorS.setPower(-0.99);
        sleep(500);
        robot.DepositorS.setPower(0);
        sleep(500);
        robot.DepositorM.setPosition(0.76);
        encoderDrive(0.4, 0.4, 370,370,5);
        encoderTurn(0.2,-97,97,5);
        encoderDrive(0.5,0.5,1000,1000,10);
        encoderTurn(0.3,-30,-30,5);
        encoderDrive(0.3,0.3,1200,1200,4);



        /*
        alt parking code
        encoderDrive(0.4, 0.4, 750,750,5);
        encoderTurn(0.5, -256, 256, 5);
        encoderDrive(0.4, 0.4, 650,650,5);

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
