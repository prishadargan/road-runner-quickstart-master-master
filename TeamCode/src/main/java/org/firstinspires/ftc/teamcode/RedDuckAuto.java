package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import static java.lang.Thread.activeCount;
import static java.lang.Thread.sleep;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;

@Autonomous(name="Red Ducks Auto", group="SummerCamp")
public class RedDuckAuto extends LinearOpMode {
    Robot robot;
    public double team_element_x;
    public double team_element_y;
    public double duck_x;
    public double duck_y;
    public String te_Stat = "Not-Collected";
    public String liftHeight;
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        while(!opModeIsActive()) {
            telemetry.addLine("Robot is ready.");
            telemetry.update();
            robot.pixyCam.engage();
            team_element_x = 0xff & robot.pixyCam.read(0x52, 5)[1];
            team_element_y = 0xff & robot.pixyCam.read(0x52, 5)[2];
            duck_x = 0xff & robot.pixyCam.read(0x51, 5)[1];
            duck_y = 0xff & robot.pixyCam.read(0x51, 5)[2];
            telemetry.addLine();
            telemetry.addData("Pixy Health :", robot.pixyCam.getHealthStatus());
            telemetry.addLine();
            telemetry.addData("Pixy Connection : ", robot.pixyCam.getConnectionInfo());
            telemetry.update();
            // pixy


            /*
            if (team_element_x != 0 && team_element_x < 130) {
                liftHeight = "low";
            } else if (team_element_x > 160) {
                if (team_element_x < 195) {
                    liftHeight = "mid";
                }
            } else {
                liftHeight = "top";
            }
            telemetry.addLine(liftHeight);

             */

            robot.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.extention.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        drive.setPoseEstimate(new Pose2d(-32.0, 65.0, Math.toRadians(-90.0)));


        telemetry.addLine("Robot is ready.");
        telemetry.update();
        lift_barriers();
        sleep(100);
        turret_turn45();
        sleep(100);
        extension_outfull();
        robot.collector.setPower(1);
        sleep(200);
        robot.collector.setPower(0);
        extension_in();
        sleep(25);
        turret_back();
        LAD();
        sleep(100);

// 1

        Trajectory builder1 = drive.trajectoryBuilder(new Pose2d())
                .back(6)
                .build();
        drive.followTrajectory(builder1);

        EncoderTurn90R(0.4, 1);

        broken();
        sleep(10);
        robot.extention.setPower(0.0000001);




        Trajectory builder2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .back(35)
                .build();
        drive.followTrajectory(builder2);


        move_forward(0.2, 1100);


        Trajectory builder2a = drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeRight(9)
                .build();
        drive.followTrajectory(builder2a);


        robot.SWOD(-0.18);
        sleep(2850);
        robot.SWOD(0);

        extension_in();

        Trajectory builder3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(5)
                .build();
        drive.followTrajectory(builder3);

        Trajectory builder3a = drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeLeft(7)
                .build();
        drive.followTrajectory(builder3a);


        turn_right(0.85, 300);


        Trajectory builder3b = drive.trajectoryBuilder(drive.getPoseEstimate())
                .back(1)
                .build();
        drive.followTrajectory(builder3b);





        /*
        for (int i = 0; i < 25; i++) {
            telemetry.addData("Variant   : ", i);
            telemetry.addData("PIXY-D-X :", duck_x);
            telemetry.addData("PIXY-D-Y : ", duck_y);
            telemetry.addData("PIXY STAT : ", robot.pixyCam.getHealthStatus());
            telemetry.addData("TE-STAT :", te_Stat);
            telemetry.update();
            robot.pixyCam.engage();
            duck_x = 0xff & robot.pixyCam.read(0x51, 5)[1];
            duck_y = 0xff & robot.pixyCam.read(0x51, 5)[2];
            for (int v = 0; v < 2; v++) {
                if (duck_x == 0) {
                    move_backwards(0.2, 100);
                }
                if (duck_x != 0) {
                    v = 5;
                }
            }
            if (duck_x < 110 && duck_x != 0) {
                turn_right(0.15, 100);
                robot.stop();
            }
            if (duck_x > 140 && duck_x != 0) {
                turn_left(0.15, 100);
                robot.stop();
            }
            if (duck_x < 135 && duck_x > 110) {
                if (te_Stat == "Not-Collected") {
                    robot.collector.setPower(-1);
                    extension_outfull();
                    telemetry.addLine("Extension Out");
                    telemetry.update();
                    sleep(250);
                    extension_in();
                    telemetry.addLine("Extension In");
                    telemetry.update();
                    sleep(1000);
                    te_Stat = "Collected";
                    telemetry.addData("TE-STAT :", te_Stat);
                } else {
                    telemetry.addData("TE-STAT :", te_Stat);
                }
                telemetry.update();
            }
        }

         */

        Trajectory builder4 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeRight(20)
                .build();
        drive.followTrajectory(builder4);

        sleep(10);
        extension_in();


        Trajectory builder5 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .back(46)
                .build();
        drive.followTrajectory(builder5);

        Trajectory builder5a = drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeLeft(7)
                .build();
        drive.followTrajectory(builder5a);
        turret_teleop_pos();

        turn_left(0.85, 300);

        Trajectory builder6 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(5)
                .build();
        drive.followTrajectory(builder6);

        lift_top();
        sleep(10);
        turret_turn90();
        sleep(100);

        turn_left(0.85, (620));

        Trajectory builder6a = drive.trajectoryBuilder(drive.getPoseEstimate())
                .back(45)
                .build();
        drive.followTrajectory(builder6a);

        sleep(500);

        robot.collector.setPower(1);
        sleep(250);
        robot.collector.setPower(0);
        extension_in();
        turret_reset();
        lift_barries2();


        Trajectory builder7 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .back(25)
                .build();
        drive.followTrajectory(builder7);

        move_backwards(0.2, 500);


        Trajectory builder8 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeLeft(40)
                .build();
        drive.followTrajectory(builder8);


        Trajectory builder9 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .back(25)
                .build();
        drive.followTrajectory(builder9);


        extension_in();



    }


    private void turn_left(double speed, long time) {
        robot.setMotorPowers(-speed, speed, -speed, speed);
        sleep(time);
        robot.stop();
    }

    private void strafe_left(double speed, long time) {
        robot.setMotorPowers(-speed, speed, speed, -speed);
        sleep(time);
        robot.stop();
    }


    private void turn_right(double speed, long time) {
        robot.setMotorPowers(speed, -speed, speed, -speed);
        sleep(time);
        robot.stop();
    }
    private void move_backwards(double speed, long time){
        robot.setMotorPowers(speed, speed, speed, speed);
        sleep(time);
        robot.stop();
    }
    private void move_forward(double speed, long time){
        robot.setMotorPowers(-speed, -speed, -speed, -speed);
        sleep(time);
        robot.stop();
    }


    private void turret_turn45(){
        robot.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.turret.setTargetPosition((robot.turret.getCurrentPosition() + 60));
        robot.turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        runtime.reset();
        if (robot.turret.getCurrentPosition() > robot.turret.getTargetPosition()){
            robot.turret.setPower(0.6);
        }

        if (robot.turret.getCurrentPosition() < robot.turret.getTargetPosition()){
            robot.turret.setPower(-0.6);
        }
        while (runtime.seconds() > 1 || !robot.turret.isBusy()) {
            telemetry.addData("Turret Stat : ", "complete");
            robot.turret.setPower(0);
            telemetry.update();
        }
    }
    private void lift_up(){
        sleep(50);
        int target = (robot.lift.getCurrentPosition() + 10);
        robot.lift.setTargetPosition(target);
        robot.lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        runtime.reset();
        robot.lift.setPower((1));
        while ((robot.lift.isBusy())){
            telemetry.addData("Target Position", robot.lift.getTargetPosition());
            telemetry.addData("Current Position", robot.lift.getCurrentPosition());
        }
        if((runtime.seconds() > 5) || (!robot.lift.isBusy())) {
            telemetry.addData("status", "complete");
            robot.lift.setPower(0);
        }
    }
    private void lift_top(){
        sleep(50);
        robot.lift.setTargetPosition((1730));
        robot.lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        runtime.reset();
        robot.lift.setPower((1));
        while ((robot.lift.isBusy())){
            telemetry.addData("Target Position", robot.lift.getTargetPosition());
            telemetry.addData("Current Position", robot.lift.getCurrentPosition());
        }
        if((runtime.seconds() > 5) || (!robot.lift.isBusy())) {
            telemetry.addData("status", "complete");
            robot.lift.setPower(0);
        }
    }
    private void lift_middle(){
        sleep(50);
        robot.lift.setTargetPosition((600));
        robot.lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        runtime.reset();
        robot.lift.setPower((1));
        while ((runtime.seconds() > 4) || (!robot.lift.isBusy())) {
            telemetry.addData("status", "complete");
            robot.lift.setPower(0);
        }
    }
    private void lift_bottom(){
        sleep(50);
        robot.lift.setTargetPosition((100));
        robot.lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        runtime.reset();
        robot.lift.setPower((1));
        while ((runtime.seconds() > 4) || (!robot.lift.isBusy())) {
            telemetry.addData("status", "complete");
            robot.lift.setPower(0);
        }
    }
    private void extension_outfull(){
        robot.linearActuator.setPosition((0.39682527));
        robot.extention.setPower(-0.75);
        sleep(650);
        robot.extention.setPower(0);
        robot.extention.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void extension_in(){
        robot.extention.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.extention.setTargetPosition(0);
        robot.extention.setPower(0.7);
        sleep(1000);
        robot.extention.setPower(0.35);
    }
    private void LAD(){
        robot.linearActuator.setPosition((0.79682527));
    }
    private void lift_down(){
        robot.lift.setTargetPosition((0));
        robot.lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        runtime.reset();
        robot.lift.setPower((1));
        if((!robot.lift.isBusy())) {
            telemetry.addData("status", "complete");
            robot.lift.setPower(0);
        }
    }

    private void lift_barriers(){
        robot.lift.setTargetPosition((400));
        robot.lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        runtime.reset();
        robot.lift.setPower((1));
        sleep(10);
        while ((!robot.lift.isBusy())) {
            telemetry.addData("status", "complete");
            robot.lift.setPower(0);
        }
    }
    private void turret_back(){
        robot.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.turret.setTargetPosition((robot.turret.getCurrentPosition() - 60));
        robot.turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        runtime.reset();
        robot.turret.setPower(1);
        sleep(500);
        robot.turret.setPower(0);

    }

    private void turret_reset(){
        robot.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.turret.setTargetPosition((0));
        robot.turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        runtime.reset();
        robot.turret.setPower(-1);
        sleep(500);
        robot.turret.setPower(0);

    }


    private void turret_backmore(){
        robot.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.turret.setTargetPosition((robot.turret.getCurrentPosition() + 330));
        robot.turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        runtime.reset();
        if (robot.turret.getCurrentPosition() < robot.turret.getTargetPosition()){
            robot.turret.setPower(0.6);
        }
        if (robot.turret.getCurrentPosition() > robot.turret.getTargetPosition()){
            robot.turret.setPower(-0.6);
        }
        while (!robot.cLimit.getState()) {
            robot.turret.setPower(0.25);
        }
        if(runtime.seconds() > 1 || !robot.turret.isBusy()) {
            robot.turret.setPower(0);
        }

    }

    private void lift_bottom_level(){
        sleep(10);
        robot.lift.setTargetPosition(100);
        robot.lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        runtime.reset();
        robot.lift.setPower((1));
        if((runtime.seconds() > 4) || (!robot.lift.isBusy())) {
            telemetry.addData("status", "complete");
            robot.lift.setPower(0);
        }
    }

    private void lift_barries2(){
        sleep(10);
        robot.lift.setTargetPosition(300);
        robot.lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        runtime.reset();
        robot.lift.setPower((1));
        if((runtime.seconds() > 4) || (!robot.lift.isBusy())) {
            telemetry.addData("status", "complete");
            robot.lift.setPower(0);
        }
    }

    private void turret_teleop_pos(){
        robot.turret.setTargetPosition(700);
        robot.turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.turret.setPower(-1);
        runtime.reset();
        while (runtime.seconds() > 3) {
            telemetry.addData("Turret Stat : ", "complete");
            robot.turret.setPower(0);
            telemetry.update();
        }
    }

    private void broken(){
        lift_middle();
        sleep(500);
        turret_teleop_pos();
        sleep(500);
        lift_bottom();

    }

    //DO NOT TOUCH THIS METHOD UNDER ANY CIRCUMSTANCE!!! DO NOT DO IT!!!! [unless it breaks ;)] BUT IT SHOULD NOT!!!!
    private void turret_turn90(){
        robot.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.turret.setTargetPosition((300));
        robot.turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        runtime.reset();
        if (robot.turret.getCurrentPosition() > robot.turret.getTargetPosition()){
            robot.turret.setPower(0.6);
        }

        if (robot.turret.getCurrentPosition() < robot.turret.getTargetPosition()){
            robot.turret.setPower(-0.6);
        }
        if(runtime.seconds() > 1.5 || !robot.turret.isBusy()) {
            telemetry.addData("Turret Stat : ", "complete");
            robot.turret.setPower(0);
            telemetry.update();
        }
    }

    public void EncoderTurn90R(double speed,double timeoutS) {
        ElapsedTime runtime = new ElapsedTime();
        // Ensure that the opmode is still active
        robot.frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        // Determine new target position, and pass to motor controller

        robot.frontLeft.setTargetPosition((410));
        robot.backLeft.setTargetPosition((485));
        robot.frontRight.setTargetPosition(-600);
        robot.backRight.setTargetPosition(-500);
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

        while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.frontLeft.isBusy() && robot.frontRight.isBusy())) {

            // Display it for the driver.
            telemetry.addData("front encoders",  "Running at Left:Right %7d :%7d",
                    robot.frontLeft.getCurrentPosition(),
                    robot.frontRight.getCurrentPosition());
            telemetry.addData("back encoders",  "Running at Left:Right %7d :%7d",
                    robot.backLeft.getCurrentPosition(),
                    robot.backRight.getCurrentPosition());
            telemetry.update();
        }
        robot.stop();

        sleep(150);

    }

    public void EncoderTurn90L(double speed,double timeoutS) {
        ElapsedTime runtime = new ElapsedTime();
        // Ensure that the opmode is still active
        robot.frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        // Determine new target position, and pass to motor controller

        robot.frontLeft.setTargetPosition((410));
        robot.backLeft.setTargetPosition((485));
        robot.frontRight.setTargetPosition(-600);
        robot.backRight.setTargetPosition(-500);
        // Turn On RUN_TO_POSITION
        robot.frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        // reset the timeout time and start motion.
        runtime.reset();
        robot.frontRight.setPower(Math.abs(speed));
        robot.backRight.setPower(Math.abs(speed));
        robot.frontLeft.setPower(-(Math.abs(speed)));
        robot.backLeft.setPower(-(Math.abs(speed)));

        while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.frontLeft.isBusy() && robot.frontRight.isBusy())) {

            // Display it for the driver.
            telemetry.addData("front encoders",  "Running at Left:Right %7d :%7d",
                    robot.frontLeft.getCurrentPosition(),
                    robot.frontRight.getCurrentPosition());
            telemetry.addData("back encoders",  "Running at Left:Right %7d :%7d",
                    robot.backLeft.getCurrentPosition(),
                    robot.backRight.getCurrentPosition());
            telemetry.update();
        }
        robot.stop();

        sleep(150);

    }



}
