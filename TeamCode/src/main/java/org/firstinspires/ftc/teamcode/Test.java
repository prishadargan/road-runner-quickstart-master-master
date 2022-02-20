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

import static java.lang.Thread.sleep;

import java.util.ServiceLoader;

@Autonomous(name="Test", group="SummerCamp")
public class Test extends LinearOpMode {
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

        while (!opModeIsActive()) {
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

        }
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.extention.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.extention.setPower(.1);
        sleep(50);
        robot.extention.setPower(0);
        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.extention.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        //robot.extention.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.setPoseEstimate(new Pose2d(-32.0, 65.0, Math.toRadians(-90.0)));


        turn_right(0.7, 300);



    }

    public void EncoderTurn90L(double speed,double timeoutS) {
        ElapsedTime runtime = new ElapsedTime();
        // Ensure that the opmode is still active
        robot.frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        // Determine new target position, and pass to motor controller

        robot.frontLeft.setTargetPosition((-410));
        robot.backLeft.setTargetPosition((-485));
        robot.frontRight.setTargetPosition(600);
        robot.backRight.setTargetPosition(500);
        // Turn On RUN_TO_POSITION
        robot.frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        // reset the timeout time and start motion.
        runtime.reset();
        robot.frontRight.setPower(speed);
        robot.backRight.setPower(speed);
        robot.frontLeft.setPower(-speed);
        robot.backLeft.setPower(-speed);

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


    public void EncoderTurn90R(double speed,double timeoutS) {
        ElapsedTime runtime = new ElapsedTime();
        // Ensure that the opmode is still active

        robot.frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontLeft.setTargetPosition((410));
        robot.backLeft.setTargetPosition((485));
        robot.frontRight.setTargetPosition(-600);
        robot.backRight.setTargetPosition(-500);
        robot.frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        runtime.reset();
        robot.frontRight.setPower(-speed);
        robot.backRight.setPower(-speed);
        robot.frontLeft.setPower(speed);
        robot.backLeft.setPower(speed);
        while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.frontLeft.isBusy() && robot.frontRight.isBusy())) {
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

    public void EncoderTurn180R(double speed,double timeoutS) {
        ElapsedTime runtime = new ElapsedTime();
        robot.frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setTargetPosition((900));
        robot.backLeft.setTargetPosition((1100));
        robot.frontRight.setTargetPosition((-1300) * 2);
        robot.backRight.setTargetPosition((-1100) * 2);
        robot.frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        runtime.reset();
        robot.frontRight.setPower(-(speed));
        robot.backRight.setPower(-(speed));
        robot.frontLeft.setPower(speed);
        robot.backLeft.setPower(speed);
        while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.frontLeft.isBusy() && robot.frontRight.isBusy())) {
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
    private void strafe_right(double speed, long time) {
        robot.setMotorPowers(speed, -speed, -speed, speed);
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

    private void turn_90(){
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addData("FL Current - ", robot.frontLeft.getCurrentPosition());
        telemetry.addData("BR Current - ", robot.backRight.getCurrentPosition());
        while((robot.frontLeft.getCurrentPosition() > -495) && (robot.backRight.getCurrentPosition() > -495)){
            robot.setMotorPowers(0.5,-0.5,0.5,-0.5);
        }
        telemetry.update();
    }
    private void turret_turn45(){
        robot.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.turret.setTargetPosition((robot.turret.getCurrentPosition() - 75)); // not 45 sorry
        robot.turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        runtime.reset();
        if (robot.turret.getCurrentPosition() > robot.turret.getTargetPosition()){
            robot.turret.setPower(-0.85);
        }

        if (robot.turret.getCurrentPosition() < robot.turret.getTargetPosition()){
            robot.turret.setPower(0.85);
        }
        if(runtime.seconds() > 1 || !robot.turret.isBusy()) {
            telemetry.addData("Turret Stat : ", "complete");
            robot.turret.setPower(0);
            telemetry.update();
        }
    }
    private void lift_up(){
        sleep(10);
        robot.linearActuator.setPosition((0.39682527));
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
        sleep(10);
        robot.linearActuator.setPosition((0.39682527));
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
    private void lift_mid(){
        sleep(10);
        robot.linearActuator.setPosition((0.39682527));
        robot.lift.setTargetPosition((775));
        robot.lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        runtime.reset();
        robot.lift.setPower((0.75));
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
        sleep(10);
        robot.linearActuator.setPosition((0.39682527));
        robot.lift.setTargetPosition((1420));
        robot.lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        runtime.reset();
        robot.lift.setPower((1));
        if((runtime.seconds() > 4) || (!robot.lift.isBusy())) {
            telemetry.addData("status", "complete");
            robot.lift.setPower(0);
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
    private void extension_outfull(){
        robot.extention.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep(1);
        robot.extention.setPower(-0.97);
        sleep(555);
        robot.extention.setPower(0);

    }
    private void extension_in(){
        robot.extention.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.extention.setTargetPosition(0);
        robot.extention.setPower(0.7);
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
    private void LAD(){ robot.linearActuator.setPosition((0.79682527)); }
    private void LAU(){ robot.linearActuator.setPosition((0.39682527)); }

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
        if((!robot.lift.isBusy())) {
            telemetry.addData("status", "complete");
            robot.lift.setPower(0);
        }
    }

    private void turret_back(){
        robot.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.turret.setTargetPosition((robot.turret.getCurrentPosition() + 60));
        robot.turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        runtime.reset();
        if (robot.turret.getCurrentPosition() < robot.turret.getTargetPosition()){
            robot.turret.setPower(0.6);
        }
        if (robot.turret.getCurrentPosition() > robot.turret.getTargetPosition()){
            robot.turret.setPower(-0.6);
        }
        while (!robot.cLimit.getState()) {
            robot.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.turret.setPower(0.54);
        }
        if(runtime.seconds() > 1 || !robot.turret.isBusy()) {
            robot.turret.setPower(0);
            robot.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

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
            robot.turret.setPower(0.45);
            robot.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }
        if(runtime.seconds() > 1 || !robot.turret.isBusy()) {
            robot.turret.setPower(0);
        }

    }

    private void turret_teleop_pos(){
        LAD();
        robot.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.turret.setTargetPosition(-720);
        robot.turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.turret.setPower(-0.7);
        runtime.reset();
        if(runtime.seconds() > 1.5 || !robot.turret.isBusy()) {
            telemetry.addData("Turret Stat : ", "complete");
            robot.turret.setPower(0);
            telemetry.update();
            robot.extention.setPower(0.4);
            sleep(100);
            robot.extention.setPower(0);
        }

    }
    private void turret_turn90(){
        robot.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.turret.setTargetPosition((robot.turret.getCurrentPosition() - 300));
        robot.turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        runtime.reset();
        if (robot.turret.getCurrentPosition() > robot.turret.getTargetPosition()){
            robot.turret.setPower(-0.7);
        }

        if (robot.turret.getCurrentPosition() < robot.turret.getTargetPosition()){
            robot.turret.setPower(0.7);
        }
        if(runtime.seconds() > 1.5 || !robot.turret.isBusy()) {
            telemetry.addData("Turret Stat : ", "complete");
            robot.turret.setPower(0);
            telemetry.update();
        }
    }

    private void turret_teleop_pos1(){
        robot.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.turret.setTargetPosition(700);
        robot.turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.turret.setPower(-1);
        runtime.reset();
        if(runtime.seconds() > 1.5 || !robot.turret.isBusy()) {
            telemetry.addData("Turret Stat : ", "complete");
            robot.turret.setPower(0);
            telemetry.update();
        }
    }

}


