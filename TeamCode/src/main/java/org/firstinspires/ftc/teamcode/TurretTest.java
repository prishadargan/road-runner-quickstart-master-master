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
public class TurretTest extends LinearOpMode {
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
        robot.extention.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.extention.setPower(.1);
        sleep(50);
        robot.extention.setPower(0);
        //robot.extention.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.setPoseEstimate(new Pose2d(-32.0, 65.0, Math.toRadians(-90.0)));

        lift_top();
        sleep(1000);
        turret_turn45();
        sleep(1000);
        extension_outfull();
        robot.collector.setPower(1);
        sleep(1000);
        robot.collector.setPower(0);
        sleep(1000);
        extension_in();
        sleep(1000);
        turret_back();
        sleep(1000);
        lift_bottom_level();
        sleep(1000);
        LAD();




    }


    private void turn_left(double speed, long time) {
        robot.setMotorPowers(-speed, speed, -speed, speed);
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
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lift.setTargetPosition((400));
        robot.lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        runtime.reset();
        robot.lift.setPower((1));
        if((!robot.lift.isBusy()) && runtime.seconds() > 1.5) {
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

    //DO NOT TOUCH THIS METHOD UNDER ANY CIRCUMSTANCE!!! DO NOT DO IT!!!! [unless it breaks ;)] BUT IT SHOULD NOT!!!!
    // i made it 0.00000000000000000000000000000001 ms faster hehe
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

}
