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

@Autonomous(name="RR Test", group="SummerCamp")
public class TestFile extends LinearOpMode {
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
            drive.setPoseEstimate(new Pose2d(-32.0, 65.0, Math.toRadians(-90.0)));


        }

        drive.turn(Math.toRadians(90));

        sleep(5000);



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







}
