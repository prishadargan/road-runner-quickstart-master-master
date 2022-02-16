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

@Autonomous(name="Blue Ducks Auto", group="SummerCamp")
public class BlueDuckAuto extends LinearOpMode {
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

        telemetry.addLine("Robot is ready.");
        telemetry.update();
        waitForStart();
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.extention.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.extention.setPower(.1);
        sleep(50);
        robot.extention.setPower(0);
        //robot.extention.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.setPoseEstimate(new Pose2d(-32.0, 65.0, Math.toRadians(-90.0)));


        while (opModeIsActive()) {
            robot.pixyCam.engage();
            team_element_x = 0xff & robot.pixyCam.read(0x51, 5)[1];
            team_element_y = 0xff & robot.pixyCam.read(0x51, 5)[2];
            duck_x = 0xff & robot.pixyCam.read(0x52, 5)[1];
            duck_y = 0xff & robot.pixyCam.read(0x52, 5)[2];
            telemetry.addLine();
            telemetry.addData("Pixy Health :", robot.pixyCam.getHealthStatus());
            telemetry.addLine();
            telemetry.addData("Pixy Connection : ", robot.pixyCam.getConnectionInfo());
            telemetry.update();
        }
        sleep(500000);

         

        lift_top();
        turret_turn45();
        sleep(10);
        extension_outfull();
        robot.collector.setPower(1);
        sleep(200);
        robot.collector.setPower(0);
        turret_back();
        robot.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension_in();
        lift_barriers();
        LAD();

        Trajectory builder1 = drive.trajectoryBuilder(new Pose2d())

                .forward(7)
                .build();
        drive.followTrajectory(builder1);

        Trajectory builder2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-65.0, 57, Math.toRadians(-90)))
                .build();


        drive.followTrajectory(builder2);

        robot.swod.setPower(0.15);

        Trajectory builder2a = drive.trajectoryBuilder(drive.getPoseEstimate())
                .back(2.05)
                .build();


        drive.followTrajectory(builder2a);


        robot.SWOD(0.13);
        sleep(3500);
        robot.SWOD(0);


        Trajectory builder3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(20)
                .build();

        drive.followTrajectory(builder3);

        turn_right(0.780,500);

        /*
        for (int i = 0; i < 57; i++) {
            telemetry.addData("Variant   : ", i);
            telemetry.addData("PIXY-D-X :", duck_x);
            telemetry.addData("PIXY-D-Y : ", duck_y);
            telemetry.addData("PIXY STAT : ", robot.pixyCam.getHealthStatus());
            telemetry.addData("TE-STAT :", te_Stat);
            telemetry.update();
            robot.pixyCam.engage();
            duck_x = 0xff & robot.pixyCam.read(0x52, 5)[1];
            duck_y = 0xff & robot.pixyCam.read(0x52, 5)[2];
            sleep(500);

            if (duck_x < 95 && duck_x != 0) {
                strafe_left(0.3,500);
                robot.stop();
            }
            if (duck_x > 130 && duck_x != 0 ){
                strafe_right(0.3,500);
                robot.stop();
            }
            if (duck_x < 130 && duck_x > 95) {
                if (te_Stat == "Not-Collected"){
                    move_straight(0.25,750);
                    te_Stat = "Collected";
                    telemetry.addData("TE-STAT :", te_Stat);
                } else {
                    telemetry.addData("TE-STAT :", te_Stat);
                }
                telemetry.update();
            }

            sleep(500);

            if (te_Stat == "Collected"){
                i = 500;
            }
        }

         */

        Trajectory builder3b = drive.trajectoryBuilder(drive.getPoseEstimate())
                .back(26)
                .build();

        drive.followTrajectory(builder3b);


        turn_right(0.710,290);
        move_backwards(-0.35, 650);
        lift_top();


        Trajectory builder4a = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(43)
                .build();

        drive.followTrajectory(builder4a);

        turret_turn90();
        sleep(450);
        robot.collector.setPower(1);
        sleep(450);
        robot.collector.setPower(0);
        turret_teleop_pos();
        sleep(10);
        lift_barriers();
        sleep(10);




        Trajectory builder4b = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(25)
                .build();

        drive.followTrajectory(builder4b);

        move_forward(-0.1,500);


        Trajectory builder5 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeLeft(31)
                .build();

        drive.followTrajectory(builder5);

        Trajectory builder6 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(50)
                .build();

        drive.followTrajectory(builder6);

        lift_down();
        sleep(1);

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
    private void lift_middle(){
        sleep(10);
        robot.lift.setTargetPosition((1420));
        robot.lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        runtime.reset();
        robot.lift.setPower((1));
        if((runtime.seconds() > 4) || (!robot.lift.isBusy())) {
            telemetry.addData("status", "complete");
            robot.lift.setPower(0);
        }
    }
    private void lift_bottom(){
        sleep(10);
        robot.lift.setTargetPosition((1390));
        robot.lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        runtime.reset();
        robot.lift.setPower((1));
        if((runtime.seconds() > 4) || (!robot.lift.isBusy())) {
            telemetry.addData("status", "complete");
            robot.lift.setPower(0);
        }
    }
    private void extension_outfull(){
        sleep(1);
        robot.extention.setPower(-0.97);
        sleep(555);
        robot.extention.setPower(0);
    }
    private void extension_in(){
        robot.extention.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.extention.setTargetPosition((0));
        robot.extention.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.extention.setPower(1);
        if ((runtime.seconds() > 1.75) || (!robot.extention.isBusy())) {
            robot.extention.setPower(0);
        }
        if (!robot.frontLimit.getState()) {
            robot.extention.setPower(0.65);
        } else {
            robot.extention.setPower(0);
        }
    }
    private void LAD(){ robot.linearActuator.setPosition((0.79682527)); }

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
        robot.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.turret.setTargetPosition(-700);
        robot.turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        runtime.reset();
        if(runtime.seconds() > 1.5 || !robot.turret.isBusy()) {
            telemetry.addData("Turret Stat : ", "complete");
            robot.turret.setPower(0);
            telemetry.update();
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
