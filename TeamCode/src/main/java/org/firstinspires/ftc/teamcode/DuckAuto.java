package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import static java.lang.Thread.sleep;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class DuckAuto {
    //Enums
    private enum LiftHeight {
        LOW, MID, TOP
    }

    public enum AllianceColor {
        RED, BLUE
    }

    //Initializing variables
    private Robot robot;
    private LiftHeight height = LiftHeight.TOP;
    private AllianceColor color;
    private LinearOpMode opMode;
    private Telemetry telemetry;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime totalTime = new ElapsedTime();
    private String te_Stat = "Not-Collected";
    private int[] pixyThresholds = new int[2];
    private int[] pixyDuckThresholds = new int[2];
    private int turretFinalPos = 0;

    // Other
    private double extensionTime = 0.35;
    private static int acolor;


    //Pixy code values
    private double team_element_x;
    private double team_element_y;
    private double current_duck_x;
    private double current_duck_y;
    private double previous_duck_x;
    private boolean duckCollectStat = false;
    private  boolean see_duck = false;
    private double search_amount;


    //Constants
    private static final int PIXY_RED_THRESHOLD_LOW = 50;
    private static final int PIXY_RED_THRESHOLD_HIGH = 110;
    private static final int PIXY_BLUE_THRESHOLD_LOW = 200;
    private static final int PIXY_BLUE_THRESHOLD_HIGH = 135;

    private static final int EXTEND_TARGET_POSITION_TOP = -365;
    private static final int EXTEND_TARGET_POSITION_MID = -325;
    private static final int EXTEND_TARGET_POSITION_LOW = -273;

    private static final int LIFT_TARGET_POSITION_TOP = 1540;
    private static final int LIFT_TARGET_POSITION_MID = 1035;
    private static final int LIFT_TARGET_POSITION_LOW = 530;

    //Variable for target positions
    private int currentLiftTargetPosition;
    private int currentExtensionTargetPosition;

    //initialized for red, flipped to blue in constructor if needed
    private int turretTargetPosition = 100;
    private Pose2d startPosition = new Pose2d(-30.25, -63.75, Math.toRadians(-90.0));
    private Pose2d depositPreload = new Pose2d(startPosition.getX(), -48.5, Math.toRadians(0));
    private Pose2d closeToCarousel = new Pose2d(-55.0, -56.0, Math.toRadians(-32.5));
    private static double StrafeAmount = 6;
    private Pose2d collectingDuck1 = new Pose2d(-50.0, -54.0, Math.toRadians(-90.0));
    private Pose2d collectingDuck2 = new Pose2d(-52.0, -46.0, Math.toRadians(-90.0));
    private Pose2d depositDuck = new Pose2d(-33.0, -27.0, Math.toRadians(0));
    private Pose2d parkAtEnd1 = new Pose2d(-39.25, -46.5, Math.toRadians(0));
    private Pose2d parkAtEnd2 = new Pose2d(-62.5, -35.0, Math.toRadians(0));
    private static double swodpower = -0.15;



    public DuckAuto(AllianceColor color, LinearOpMode opMode) {
        this.color = color;
        this.opMode = opMode;
        telemetry = opMode.telemetry;
        switch (color) {
            case RED:
                pixyThresholds[0] = PIXY_RED_THRESHOLD_LOW;
                pixyThresholds[1] = PIXY_RED_THRESHOLD_HIGH;
                acolor = 0;
                search_amount = 2.5;
                break;
            case BLUE:
                pixyThresholds[0] = PIXY_BLUE_THRESHOLD_LOW;
                pixyThresholds[1] = PIXY_BLUE_THRESHOLD_HIGH;
                turretTargetPosition *= -1;
                startPosition = new Pose2d(startPosition.getX(), -startPosition.getY(), startPosition.getHeading());
                depositPreload = new Pose2d(depositPreload.getX(), -depositPreload.getY(), depositPreload.getHeading() + Math.toRadians(180));
                closeToCarousel = new Pose2d(-56.0, 60.5, closeToCarousel.getHeading() - Math.toRadians(45));
                StrafeAmount = 3.5;
                collectingDuck1 = new Pose2d(collectingDuck1.getX(), -collectingDuck1.getY(), collectingDuck1.getHeading());
                collectingDuck2 = new Pose2d(collectingDuck2.getX(), -collectingDuck2.getY(),collectingDuck2.getHeading());
                depositDuck = new Pose2d(depositDuck.getX(), -depositDuck.getY(), depositDuck.getHeading() + Math.toRadians(180));
                parkAtEnd1 = new Pose2d(parkAtEnd1.getX(), 44.5, parkAtEnd1.getHeading() + Math.toRadians(180));
                parkAtEnd2 = new Pose2d(parkAtEnd2.getX() -(1), -parkAtEnd2.getY(),parkAtEnd2.getHeading() + Math.toRadians(180));
                swodpower = 0.1;
                acolor = 1;
                turretFinalPos = -680;
                search_amount = 2.25;
                break;
        }
    }

    public void run() throws InterruptedException {
        robot = new Robot(opMode.hardwareMap, opMode.telemetry, opMode);
        SampleMecanumDrive drive = new SampleMecanumDrive(opMode.hardwareMap);
        robot.pixyCam.engage();

        robot.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.extention.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.extention.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Robot is ready.");
        telemetry.update();

        robot.pixyCam.disengage();
        robot.pixyCam.engage();

        while(!opMode.opModeIsActive()) {

            team_element_x = 0xff & robot.pixyCam.read(0x51, 5)[1];
            team_element_y = 0xff & robot.pixyCam.read(0x51, 5)[2];


            if (acolor == 0) {
                if (team_element_x == 0 || team_element_x > pixyThresholds[1]) {
                    //detecting top
                    height = LiftHeight.TOP;
                    currentLiftTargetPosition = LIFT_TARGET_POSITION_TOP;
                    currentExtensionTargetPosition = EXTEND_TARGET_POSITION_TOP;
                    Log.d("Laptop", "Red Top");
                } else if (team_element_x < pixyThresholds[0]) {
                    //detecting low
                    height = LiftHeight.LOW;
                    currentLiftTargetPosition = LIFT_TARGET_POSITION_LOW;
                    currentExtensionTargetPosition = EXTEND_TARGET_POSITION_LOW;
                    Log.d("Laptop", "Red Low");

                } else {
                    //detecting mid
                    height = LiftHeight.MID;
                    currentLiftTargetPosition = LIFT_TARGET_POSITION_MID;
                    currentExtensionTargetPosition = EXTEND_TARGET_POSITION_MID;
                    Log.d("Laptop", "Red Mid");


                }
            } if (acolor == 1) {
                if (team_element_x == 0 || team_element_x > pixyThresholds[0]) {
                    //detecting top
                    height = LiftHeight.TOP;
                    currentLiftTargetPosition = LIFT_TARGET_POSITION_TOP;
                    currentExtensionTargetPosition = EXTEND_TARGET_POSITION_TOP;
                    Log.d("Laptop", "Blue Top");

                } else if (team_element_x < pixyThresholds[1]) {
                    //detecting low
                    height = LiftHeight.LOW;
                    currentLiftTargetPosition = LIFT_TARGET_POSITION_LOW;
                    currentExtensionTargetPosition = EXTEND_TARGET_POSITION_LOW;
                    Log.d("Laptop", "Blue Low");


                } else {
                    //detecting mid
                    height = LiftHeight.MID;
                    currentLiftTargetPosition = LIFT_TARGET_POSITION_MID;
                    currentExtensionTargetPosition = EXTEND_TARGET_POSITION_MID;
                    Log.d("Laptop", "Blue Mid");


                }
            }

            telemetry.addLine();
            //telemetry.addData("Pixy Health :", robot.pixyCam.getHealthStatus());
            telemetry.addLine();
            //telemetry.addData("Pixy Connection : ", robot.pixyCam.getConnectionInfo());
            telemetry.addLine();
            telemetry.addData("Completion Status : ", "Innit");
            telemetry.addLine();
            telemetry.addData("Completion Status : ", "Innit");
            telemetry.addData("Lift height" ,height.toString());
            telemetry.addData("team element", team_element_x);
            telemetry.update();

        }


        opMode.waitForStart();

        drive.setPoseEstimate(startPosition);
         totalTime.reset();
        //move towards hub
        Trajectory depositPreloadTrajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineTo(depositPreload.vec())
                .build();
        drive.followTrajectoryAsync(depositPreloadTrajectory);
        //will happen while moving: tilt, lift, turn turret
        //robot.LAup();


        lift_up();
        runtime.reset();
        sleep(400);
        boolean firstTime = true;
        while(drive.isBusy() || firstTime) {
            if (firstTime && runtime.seconds() > 1) {
                turret_turn_deposit_preload();
                firstTime = false;
            }
            drive.update();
        }
        drive.waitForIdle();

        runtime.reset();
        extend_to_target(currentExtensionTargetPosition);
        //wait for extension to reach position (or timeout)
        while(runtime.seconds() < 0.7 && opMode.opModeIsActive());

        //deposit preload block
        robot.collector.setPower(0.4); // minimum depositing speed
        sleep(500);
        robot.collector.setPower(0);

        //start extending in
        runtime.reset();
        extension_in();

        //wait for extension to come back (or timeout)
        while(robot.extention.isBusy() && runtime.seconds() < .2 && opMode.opModeIsActive());

        //extend in slowly to hold it in
        //extension_in_slow();

        //move close to carousel
        Trajectory closeToCarouselTrajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(closeToCarousel, SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(65))
                .build();
        drive.followTrajectoryAsync(closeToCarouselTrajectory);
        lift_barriers();
        runtime.reset();

        if (acolor == 0) {
            turret_back();
        }
        if (acolor == 1){
            turret_duck();
        }
        while(runtime.seconds() < 1.069420);
        lift_down();

        runtime.reset();

        //cLimit works for both colors
        while(runtime.seconds() < 1.5 && opMode.opModeIsActive()) {
            drive.update();
        }
        drive.waitForIdle();


        //slowly move to press wheel against carousel
        Trajectory pressWheelAgainstCarousel = drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeRight(StrafeAmount,
                        SampleMecanumDrive.getVelocityConstraint(7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(7))
                .build();
        drive.followTrajectory(pressWheelAgainstCarousel);

        robot.SWOD(swodpower);
        sleep(5750);
        robot.SWOD(0);




        Trajectory collectingTheDuck = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(collectingDuck2)
                .build();
        drive.followTrajectoryAsync(collectingTheDuck);

        //  extension out here
        runtime.reset();
        while(!robot.cLimit.getState() && runtime.seconds() < 1.5 && opMode.opModeIsActive()) {
            drive.update();
        }
        drive.waitForIdle();



        runtime.reset();
        while (runtime.seconds() < 6.5) {
            for (int i = 0; i < 50; i++) {
                robot.pixyCam.engage();
                current_duck_x = 0xff & robot.pixyCam.read(0x52, 5)[1];
                current_duck_y = 0xff & robot.pixyCam.read(0x52, 5)[2];
                telemetry.addData("Loop  : ", i);
                telemetry.addData("PIXY-D-C-X :", current_duck_x);
                telemetry.addData("PIXY-D-P-X :", previous_duck_x);
                telemetry.addData("PIXY STAT : ", robot.pixyCam.getHealthStatus());
                telemetry.addData("TE-STAT :", te_Stat);
                telemetry.addData("Duck Collection Status (true means it goes to the second position)", duckCollectStat);
                telemetry.update();

                if (acolor == 0) {
                    while (current_duck_x == 0 && runtime.seconds() < search_amount) {
                        drive.turn(Math.toRadians(-7.5));
                        sleep(25);
                        Log.d("BrainSTEM", "Finding the duck red");
                    }
                }

                if (acolor == 1){
                    while (current_duck_x == 0 && runtime.seconds() < search_amount){
                        drive.turn(Math.toRadians(6));
                        sleep(25);
                        Log.d("BrainSTEM 17895", "Finding the duck blue");
                    }
                }

                if (current_duck_x != 0){
                    see_duck = true;
                }

                if (current_duck_x < 110 && current_duck_x != 0) { // move right
                    drive.turn(Math.toRadians(5.5));
                    telemetry.addLine("Moving Right");
                    telemetry.update();
                    Log.d("BrainSTEM 17895", "Adjusting Right");
                }
                if (current_duck_x > 130 && current_duck_x != 0) { // move left
                    drive.turn(Math.toRadians(-5.5));
                    telemetry.addLine("Moving Left");
                    telemetry.update();
                    Log.d("BrainSTEM 17895", "Adjusting Left");
                }
                if ((current_duck_x <= 130) && (current_duck_x >= 110) && (previous_duck_x == current_duck_x)){
                    telemetry.addLine("Collected");
                    telemetry.update();
                    robot.collector.setPower(-1);
                    robot.extention.setTargetPosition(-374);
                    robot.extention.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    runtime.reset();
                    robot.extention.setPower(0.4);
                    while (runtime.seconds() < 1.75) ;
                    robot.extention.setTargetPosition(0);
                    robot.extention.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.extention.setPower(0.75);
                    runtime.reset();
                    sleep(10);
                    duckCollectStat = true;
                    Log.d("BrainSTEM 17895", "picking up the duck");
                    i = 99;
                }
                previous_duck_x = current_duck_x;
                telemetry.update();
            }

            break;

        }







        if (duckCollectStat) {
            robot.collector.setPower(-1);
            Trajectory depositTheDuck = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(depositDuck, SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(25))
                    .build();
            drive.followTrajectoryAsync(depositTheDuck);

            move_lift(1560);
            runtime.reset();
            while (!robot.cLimit.getState() && runtime.seconds() < 1.5 && opMode.opModeIsActive()) {
                drive.update();
            }
            drive.waitForIdle();


            robot.extention.setTargetPosition(-165);
            robot.extention.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.extention.setPower(0.75);
            runtime.reset();
            while (runtime.seconds() < extensionTime);
            robot.collector.setPower(0.3);
            while (runtime.seconds() < 2 + extensionTime) ;
            robot.collector.setPower(0);

        }



        Trajectory parkingAtTheEnd = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(parkAtEnd1,
                        SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(65))
                .build();
        drive.followTrajectoryAsync(parkingAtTheEnd);

        robot.extention.setTargetPosition(0);
        robot.extention.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extention.setPower(0.75);

        runtime.reset();
        move_lift(1000);
        while (runtime.seconds() < 0.65 && (robot.lift.getCurrentPosition() > 600));
        runtime.reset();
        move_turret(turretFinalPos);

        telemetry.addLine("Lift Down");
        runtime.reset();
        while(!robot.cLimit.getState() && runtime.seconds() < .75 && opMode.opModeIsActive()) {
            drive.update();
        }
        drive.waitForIdle();

        lift_barriers();
          while (totalTime.seconds() < 26.75) {
                sleep(50);
          }
              if (acolor == 0) {
                  robot.setMotorPowers(.95, 1, .95, 1);
                  sleep(1690);
                  robot.stop();
              }
              if (acolor == 1) {
                  robot.setMotorPowers(-1, -.95, -1, -.95);
                  sleep(1690);
                  robot.stop();
              }

        if (acolor == 1) {
            Trajectory finaltraj = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .strafeRight(10)
                    .build();
            drive.followTrajectoryAsync(finaltraj);
        }
        if (acolor == 0) {
            Trajectory finaltraj0 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .strafeLeft(10)
                    .build();
            drive.followTrajectoryAsync(finaltraj0);
        }


        lift_down();





    }









    //Extension
    private void extension_in() {
        //uses encoders
        robot.extention.setTargetPosition(0);
        robot.extention.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extention.setPower(0.7);
    }

    private void extension_in_slow() {
        //does not use encoders
        robot.extention.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.extention.setPower(0.15);
    }

    public void extend_to_target(int target_position){
        robot.extention.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.extention.setTargetPosition(target_position);
        robot.extention.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.extention.setPower(-.9);
    }

    //Turret
    private void turret_turn_deposit_preload(){
        robot.turret.setTargetPosition(turretTargetPosition);
        robot.turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.turret.setPower(0.85);
    }
    private void turret_duck() {
        robot.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.turret.setTargetPosition(-685);
        robot.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.turret.setPower(0.85);

    }

    private void turret_back(){
        robot.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.turret.setPower(1);
    }

    private void move_turret(int position){
        robot.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.turret.setTargetPosition(position);
        robot.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.turret.setPower(1);
    }

    //Lift
    private void lift_up(){
        robot.lift.setTargetPosition(currentLiftTargetPosition);
        robot.lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(1);
    }

    private void lift_barriers(){
        robot.lift.setTargetPosition(LIFT_TARGET_POSITION_LOW);
        robot.lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(1);

    }

    private void lift_down(){
        robot.lift.setTargetPosition(0);
        robot.lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(1);
    }

    private void move_lift(int encoder_value){
        robot.lift.setTargetPosition(encoder_value);
        robot.lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(1);
    }
}