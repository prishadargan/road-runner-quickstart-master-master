package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import static java.lang.Thread.sleep;

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
    private String te_Stat = "Not-Collected";
    private int[] pixyThresholds = new int[2];

    //Pixy code values
    private double team_element_x;
    private double team_element_y;
    private double current_duck_x;
    private double current_duck_y;
    private double previous_duck_x;
    private boolean duckCollectStat = false;
    private boolean see_duck = false;

    //Constants
    private static final int PIXY_RED_THRESHOLD_LOW = 35;
    private static final int PIXY_RED_THRESHOLD_HIGH = 115;

    private static final int PIXY_BLUE_THRESHOLD_LOW = 185;
    private static final int PIXY_BLUE_THRESHOLD_HIGH = 190;

    private static final int EXTEND_TARGET_POSITION_TOP = -365;
    private static final int EXTEND_TARGET_POSITION_MID = -335;
    private static final int EXTEND_TARGET_POSITION_LOW = -300;

    private static final int LIFT_TARGET_POSITION_TOP = 1520;
    private static final int LIFT_TARGET_POSITION_MID = 1050;
    private static final int LIFT_TARGET_POSITION_LOW = 520;

    //Variable for target positions
    private int currentLiftTargetPosition;
    private int currentExtensionTargetPosition;

    //initialized for red, flipped to blue in constructor if needed
    private int turretTargetPosition = 100;
    private Pose2d startPosition = new Pose2d(-30.25, -63.75, Math.toRadians(-90.0));
    private Pose2d depositPreload = new Pose2d(startPosition.getX(), -48.5, Math.toRadians(0));
    private Pose2d closeToCarousel = new Pose2d(-55.0, -56.0, Math.toRadians(-32.5));
    private Pose2d collectingDuck1 = new Pose2d(-50.0, -54.0, Math.toRadians(-90.0));
    private Pose2d collectingDuck2 = new Pose2d(-52.0, -46.0, Math.toRadians(-90.0));
    private Pose2d depositDuck = new Pose2d(-33.0, -25.0, Math.toRadians(0));
    private Pose2d parkAtEnd = new Pose2d(-60.0, -35.0, Math.toRadians(0));



    public DuckAuto(AllianceColor color, LinearOpMode opMode) {
        this.color = color;
        this.opMode = opMode;
        telemetry = opMode.telemetry;
        switch (color) {
            case RED:
                pixyThresholds[0] = PIXY_RED_THRESHOLD_LOW;
                pixyThresholds[1] = PIXY_RED_THRESHOLD_HIGH;
                break;
            case BLUE:
                pixyThresholds[0] = PIXY_BLUE_THRESHOLD_LOW;
                pixyThresholds[1] = PIXY_BLUE_THRESHOLD_HIGH;
                turretTargetPosition *= -1;
                startPosition = new Pose2d(startPosition.getX(), -startPosition.getY(), startPosition.getHeading());
                depositPreload = new Pose2d(depositPreload.getX(), -depositPreload.getY(), depositPreload.getHeading() + Math.toRadians(180));
                closeToCarousel = new Pose2d(-53.25, 55.5, depositPreload.getHeading() + Math.toRadians(180));
                collectingDuck1 = new Pose2d(collectingDuck1.getX(), -collectingDuck1.getY(), collectingDuck1.getHeading());
                collectingDuck2 = new Pose2d(collectingDuck2.getX(), -collectingDuck2.getY(),collectingDuck2.getHeading());
                depositDuck = new Pose2d(depositDuck.getX(), -depositDuck.getY(), depositDuck.getHeading());
                parkAtEnd = new Pose2d(depositDuck.getX(), -depositDuck.getY(), depositDuck.getHeading() + Math.toRadians(180));


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

        while(!opMode.opModeIsActive()) {
            team_element_x = 0xff & robot.pixyCam.read(0x51, 5)[1];
            team_element_y = 0xff & robot.pixyCam.read(0x51, 5)[2];


            if (team_element_x == 0 || team_element_x > pixyThresholds[1]) {
                //detecting top
                height = LiftHeight.TOP;
                currentLiftTargetPosition = LIFT_TARGET_POSITION_TOP;
                currentExtensionTargetPosition = EXTEND_TARGET_POSITION_TOP;
            } else if (team_element_x < pixyThresholds[0]) {
                //detecting low
                height = LiftHeight.LOW;
                currentLiftTargetPosition = LIFT_TARGET_POSITION_LOW;
                currentExtensionTargetPosition = EXTEND_TARGET_POSITION_LOW;
            } else {
              //detecting mid
                height = LiftHeight.MID;
                currentLiftTargetPosition = LIFT_TARGET_POSITION_MID;
                currentExtensionTargetPosition = EXTEND_TARGET_POSITION_MID;

            }

            telemetry.addLine();
            telemetry.addData("Pixy Health :", robot.pixyCam.getHealthStatus());
            telemetry.addLine();
            telemetry.addData("Pixy Connection : ", robot.pixyCam.getConnectionInfo());
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

        //move towards hub
        Trajectory depositPreloadTrajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineTo(depositPreload.vec())
                .build();
        drive.followTrajectoryAsync(depositPreloadTrajectory);
        //will happen while moving: tilt, lift, turn turret
        //robot.LAup();


        lift_up();
        runtime.reset();
        sleep(100);
        boolean firstTime = true;
        while(drive.isBusy() || firstTime) {
            if (firstTime && runtime.seconds() > 0.8) {
                turret_turn_deposit_preload();
                firstTime = false;
            }
            drive.update();
        }
        drive.waitForIdle();

        runtime.reset();
        extend_to_target(currentExtensionTargetPosition);
        //wait for extension to reach position (or timeout)
        while((robot.extention.isBusy() && runtime.seconds() < 0.8) && opMode.opModeIsActive());

        //deposit preload block
        robot.collector.setPower(0.65); // minimum depositing speed
        sleep(500);
        robot.collector.setPower(0);

        //start extending in
        runtime.reset();
        extension_in();

        //wait for extension to come back (or timeout)
        while(robot.extention.isBusy() && runtime.seconds() < 1 && opMode.opModeIsActive());

        //extend in slowly to hold it in
        //extension_in_slow();

        //move close to carousel
        Trajectory closeToCarouselTrajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(closeToCarousel)
                .build();
        drive.followTrajectoryAsync(closeToCarouselTrajectory);
        linear_actuator_down();
        lift_barriers();
        turret_back();
        runtime.reset();
        //cLimit works for both colors
        while(!robot.cLimit.getState() && runtime.seconds() < 1.5 && opMode.opModeIsActive()) {
            drive.update();
        }
        drive.waitForIdle();


        //slowly move to press wheel against carousel
        Trajectory pressWheelAgainstCarousel = drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeRight(5.75,
                        SampleMecanumDrive.getVelocityConstraint(7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(7))
                .build();

        drive.followTrajectory(pressWheelAgainstCarousel);

        robot.SWOD(-0.15);
        sleep(5000);
        robot.SWOD(0);



        Trajectory collectingTheDuck = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(collectingDuck2)
                .build();
        drive.followTrajectoryAsync(collectingTheDuck);

        move_lift(0);
        //  extension out here
        runtime.reset();
        while(!robot.cLimit.getState() && runtime.seconds() < 4 && opMode.opModeIsActive()) {
            drive.update();
        }
        drive.waitForIdle();



        runtime.reset();
        while (runtime.seconds() < 10)
        for (int i = 0; i < 12; i++) {
            robot.pixyCam.engage();
            current_duck_x = 0xff & robot.pixyCam.read(0x52, 5)[1];
            current_duck_y = 0xff & robot.pixyCam.read(0x52, 5)[2];
            telemetry.addData("Variant : ", i);
            telemetry.addData("PIXY-D-C-X :", current_duck_x);
            telemetry.addData("PIXY-D-P-X :", previous_duck_x);
            telemetry.addData("PIXY-D-Y : ", current_duck_y);
            telemetry.addData("PIXY STAT : ", robot.pixyCam.getHealthStatus());
            telemetry.addData("TE-STAT :", te_Stat);
            telemetry.addData("Duck Collection Status (true means it goes to the second position)", duckCollectStat);
            telemetry.update();
            runtime.reset();
            while (current_duck_x == 0 && runtime.seconds() < 0.5){
                drive.turn(Math.toRadians(-20));
                see_duck = true;
            }



            if (current_duck_x < 125 && current_duck_x != 0) { // move right
                drive.turn(Math.toRadians(10));
                telemetry.addLine("Moving Right");
                telemetry.update();
            }
            if (current_duck_x > 145 && current_duck_x != 0) { // move left
                drive.turn(Math.toRadians(-10));
                telemetry.addLine("Moving Left");
                telemetry.update();
            }
            if (current_duck_x < 145 && current_duck_x > 125 && previous_duck_x == current_duck_x && see_duck == true) {
                telemetry.addLine("Collected");
                telemetry.update();
                i = 99;
                robot.collector.setPower(-1);
                robot.extention.setTargetPosition(-374);
                robot.extention.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                runtime.reset();
                robot.extention.setPower(0.4);
                while (runtime.seconds() < 1.75);
                robot.extention.setTargetPosition(0);
                robot.extention.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.extention.setPower(0.75);
                runtime.reset();
                sleep(250);
                duckCollectStat = true;

            }

            sleep(100);
            previous_duck_x = current_duck_x;

            telemetry.update();
        }

/*
        if (duckCollectStat == true) {
            telemetry.addLine("Second time!");
            telemetry.update();
            Trajectory duckCollect2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(collectingDuck2)
                    .build();
            drive.followTrajectory(duckCollect2);

            runtime.reset();
            while (runtime.seconds() < 3);
            telemetry.addLine("Second time!");
            telemetry.update();
            for (int i = 0; i < 10; i++) {
                    robot.pixyCam.engage();
                    duck_x = 0xff & robot.pixyCam.read(0x52, 5)[1];
                    duck_y = 0xff & robot.pixyCam.read(0x52, 5)[2];
                    if (duck_x == 0){
                        i = 99;
                    }
                    telemetry.addData("Variant : ", i);
                    telemetry.addData("PIXY-D-X :", duck_x);
                    telemetry.addData("PIXY-D-Y : ", duck_y);
                    telemetry.addData("PIXY STAT : ", robot.pixyCam.getHealthStatus());
                    telemetry.addData("TE-STAT :", te_Stat);
                    telemetry.addData("Duck Collection Status (true means it goes to second position)", duckCollectStat);

                telemetry.update();


                    if (duck_x < 125 && duck_x != 0) { // move right
                        Trajectory adjustemntsForCollectingDuckLeft = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .strafeLeft(2,
                                        SampleMecanumDrive.getVelocityConstraint(11, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(11))
                                .build();

                        drive.followTrajectory(adjustemntsForCollectingDuckLeft);
                        telemetry.addLine("Moving Right");
                        telemetry.update();
                    }
                    if (duck_x > 145 && duck_x != 0) { // move left
                        Trajectory adjustemntsForCollectingDuckRight = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .strafeRight(2,
                                        SampleMecanumDrive.getVelocityConstraint(11, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(11))
                                .build();
                        drive.followTrajectory(adjustemntsForCollectingDuckRight);

                        telemetry.addLine("Moving Left");
                        telemetry.update();



                    }
                    if (duck_x < 145 && duck_x > 125) {
                        telemetry.addLine("Collected");
                        telemetry.update();
                        robot.collector.setPower(-0.6);
                        robot.extention.setTargetPosition(-380);
                        robot.extention.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        runtime.reset();
                        robot.extention.setPower(0.4);
                        while (runtime.seconds() < 1.75);
                        robot.extention.setTargetPosition(0);
                        robot.extention.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.extention.setPower(0.75);
                        runtime.reset();
                        sleep(250);
                        i = 26;
                    }
                    telemetry.update();
                }
        }


 */

        if (duckCollectStat == true && see_duck == true) {

            Trajectory depositTheDuck = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(depositDuck)
                    .build();
            drive.followTrajectoryAsync(depositTheDuck);

            move_lift(1580);
            runtime.reset();
            while (!robot.cLimit.getState() && runtime.seconds() < 4 && opMode.opModeIsActive()) {
                drive.update();
            }
            drive.waitForIdle();

            robot.extention.setTargetPosition(-100);
            robot.extention.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.extention.setPower(0.75);
            runtime.reset();

            robot.collector.setPower(0.75);
            while (runtime.seconds() < 2) ;
            robot.collector.setPower(0);

        }




        Trajectory parkingAtTheEnd = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(parkAtEnd)
                .build();
        drive.followTrajectoryAsync(parkingAtTheEnd);

        robot.extention.setTargetPosition(0);
        robot.extention.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extention.setPower(0.75);

        runtime.reset();
        telemetry.addLine("Moving  Turret");
        move_turret(-50);
        while (robot.turret.isBusy() && runtime.seconds() < 1 && opMode.opModeIsActive()){
            telemetry.addLine("In while loop");
        }
        lift_down();
        telemetry.addLine("Lift Down");
        runtime.reset();
        while(!robot.cLimit.getState() && runtime.seconds() < 2 && opMode.opModeIsActive()) {
            drive.update();
        }
        drive.waitForIdle();





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
        robot.extention.setPower(-0.65);
    }

    //Turret
    private void turret_turn_deposit_preload(){
        robot.turret.setTargetPosition(turretTargetPosition);
        robot.turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.turret.setPower(0.6);
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



    //Linear Actuator
    private void linear_actuator_down(){
        //robot.linearActuator.setPosition(0.79682527);
    }
}
