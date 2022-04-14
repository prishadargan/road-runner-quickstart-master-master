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
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class WharehouseAuto {
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
    private ElapsedTime totaltime = new ElapsedTime();
    private String te_Stat = "Not-Collected";
    private int[] pixyThresholds = new int[2];
    private int[] pixyDuckThresholds = new int[2];
    private int turretFinalPos = -10;

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
    private boolean firstTime;


    //Constants-
    private static final int PIXY_RED_THRESHOLD_LOW = 135;
    private static final int PIXY_RED_THRESHOLD_HIGH = 195;

    private static final int PIXY_BLUE_THRESHOLD_LOW = 100;
    private static final int PIXY_BLUE_THRESHOLD_HIGH = 40;

    private static final int EXTEND_TARGET_POSITION_TOP = -275;
    private static final int EXTEND_TARGET_POSITION_MID = -230;
    private static final int EXTEND_TARGET_POSITION_LOW = -190;

    private static final int LIFT_TARGET_POSITION_TOP = 1540;
    private static final int LIFT_TARGET_POSITION_MID = 870;
    private static final int LIFT_TARGET_POSITION_LOW = 500;

    //Variable for target positions
    private int currentLiftTargetPosition;
    private int currentExtensionTargetPosition;

    //initialized for red, flipped to blue in constructor if needed
    private int turretTargetPosition = -580;
    private Pose2d startPosition = new Pose2d(4, -64, Math.toRadians(90.0));
    private Pose2d depositPreload = new Pose2d(7.5, -37.0, Math.toRadians(0));
    private Pose2d startCycle1 = new Pose2d(6.5, -67, Math.toRadians(0));
    private Pose2d startCycle2 = new Pose2d(6.5, -67, Math.toRadians(0));
    private Pose2d startCycle3 = new Pose2d(6.5, -67, Math.toRadians(0));
    private Pose2d collectingCycle1 = new Pose2d(42.0, -67.0, Math.toRadians(0));
    private Pose2d collectingCycle2 = new Pose2d(42.0, -67.0, Math.toRadians(0));
    private Pose2d collectingCycle3 = new Pose2d(42.0, -67.0, Math.toRadians(0));




    public WharehouseAuto(AllianceColor color, LinearOpMode opMode) {
        this.color = color;
        this.opMode = opMode;
        telemetry = opMode.telemetry;
        switch (color) {
            case RED:
                pixyThresholds[0] = PIXY_RED_THRESHOLD_LOW;
                pixyThresholds[1] = PIXY_RED_THRESHOLD_HIGH;
                acolor = 0;

                break;
            case BLUE:
                    final int EXTEND_TARGET_POSITION_TOP = -240;
                pixyThresholds[0] = PIXY_BLUE_THRESHOLD_LOW;
                pixyThresholds[1] = PIXY_BLUE_THRESHOLD_HIGH;
                turretTargetPosition = 555;
                startPosition = new Pose2d(6.5, -startPosition.getY(), startPosition.getHeading());
                depositPreload = new Pose2d(7.5, -depositPreload.getY(), depositPreload.getHeading() + Math.toRadians(180));
                startCycle1 = new Pose2d(4, -startCycle1.getY(), startCycle1.getHeading() + Math.toRadians(180));
                startCycle2 = new Pose2d(4, -startCycle2.getY(), startCycle2.getHeading() + Math.toRadians(180));
                startCycle3 = new Pose2d(4, -startCycle3.getY(), startCycle3.getHeading() + Math.toRadians(180));
                collectingCycle1 = new Pose2d(42.0, -collectingCycle1.getY(), collectingCycle1.getHeading() + Math.toRadians(180));
                collectingCycle2 = new Pose2d(42.0, -collectingCycle2.getY(), collectingCycle2.getHeading() + Math.toRadians(180));
                collectingCycle3 = new Pose2d(42.0, -collectingCycle3.getY(), collectingCycle3.getHeading() + Math.toRadians(180));
                acolor = 1;
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


        // init loop
        while (!opMode.opModeIsActive()) {

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


            }
            if (acolor == 1) {
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
            telemetry.addData("Completion Status : ", "Innit");
            telemetry.addData("Completion Status : ", "Innit");
            telemetry.addData("Lift height", height.toString());
            telemetry.addData("team element", team_element_x);
            telemetry.update();

        }


        // start of auto
        opMode.waitForStart();

        drive.setPoseEstimate(startPosition);

        totaltime.reset();
        telemetry.addData("Collector Current Draw (mA) ", robot.collector.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.update();
        //move towards hub
        Trajectory depositPreloadTrajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(depositPreload,
                        SampleMecanumDrive.getVelocityConstraint(75,
                                DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(75))
                .build();
        drive.followTrajectoryAsync(depositPreloadTrajectory);

        // move the lift up for the pre-load
        move_lift(currentLiftTargetPosition);
        runtime.reset();
        firstTime = true;
        sleep(250);


        // turret for pre-load
        while (drive.isBusy() || firstTime) {
            if (firstTime && runtime.seconds() > 0.75) {
                turret_turn_deposit_pos();
                firstTime = false;
            }

            drive.update();
        }
        drive.waitForIdle();

        // move the extension
        runtime.reset();
        sleep(50);
        extend_to_target(currentExtensionTargetPosition);

        //wait for extension to reach position (or timeout)
        while (runtime.seconds() < 0.3 && opMode.opModeIsActive()) ;

        //deposit preload block
        robot.collector.setPower(0.35); // minimum depositing speed
        sleep(600);
        robot.collector.setPower(0);

        //start extending in
        runtime.reset();
        extension_in();
        move_lift(750);

        //going to the start position of cycle 1
        Trajectory cycle1Start = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(startCycle1,
                        SampleMecanumDrive.getVelocityConstraint(75,
                                DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(75))
                .build();
        drive.followTrajectoryAsync(cycle1Start);

        // reset the turret and lift
        move_turret(0);
        while (runtime.seconds() < 1) ;
        lift_down();
        while (!robot.cLimit.getState() && runtime.seconds() < 1.2 && opMode.opModeIsActive()) {
            drive.update();
        }
        drive.waitForIdle();


        Trajectory strafeonWall1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeRight(4, SampleMecanumDrive.getVelocityConstraint(75,
                        DriveConstants.MAX_ANG_VEL,
                        DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(75))
                .build();
        drive.followTrajectoryAsync(strafeonWall1);


        //cycle 1

        Trajectory driveInWareHouse1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(collectingCycle1,
                        SampleMecanumDrive.getVelocityConstraint(75,
                                DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(75))
                .build();
        drive.followTrajectoryAsync(driveInWareHouse1);


        // collection
        robot.collector.setPower(-1);
        extend_to_target(-215);
        runtime.reset();
        while (!robot.cLimit.getState() && runtime.seconds() < 0.4 && opMode.opModeIsActive()) {
            drive.update();
        }
        drive.waitForIdle();


        // current draw for collecting cargo in warehouse
        telemetry.addData("Collector Current Draw (mA) ", robot.collector.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.update();
        runtime.reset();
        while ((robot.collector.getCurrent(CurrentUnit.MILLIAMPS) < 1849) && runtime.seconds() < 0.7) {
            telemetry.update();
        }
        while (robot.collector.getCurrent(CurrentUnit.MILLIAMPS) < 1849) {
            drive.turn(Math.toRadians(10));
            extend_to_target(robot.extention.getCurrentPosition() - 50);
            sleep(100);
            telemetry.addLine("Current Draw is below 1000");
            telemetry.update();
        }
        sleep(100);
        telemetry.addLine("collected");
        telemetry.update();
        if (acolor == 0) {
            Trajectory goBacktoCheck = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .back(5)
                    .build();
            drive.followTrajectoryAsync(goBacktoCheck);

        }

        if (acolor == 1) {
            Trajectory goBacktoCheck = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .forward(5)
                    .build();
            drive.followTrajectoryAsync(goBacktoCheck);
        }
        if (acolor == 0) {

            if (robot.collector.getCurrent(CurrentUnit.MILLIAMPS) < 1849) {
                Trajectory adjustToGettheBlock = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .forward(8.5)
                        .build();
                drive.followTrajectoryAsync(adjustToGettheBlock);
            }
        }
        if (acolor == 1) {
            if (robot.collector.getCurrent(CurrentUnit.MILLIAMPS) < 1849) {
                Trajectory adjustToGettheBlock = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .back(8.5)
                        .build();
                drive.followTrajectoryAsync(adjustToGettheBlock);
            }
        }
        // exit the warehouse
        Trajectory leaveWarehouse1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(startCycle1,
                        SampleMecanumDrive.getVelocityConstraint(75,
                                DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(75))
                .build();
        drive.followTrajectoryAsync(leaveWarehouse1);
        robot.collector.setPower(-1);
        extend_to_target(25);
        lift_warehouse();
        extend_to_target(0);
        turret_turn_deposit_pos();
        while (!robot.cLimit.getState() && runtime.seconds() < 0.6 && opMode.opModeIsActive()) {
            drive.update();
        }
        drive.waitForIdle();


        // move to deposit position (same as preload pos)
        Trajectory depositCycle1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(depositPreload,
                        SampleMecanumDrive.getVelocityConstraint(60,
                                DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(60))
                .build();
        drive.followTrajectoryAsync(depositCycle1);
        runtime.reset();

        // raise lift
        move_lift(1580);
        runtime.reset();
        firstTime = true;
        sleep(250);

        while (drive.isBusy() || firstTime) {
            if (firstTime && robot.lift.getCurrentPosition() > 650 && runtime.seconds() > 0.75) {
                turret_turn_deposit_pos();
                firstTime = false;


            }


            drive.update();
        }
        drive.waitForIdle();


        //extension
        runtime.reset();
        extend_to_target(-275);
        //wait for extension to reach position (or timeout)
        while (runtime.seconds() < 0.3 && opMode.opModeIsActive()) ;


        //deposit collected block
        robot.collector.setPower(0.35); // minimum depositing speed
        sleep(600);
        robot.collector.setPower(0);
        extension_in();
        move_lift(750);

        // cycle 1 end (go to cycle 2 start pos)
        Trajectory cycle2Start = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(startCycle2,
                        SampleMecanumDrive.getVelocityConstraint(75,
                                DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(75))
                .build();
        drive.followTrajectoryAsync(cycle2Start);
        move_turret(0);
        while (runtime.seconds() < .7) ;
        lift_down();
        while (!robot.cLimit.getState() && runtime.seconds() < 1 && opMode.opModeIsActive()) {
            drive.update();
        }
        drive.waitForIdle();

        Trajectory strafeonWall = drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeRight(4, SampleMecanumDrive.getVelocityConstraint(75,
                        DriveConstants.MAX_ANG_VEL,
                        DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(75))
                .build();
        drive.followTrajectoryAsync(strafeonWall);


        // cycle 2

        Trajectory driveIntoWareHouse2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(collectingCycle2,
                        SampleMecanumDrive.getVelocityConstraint(75,
                                DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(75))
                .build();
        drive.followTrajectoryAsync(driveIntoWareHouse2);


        //collection
        robot.collector.setPower(-1);
        extend_to_target(-300);
        runtime.reset();
        while (!robot.cLimit.getState() && runtime.seconds() < 0.6 && opMode.opModeIsActive()) {
            drive.update();
        }
        drive.waitForIdle();

        if (totaltime.seconds() > 23) {
            while (opMode.opModeIsActive()) ;
            // end of auto with 2 cycles
        }

        // current draw for collecting cargo in warehouse
        telemetry.addData("Collector Current Draw (mA) ", robot.collector.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.update();
        runtime.reset();
        while ((robot.collector.getCurrent(CurrentUnit.MILLIAMPS) < 1849) && runtime.seconds() < 0.5) {
            telemetry.update();
        }
        while (robot.collector.getCurrent(CurrentUnit.MILLIAMPS) < 1849) {
            drive.turn(Math.toRadians(20));
            extend_to_target(robot.extention.getCurrentPosition() - 75);
            sleep(500);
            telemetry.addLine("Current Draw is below 1000");
            telemetry.update();
        }

        sleep(500);
        telemetry.addLine("collected");
        telemetry.update();


        //exit warehouse
        Trajectory leaveWareHouse2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(startCycle1,
                        SampleMecanumDrive.getVelocityConstraint(75,
                                DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(75))
                .build();
        robot.collector.setPower(-1);
        drive.followTrajectoryAsync(leaveWareHouse2);
        extend_to_target(25);
        lift_warehouse();
        extend_to_target(0);
        turret_turn_deposit_pos();
        while (!robot.cLimit.getState() && runtime.seconds() < .6 && opMode.opModeIsActive()) {
            drive.update();
        }
        drive.waitForIdle();

        //go to depositing location (same as preload pos)
        Trajectory depositCycle2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(depositPreload,
                        SampleMecanumDrive.getVelocityConstraint(65,
                                DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(65))
                .build();
        drive.followTrajectoryAsync(depositCycle2);
        runtime.reset();

        //depositing
        move_lift(1580);
        while (runtime.seconds() < 0.75) ;
        turret_turn_deposit_pos();
        drive.waitForIdle();
        runtime.reset();
        extend_to_target(-325);
        while (runtime.seconds() < 0.3) ;
        robot.collector.setPower(0.35);
        sleep(600);
        robot.collector.setPower(0);
        extend_to_target(0);
        move_lift(750);


        //go to cycle 3 start pos
        Trajectory cycle3StartTrajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(startCycle3,
                        SampleMecanumDrive.getVelocityConstraint(75,
                                DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(75))
                .build();
        drive.followTrajectoryAsync(cycle3StartTrajectory);
        move_turret(0);
        while (runtime.seconds() < .7) ;
        lift_down();
        while (!robot.cLimit.getState() && runtime.seconds() < 1 && opMode.opModeIsActive()) {
            drive.update();
        }
        drive.waitForIdle();


        Trajectory strafeonWall3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeRight(6.5, SampleMecanumDrive.getVelocityConstraint(75,
                        DriveConstants.MAX_ANG_VEL,
                        DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(75))
                .build();
        drive.followTrajectoryAsync(strafeonWall3);


        //park if not enough time to do 3rd cycle


        if (totaltime.seconds() > 24.5) {
            Trajectory driveForward = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(collectingCycle3, SampleMecanumDrive.getVelocityConstraint(75, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(75))
                    .build();
            drive.followTrajectory(driveForward);
            while (opMode.opModeIsActive()) ;
            // end of auto with 2 cycles
        }

        //cycle 3

        Trajectory driveIntoWareHouse3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(collectingCycle3,
                        SampleMecanumDrive.getVelocityConstraint(75,
                                DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(75))
                .build();
        drive.followTrajectoryAsync(driveIntoWareHouse3);

        //collection
        robot.collector.setPower(-1);
        extend_to_target(-300);
        runtime.reset();
        while (!robot.cLimit.getState() && runtime.seconds() < 0.45 && opMode.opModeIsActive()) {
            drive.update();
        }
        drive.waitForIdle();

        // current draw for collecting cargo in warehouse
        telemetry.addData("Collector Current Draw (mA) ", robot.collector.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.update();
        runtime.reset();
        while ((robot.collector.getCurrent(CurrentUnit.MILLIAMPS) < 1849) && runtime.seconds() < 0.25) {
            telemetry.update();
        }
        while (robot.collector.getCurrent(CurrentUnit.MILLIAMPS) < 1849) {
            drive.turn(Math.toRadians(5));
            extend_to_target(robot.extention.getCurrentPosition() - 50);
            sleep(500);
            telemetry.addLine("Current Draw is below 1000");
            telemetry.update();
        }

        sleep(500);


        //exit WareHouse
        Trajectory leaveWareHouse3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(startCycle1,
                        SampleMecanumDrive.getVelocityConstraint(75,
                                DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(75))
                .build();
        drive.followTrajectoryAsync(leaveWareHouse3);
        robot.collector.setPower(-1);
        extend_to_target(25);
        lift_warehouse();
        extend_to_target(0);
        turret_turn_deposit_pos();

        while (!robot.cLimit.getState() && runtime.seconds() < .6 && opMode.opModeIsActive()) {
            drive.update();
        }
        drive.waitForIdle();


        //go to depositing pos (same as preload pos)
        Trajectory depositblock3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(depositPreload,
                        SampleMecanumDrive.getVelocityConstraint(60,
                                DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(60))
                .build();
        drive.followTrajectoryAsync(depositblock3);
        runtime.reset();
        move_lift(1560);
        while (runtime.seconds() < 0.75) ;
        if(acolor == 0) {
            move_turret(-570);
        }
        if(acolor == 1) {
            move_turret(570);
        }

        drive.waitForIdle();

        //depositing
        runtime.reset();
        extend_to_target(-300);
        while (runtime.seconds() < 0.3) ;
        robot.collector.setPower(0.35);
        sleep(400);
        robot.collector.setPower(0);
        extension_in();
        move_lift(750);
      //  move_turret(-340);

        //parking
        if(acolor == 0) {
            robot.setMotorPowers(1, 1, 1, 1);
            sleep(1000);
            robot.stop();
        }
        if(acolor == 1) {
            robot.setMotorPowers(-1, -1, -1, -1);
            sleep(1000);
            robot.stop();
        }
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
        robot.extention.setPower(-1);
    }

    //Turret
    private void turret_turn_deposit_pos(){
        robot.turret.setTargetPosition(turretTargetPosition);
        robot.turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.turret.setPower(1);
    }
    private void turret_duck() {
        robot.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.turret.setTargetPosition(-685);
        robot.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.turret.setPower(0.5);

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

    private void lift_warehouse(){
        robot.lift.setTargetPosition(LIFT_TARGET_POSITION_LOW + 100);
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
