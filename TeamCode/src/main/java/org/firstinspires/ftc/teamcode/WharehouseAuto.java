package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.robotcore.internal.android.dx.dex.code.StdCatchBuilder.build;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

    private static final int PIXY_BLUE_THRESHOLD_LOW = 225;
    private static final int PIXY_BLUE_THRESHOLD_HIGH = 135;

    private static final int EXTEND_TARGET_POSITION_TOP = -300;
    private static final int EXTEND_TARGET_POSITION_MID = -255;
    private static final int EXTEND_TARGET_POSITION_LOW = -240;

    private static final int LIFT_TARGET_POSITION_TOP = 1585;
    private static final int LIFT_TARGET_POSITION_MID = 850;
    private static final int LIFT_TARGET_POSITION_LOW = 276;

    //Variable for target positions
    private int currentLiftTargetPosition;
    private int currentExtensionTargetPosition;

    //initialized for red, flipped to blue in constructor if needed
    private int turretTargetPosition = -560;
    private Pose2d startPosition = new Pose2d(6.5, -64.5, Math.toRadians(90.0));
    private Pose2d depositPreload = new Pose2d(10.0, -44.5, Math.toRadians(0));
    private Pose2d startCycle1 = new Pose2d(6.5, -73.0, Math.toRadians(0));
    private Pose2d startCycle2 = new Pose2d(6.5, -75, Math.toRadians(0));
    private Pose2d startCycle3 = new Pose2d(6.5, -76.5, Math.toRadians(0));
    private Pose2d collectingCycle1 = new Pose2d(42.0, -70.0, Math.toRadians(0));
    private Pose2d collectingCycle2 = new Pose2d(42.0, -72, Math.toRadians(0));
    private Pose2d collectingCycle3 = new Pose2d(45.0, -74, Math.toRadians(0));
    private Pose2d collectingCycle4 = new Pose2d(42.0, -76, Math.toRadians(0));
    private Pose2d collectingDuck2 = new Pose2d(-52.0, -46.0, Math.toRadians(-90.0));
    private Pose2d depositDuck = new Pose2d(-33.0, -25.0, Math.toRadians(0));
    private Pose2d parkAtEnd1 = new Pose2d(-39.25, -13.5, Math.toRadians(0));
    private Pose2d parkAtEnd2 = new Pose2d(-62.5, -35.0, Math.toRadians(0));
    private static double swodpower = -0.15;



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
                pixyThresholds[0] = PIXY_BLUE_THRESHOLD_LOW;
                pixyThresholds[1] = PIXY_BLUE_THRESHOLD_HIGH;
                turretTargetPosition *= -1;
                startPosition = new Pose2d(startPosition.getX(), -startPosition.getY(), startPosition.getHeading());
                depositPreload = new Pose2d(depositPreload.getX(), -depositPreload.getY(), depositPreload.getHeading() + Math.toRadians(180));
                startCycle1 = new Pose2d(6.5, -66.0, startCycle1.getHeading() - Math.toRadians(45));
                collectingCycle1 = new Pose2d(collectingCycle1.getX(), -collectingCycle1.getY(), collectingCycle1.getHeading());
                collectingDuck2 = new Pose2d(collectingDuck2.getX(), -collectingDuck2.getY(),collectingDuck2.getHeading());
                depositDuck = new Pose2d(depositDuck.getX(), -depositDuck.getY(), depositDuck.getHeading() + Math.toRadians(180));
                parkAtEnd1 = new Pose2d(parkAtEnd1.getX(), -parkAtEnd1.getY(), parkAtEnd1.getHeading() + Math.toRadians(180));
                parkAtEnd2 = new Pose2d(parkAtEnd2.getX() -(1), -parkAtEnd2.getY(),parkAtEnd2.getHeading() + Math.toRadians(180));
                swodpower = 0.15;
                acolor = 1;
                turretFinalPos = -680;
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
            telemetry.addData("Completion Status : ", "Innit");
            telemetry.addData("Completion Status : ", "Innit");
            telemetry.addData("Lift height" ,height.toString());
            telemetry.addData("team element", team_element_x);
            telemetry.update();

        }


        opMode.waitForStart();

        drive.setPoseEstimate(startPosition);

        //move towards hub
        Trajectory depositPreloadTrajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(depositPreload,SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(55))
                .build();
        drive.followTrajectoryAsync(depositPreloadTrajectory);

        move_lift(1550);
        runtime.reset();
        firstTime = true;
        sleep(250);

        while(drive.isBusy() || firstTime) {
            if (firstTime && robot.lift.getCurrentPosition() > 650 && runtime.seconds() > 0.75) {
                turret_turn_deposit_preload();
               // extend_to_target(-50);
                firstTime = false;
            }



            drive.update();
        }
        drive.waitForIdle();
        runtime.reset();
        sleep(50);
        extend_to_target(-300);
        //wait for extension to reach position (or timeout)
        while(runtime.seconds() < 0.3 && opMode.opModeIsActive());

        //deposit preload block

        robot.collector.setPower(0.5); // minimum depositing speed
        sleep(600);
        robot.collector.setPower(0);

        //start extending in
        runtime.reset();
        extension_in();
        move_lift(500);

        //wait for extension to come back (or timeout)




        //going to the start position of cycle 1
        Trajectory cycle1Start = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(startCycle1,SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(50))
                .build();
        drive.followTrajectoryAsync(cycle1Start);
        move_turret(0);
        while(runtime.seconds() < 0.8);
        lift_down();
        while(!robot.cLimit.getState() && runtime.seconds() < 1.5 && opMode.opModeIsActive()) {
            drive.update();
        }
        drive.waitForIdle();




// cycle 1
        move_lift(75);
        Trajectory strafeToGetCycle1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(collectingCycle1,SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .build();
        drive.followTrajectoryAsync(strafeToGetCycle1);

        robot.collector.setPower(-1);
        extend_to_target(-200);
        runtime.reset();
        while(!robot.cLimit.getState() && runtime.seconds() < 0.6 && opMode.opModeIsActive()) {
            drive.update();
        }
        drive.waitForIdle();



        Trajectory depositCycle1Trajectory1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(startCycle1,SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(50))
                .build();
        drive.followTrajectoryAsync(depositCycle1Trajectory1);
        extend_to_target(0);
        while(!robot.cLimit.getState() && runtime.seconds() < 0.75 && opMode.opModeIsActive()) {
            drive.update();
        }
        drive.waitForIdle();



        Trajectory depositCycle1Trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(depositPreload,SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(55))
                .build();
        drive.followTrajectoryAsync(depositCycle1Trajectory);
        runtime.reset();
        move_lift(1550);
        runtime.reset();
        firstTime = true;
        sleep(250);

        while(drive.isBusy() || firstTime) {
            if (firstTime && robot.lift.getCurrentPosition() > 650 && runtime.seconds() > 0.75) {
                turret_turn_deposit_preload();
                //extend_to_target(-50);
                firstTime = false;
            }



            drive.update();
        }
        drive.waitForIdle();

        runtime.reset();
        runtime.reset();
        sleep(100);
        extend_to_target(-300);
        //wait for extension to reach position (or timeout)
        while(runtime.seconds() < 0.3 && opMode.opModeIsActive());

        //deposit preload block

        robot.collector.setPower(0.4); // minimum depositing speed
        sleep(600);
        robot.collector.setPower(0);
        extension_in();
        move_lift(500);

       /* move_lift(1450);
        while (runtime.seconds() < 0.5);
        turret_turn_deposit_preload();
        drive.waitForIdle();

        runtime.reset();
        extend_to_target(-325);
        robot.collector.setPower(0.5);
        while (runtime.seconds() < 0.5);
        extend_to_target(0);
        move_turret(0);
        while (runtime.seconds() < 1.5);
        lift_down();
*/
        Trajectory cycle2Start = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(startCycle2,SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(50))
                .build();
        drive.followTrajectoryAsync(cycle2Start);
        move_turret(0);
        lift_down();
        while(!robot.cLimit.getState() && runtime.seconds() < 1.25 && opMode.opModeIsActive()) {
            drive.update();
        }
        drive.waitForIdle();


        // cycle 2
        move_lift(75);
        Trajectory strafeToGetCycle2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(collectingCycle2, SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .build();
        drive.followTrajectoryAsync(strafeToGetCycle2);

        robot.collector.setPower(-1);
        extend_to_target(-300);
        runtime.reset();
        while(!robot.cLimit.getState() && runtime.seconds() < 0.6 && opMode.opModeIsActive()) {
            drive.update();
        }
        drive.waitForIdle();





        Trajectory depositCycle2Trajectory2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(startCycle1,  SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(50))
                .build();
        drive.followTrajectoryAsync(depositCycle2Trajectory2);
        extend_to_target(0);
        while(!robot.cLimit.getState() && runtime.seconds() < 1.5 && opMode.opModeIsActive()) {
            drive.update();
        }
        drive.waitForIdle();


        Trajectory depositCycle2Trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(depositPreload, SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(55))
                .build();
        drive.followTrajectoryAsync(depositCycle2Trajectory);
        runtime.reset();
        move_lift(1565);
        while (runtime.seconds() < 0.75);
        turret_turn_deposit_preload();
        drive.waitForIdle();

        runtime.reset();
        extend_to_target(-300);
        while (runtime.seconds() < 0.3);
        robot.collector.setPower(0.45);
       sleep(600);
       robot.collector.setPower(0);
        extend_to_target(0);
        move_lift(500);
        Trajectory cycle3StartTrajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(startCycle3, SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(50))
                .build();
        drive.followTrajectoryAsync(cycle3StartTrajectory);
        move_turret(0);
        while(runtime.seconds() < 0.4);
        lift_down();
        while(!robot.cLimit.getState() && runtime.seconds() < 1.25 && opMode.opModeIsActive()) {
            drive.update();
        }
        drive.waitForIdle();
        move_lift(75);
        Trajectory strafeToGetCycle3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(collectingCycle3, SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .build();
        drive.followTrajectoryAsync(strafeToGetCycle3);

        robot.collector.setPower(-1);
        extend_to_target(-300);
        runtime.reset();
        while(!robot.cLimit.getState() && runtime.seconds() < 0.45 && opMode.opModeIsActive()) {
            drive.update();
        }
        drive.waitForIdle();
        Trajectory depositCycle3Trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(startCycle1, SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(50))
                .build();
        drive.followTrajectoryAsync(depositCycle3Trajectory);
        extend_to_target(0);
        while(!robot.cLimit.getState() && runtime.seconds() < 1 && opMode.opModeIsActive()) {
            drive.update();
        }
        drive.waitForIdle();



        Trajectory depositCycle3Trajectory2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(depositPreload, SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(55))
                .build();
        drive.followTrajectoryAsync(depositCycle3Trajectory2);
        runtime.reset();
        move_lift(1560);
        //while (runtime.seconds() < 0.75);
        turret_turn_deposit_preload();
        drive.waitForIdle();

        runtime.reset();
        extend_to_target(-300);
        while (runtime.seconds() < 0.3);
        robot.collector.setPower(0.45);
        sleep(600);
        robot.collector.setPower(0);
        extension_in();
        move_lift(500);
        move_turret(-340);

        Trajectory driveForward = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(50, SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(55))
                .build();
        drive.followTrajectory(driveForward);

    }

    private void build() {
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

    private void lift_down(){
        robot.lift.setTargetPosition(0);
        robot.lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(1);
    }

    private void move_lift(int encoder_value){
        robot.lift.setTargetPosition(encoder_value);
        robot.lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(0.75);
    }



    //Linear Actuator
    private void linear_actuator_down(){
        //robot.linearActuator.setPosition(0.79682527);
    }
}