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

    //Pixy x/y values
    private double team_element_x;
    private double team_element_y;
    private double duck_x;
    private double duck_y;

    //Constants
    private static final int PIXY_RED_THRESHOLD_LOW = 175;
    private static final int PIXY_RED_THRESHOLD_HIGH = 190;

    private static final int PIXY_BLUE_THRESHOLD_LOW = 185;
    private static final int PIXY_BLUE_THRESHOLD_HIGH = 190;

    private static final int EXTEND_TARGET_POSITION_TOP = -380;
    private static final int EXTEND_TARGET_POSITION_MID = -304;
    private static final int EXTEND_TARGET_POSITION_LOW = -269;

    private static final int LIFT_TARGET_POSITION_TOP = 1585;
    private static final int LIFT_TARGET_POSITION_MID = 1000;
    private static final int LIFT_TARGET_POSITION_LOW = 516;

    //Variable for target positions
    private int currentLiftTargetPosition;
    private int currentExtensionTargetPosition;

    //initialized for red, flipped to blue in constructor if needed
    private int turretTargetPosition = 115;
    private Pose2d startPosition = new Pose2d(-30.25, -63.75, Math.toRadians(-90.0));
    private Pose2d depositPreload = new Pose2d(startPosition.getX(), -50.5, Math.toRadians(-90.0));
    private Pose2d closeToCarousel = new Pose2d(-65.0, -47.0, Math.toRadians(0));

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
                startPosition = new Pose2d(startPosition.getX(), -startPosition.getY(), startPosition.getHeading() + Math.toRadians(180));
                depositPreload = new Pose2d(depositPreload.getX(), -depositPreload.getY(), depositPreload.getHeading() + Math.toRadians(180));
                closeToCarousel = new Pose2d(closeToCarousel.getX(), -closeToCarousel.getY(), closeToCarousel.getHeading());
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
            team_element_x = 0xff & robot.pixyCam.read(0x52, 5)[1];
            team_element_y = 0xff & robot.pixyCam.read(0x52, 5)[2];
            telemetry.addLine();
            telemetry.addData("Pixy Health :", robot.pixyCam.getHealthStatus());
            telemetry.addLine();
            telemetry.addData("Pixy Connection : ", robot.pixyCam.getConnectionInfo());
            telemetry.addLine();
            telemetry.addData("Completion Status : ", "Innit");

//            if (team_element_x == 0 || team_element_x > pixyThresholds[1]) {
//                //detecting top
                height = LiftHeight.TOP;
                currentLiftTargetPosition = LIFT_TARGET_POSITION_TOP;
                currentExtensionTargetPosition = EXTEND_TARGET_POSITION_TOP;
//            } else if (team_element_x < pixyThresholds[0]) {
//                //detecting low
//                height = LiftHeight.LOW;
//                currentLiftTargetPosition = LIFT_TARGET_POSITION_LOW;
//                currentExtensionTargetPosition = EXTEND_TARGET_POSITION_LOW;
//            } else {
//               //detecting mid
//                height = LiftHeight.MID;
//                currentLiftTargetPosition = LIFT_TARGET_POSITION_MID;
//                currentExtensionTargetPosition = EXTEND_TARGET_POSITION_MID;
//            }

            telemetry.addLine(height.toString());
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

        //after reaching deposit pose: wait for lift to reach position (or timeout)
//        while(robot.lift.isBusy() && runtime.seconds() < 0.3 && opMode.opModeIsActive());

        //start extending out
        runtime.reset();
        extend_to_target(currentExtensionTargetPosition);

        //wait for extension to reach position (or timeout)
        while(robot.extention.isBusy() && runtime.seconds() < 1.5 && opMode.opModeIsActive());

        //deposit preload block
        robot.collector.setPower(1); // minimum depositing speed
        sleep(500);
        robot.collector.setPower(0);

        //start extending in
        runtime.reset();
        extension_in();

        //wait for extension to come back (or timeout)
        while(robot.extention.isBusy() && runtime.seconds() < 1 && opMode.opModeIsActive());

        //extend in slowly to hold it in
        extension_in_slow();

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

        while(opMode.opModeIsActive());

        //slowly move to press wheel against carousel
        Trajectory pressWheelAgainstCarousel = drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeRight(8,
                        SampleMecanumDrive.getVelocityConstraint(7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(7))
                .build();

        drive.followTrajectory(pressWheelAgainstCarousel);

        robot.SWOD(-0.15);
        sleep(4700);
        robot.SWOD(0);

//        Trajectory builder3 = drive.trajectoryBuilder(drive.getPoseEstimate())
//                .forward(5)
//                .build();
//        drive.followTrajectory(builder3);
//
//        Trajectory builder3a = drive.trajectoryBuilder(drive.getPoseEstimate())
//                .strafeLeft(19)
//                .build();
//        drive.followTrajectory(builder3a);
//
//        Trajectory builder4 = drive.trajectoryBuilder(drive.getPoseEstimate())
//                .back(2)
//                .build();
//        drive.followTrajectory(builder4);
//
//        lift_barriers();
//        sleep(400);
//        turret_withexpanlimit();
//        sleep(500);
//        lift_down();

/*

        drive.turn(Math.toRadians(-90));


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



        Trajectory builder4 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeRight(10)
                .build();
        drive.followTrajectory(builder4);

        sleep(7);
        extension_in();


        Trajectory builder5 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .back(39)
                .build();
        drive.followTrajectory(builder5);

        Trajectory builder5a = drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeLeft(6)
                .build();
        drive.followTrajectory(builder5a);

        drive.turn(Math.toRadians(-93));

        lift_barries2();

        Trajectory builder6a = drive.trajectoryBuilder(drive.getPoseEstimate())
                .back(60)
                .build();
        drive.followTrajectory(builder6a);



        Trajectory builder8 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeLeft(27
                        , SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .build();
        drive.followTrajectory(builder8);


        Trajectory builder9 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .back(35,
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        drive.followTrajectory(builder9);

 */



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
        robot.extention.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.extention.setTargetPosition(target_position);
        robot.extention.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        runtime.reset();
        while (robot.extention.isBusy() && runtime.milliseconds() < 600){
            robot.extention.setPower(-0.65);
        }
        robot.extention.setPower(0);
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

    //Linear Actuator
    private void linear_actuator_down(){
        //robot.linearActuator.setPosition(0.79682527);
    }
}
