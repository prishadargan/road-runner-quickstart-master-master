package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import static java.lang.Thread.sleep;

@Autonomous(name="17895 - RR Test", group="SummerCamp")
public class AutonomousCode extends LinearOpMode {
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
        robot.extention.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.extention.setPower(-.1);
        sleep(250);
        robot.extention.setPower(-0);
        robot.extention.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.extention.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setPoseEstimate(new Pose2d(-32.0, 65.0, Math.toRadians(-90.0)));
// 4

        team_element_x = 0xff&robot.pixyCam.read(0x51,5)[1];
        team_element_y = 0xff&robot.pixyCam.read(0x51,5)[2];
        for (int i = 0; i < 13; i++) {
            sleep(100);
            if ((team_element_x < 100) && team_element_x > 70) {
                liftHeight = "low";

            } else if ((team_element_x > 110) && team_element_x < 160) {
                liftHeight = "mid";
            } else {
                liftHeight = "top";
            }
        }

        if (liftHeight == "low") {
            robot.state = Robot.states.LIFT_MIDO;
        } else if (liftHeight == "mid") {
            robot.state = Robot.states.LIFT_MIDO;
            robot.state = Robot.states.LIFT_FAU;
            robot.state = Robot.states.LIFT_FAU;
        } else {
            robot.state = Robot.states.LIFTING_UP;
        }

        robot.update();

// 1

        Trajectory builder1 = drive.trajectoryBuilder(new Pose2d())

                .forward(9)
                .build();


        drive.followTrajectory(builder1);


        Trajectory builder2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-65.0, 57, Math.toRadians(-90)))
                .build();


        drive.followTrajectory(builder2);

        Trajectory builder2a = drive.trajectoryBuilder(drive.getPoseEstimate())
                .back(4)
                .build();

        drive.followTrajectory(builder2a);

        sleep(50);
        robot.SWOD(0.15);
        sleep(2300);
        robot.SWOD(0);



        Trajectory builder3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(20)
                .build();

        drive.followTrajectory(builder3);

// 3
        turn_right(0.3,1100);

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

        Trajectory builder3b = drive.trajectoryBuilder(drive.getPoseEstimate())
                .back(25)
                .build();

        drive.followTrajectory(builder3b);


        turn_right(0.4,400);
        move_backwards(-0.1, 1500);


        Trajectory builder4a = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(23)
                .build();

        drive.followTrajectory(builder4a);


        if (liftHeight == "low") {
            robot.state = Robot.states.LIFT_MIDO;
        } else if (liftHeight == "mid") {
            robot.state = Robot.states.LIFT_MIDO;
            robot.state = Robot.states.LIFT_FAU;
            robot.state = Robot.states.LIFT_FAU;
        } else {
            robot.state = Robot.states.LIFTING_UP;
        }

        robot.update();
// 2


        Trajectory builder4b = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(40)
                .build();

        drive.followTrajectoryAsync(builder4b);
        robot.state = Robot.states.LIFT_MIDO;
        while (drive.isBusy()) {
            robot.update();
        }


        Trajectory builder5 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeLeft(40)
                .build();

        drive.followTrajectory(builder5);



        Trajectory builder6 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(35)
                .build();

        drive.followTrajectory(builder6);



    }
    private void encodeForward (int distance, double power) {

        robot.frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setTargetPosition((distance));
        robot.backLeft.setTargetPosition((distance ));
        robot.frontRight.setTargetPosition((distance));
        robot.backRight.setTargetPosition((distance));
        robot.frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.frontRight.setPower((power));
        robot.backRight.setPower((power));
        robot.frontLeft.setPower(((power)));
        robot.backLeft.setPower(((power)));

    }
    private void move_straight(double speed, long time) {
        robot.setMotorPowers(-speed, -speed, -speed, -speed);
        sleep(time);
        robot.stop();
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
    private void strafe_right(double speed, long time){
        robot.setMotorPowers(-speed, speed, speed, -speed);
        sleep(time);
        robot.stop();
    }
    private void strafe_left(double speed, long ms){
        robot.setMotorPowers(speed, -speed, -speed, speed);
        sleep(ms);
        robot.stop();
    }

}