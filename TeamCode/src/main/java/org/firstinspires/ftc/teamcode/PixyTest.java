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

@Autonomous(name="Pixy-test", group="SummerCamp")
public class PixyTest extends LinearOpMode {
    Robot robot;
    private double team_element_x;
    private double team_element_y;
    private double duck_x;
    private double duck_y;
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


        /*
        robot.pixyCam.engage();
        team_element_x = 0xff&robot.pixyCam.read(0x51,5)[1];
        team_element_y = 0xff&robot.pixyCam.read(0x51,5)[2];
        for (int i = 0; i < 1000; i++) {
            sleep(100);
            telemetry.addData("Team Element X ", team_element_x);
            telemetry.update();
        }

         */


        while (true) {
            robot.pixyCam.engage();
            team_element_x = 0xff & robot.pixyCam.read(0x51, 5)[1];
            team_element_y = 0xff & robot.pixyCam.read(0x51, 5)[2];
            duck_x = 0xff & robot.pixyCam.read(0x52, 5)[1];
            duck_y = 0xff & robot.pixyCam.read(0x52, 5)[2];
            telemetry.addLine();
            telemetry.addData("Pixy Health :", robot.pixyCam.getHealthStatus());
            telemetry.addLine();
            telemetry.addData("Team Element X ", team_element_x);
            telemetry.addLine();
            telemetry.addData("Team Element Y ", team_element_y);
            telemetry.addLine();
            telemetry.addData("Duck X ", duck_x);
            telemetry.addLine();
            telemetry.addData("Duck Y ", duck_y);
            telemetry.update();
        }






    }
}