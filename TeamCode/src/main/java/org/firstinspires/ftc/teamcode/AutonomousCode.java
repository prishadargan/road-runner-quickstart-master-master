package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
link - https://learnroadrunner.com/quickstart-overview.html#are-you-using-drive-encoders
 */

@Autonomous(name="felipe code things", group="SummerCamp")
public class AutonomousCode extends LinearOpMode {
    Robot robot;
    private ElapsedTime runtime = new ElapsedTime();
    @Override

    public void runOpMode() throws InterruptedException {

        double[] p1 = {0, 0, 0, 0};
        double[] p2 = {0, 0, 0, 0};
        double[] p3 = {0, 0, 0, 0};

        robot = new Robot(hardwareMap, telemetry, this);
        waitForStart();
        robot.Encoders();



        robot.encoderDrive(1, 500, 1000, 5);




    }


    }




