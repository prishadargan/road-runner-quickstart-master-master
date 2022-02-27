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

@Autonomous(name="Blue Ducks Auto")
public class BlueDuckAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DuckAuto auto = new DuckAuto(DuckAuto.AllianceColor.BLUE);
        auto.runOpMode();
    }
}