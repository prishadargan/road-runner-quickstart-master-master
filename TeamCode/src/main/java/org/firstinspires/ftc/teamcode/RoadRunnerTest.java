package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import static java.lang.Thread.sleep;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous(name="Road Runner Test", group="SummerCamp")
public class RoadRunnerTest extends LinearOpMode {
    Robot robot;
    public double team_element_x;
    public double team_element_y;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap, telemetry, this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        while (!opModeIsActive()) {


        }

        drive.setPoseEstimate(new Pose2d(-32.0, 65.0, Math.toRadians(-90.0)));


        Trajectory builder1 = drive.trajectoryBuilder(new Pose2d())

                .forward(7)
                .build();
        drive.followTrajectory(builder1);

        Trajectory builder2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeRight(10)
                .build();


        drive.followTrajectory(builder2);


    }
}
