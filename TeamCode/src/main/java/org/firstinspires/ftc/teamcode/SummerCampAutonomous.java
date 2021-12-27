package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// https://learnroadrunner.com/quickstart-overview.html#drivefeedforwardtuner 
@Autonomous(name="SummerCampAutonomous", group="SummerCamp")
public class SummerCampAutonomous extends LinearOpMode {
    Robot robot;



    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, this);
        waitForStart();
        // Put code that you want to run here, in chronological order.

        move_straight(.5, 1);
        turn_right(.5, 1);
        move_straight(.5, 1);
        move_straight(.5, 1);
        turn_right(.5, 1);
        move_straight(.5, 1);


    }

    private void move_straight(double speed, double time) {
        robot.setMotorPowers(speed, speed, speed, speed);
        sleep(time);
        robot.stop();
    }

    private void turn_left(double speed, double time) {
        robot.setMotorPowers(speed, -speed, speed, -speed);
        sleep(time);
        robot.stop();

    }

    private void turn_right(double speed, long time) {
        robot.setMotorPowers(-speed, speed, -speed, speed);
        sleep(time);
        robot.stop();
    }

    private void move_backwards(double speed, long time) {
        robot.setMotorPowers(-speed, -speed, -speed, -speed);
        sleep(time);
        robot.stop();
    }

}