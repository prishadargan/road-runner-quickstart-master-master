package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="CampAutonomous", group="SummerCamp")
public class AutonomousCode extends LinearOpMode {
    Robot robot;
    @Override
    // right turn = +, -, +, -
    // left turn = -, +, -, +
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, this);
        waitForStart();

       while ( true ) {

           move_straight(.25, 500);
           robot.spinningWheelofDeath(1);
       }






    }
    // right turn = +, -, +, -
    // left turn = -, +, -, +
    private void move_straight(double speed, long time) {
        robot.setMotorPowers(speed, speed, speed, speed);
        sleep(time);
        robot.stop();
    }
    private void turn_left(double speed, long time) {
        robot.setMotorPowers(speed, -speed, speed, -speed);
        sleep(time);
        robot.stop();

    }
    private void turn_right(double speed, long time) {
        robot.setMotorPowers(-speed, speed, -speed, speed);
        sleep(time);
        robot.stop();
    }
    private void move_backwards(double speed, long time){
        robot.setMotorPowers(-speed, -speed, -speed, -speed);
        sleep(time);
        robot.stop();
    }
    private void strafe_right(double speed, long time){
        robot.setMotorPowers(speed, -speed, -speed, speed);
        sleep(time);
        robot.stop();
    }
    private void strafe_left(double speed, long time){
        robot.setMotorPowers(-speed, speed, speed, -speed);
        sleep(time);
        robot.stop();
    }
    private void testcode(double power, long time, long sleeptime) {
        move_backwards(power, time);
        sleep(sleeptime);
        move_straight(power, time);
        robot.stop();
    }
}

