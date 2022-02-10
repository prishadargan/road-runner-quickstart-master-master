package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Autonomous(name="17895 - RR Test", group="SummerCamp")
public class AutonomousCode extends LinearOpMode {
    Robot robot;
    /*
    public double team_element_x;
    public double team_element_y;
    public double duck_x;
    public double duck_y;
    public String te_Stat = "Not-Collected";

     */
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    // right turn = +, -, +, -
    // left turn = -, +, -, +


    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap, telemetry, this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        telemetry.addLine("Robot is ready.");
        telemetry.update();
        waitForStart();

        robot.extention.setPower(-.1);
        sleep(250);
        robot.extention.setPower(-0);


        drive.setPoseEstimate(new Pose2d(-32.0, 65.0, Math.toRadians(-90.0)));

        Trajectory builder1 = drive.trajectoryBuilder(new Pose2d())

                .forward(6)
                .build();


        drive.followTrajectory(builder1);


        Trajectory builder2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeRight(30)
            .build();


        drive.followTrajectory(builder2);

        Trajectory builder2x = drive.trajectoryBuilder(drive.getPoseEstimate())
                .back(4)
                .build();


        drive.followTrajectory(builder2x);

        Trajectory builder3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(35)
                .build();

        drive.followTrajectory(builder3);





        /*

        sleep(500);
        robot.backLeft.setPower(1);
        sleep(1000);
        robot.backLeft.setPower(0);
        sleep(250);
        robot.backRight.setPower(1);
        sleep(1000);
        robot.backRight.setPower(0);
        sleep(250);
        robot.frontLeft.setPower(1);
        sleep(1000);
        robot.frontLeft.setPower(0);
        sleep(250);
        robot.frontRight.setPower(1);
        sleep(1000);
        robot.frontRight.setPower(0);

         */



        /*
        robot.pixyCam.engage();
        duck_x = 0xff&robot.pixyCam.read(0x52,5)[1];
        duck_y = 0xff&robot.pixyCam.read(0x52,5)[2];
        for (int i = 0; i < 10000000; i++) {
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
                    robot.lSErvo(0);
                    te_Stat = "Collected";
                    telemetry.addData("TE-STAT :", te_Stat);
                } else {
                    telemetry.addData("TE-STAT :", te_Stat);
                }
                telemetry.update();
            }



            sleep(500);
        }

         */



        





    }
    // right turn = +, -, +, -
    // left turn = -, +, -, +


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

