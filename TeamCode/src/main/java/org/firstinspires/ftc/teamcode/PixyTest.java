package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

@Autonomous
public class PixyTest extends LinearOpMode {
    private  double duck_x;
    private double duck_y;
    private double teameX;

    public I2cDeviceSynch pixyCam;
    @Override

    public void runOpMode() throws InterruptedException {
        /*
        Robot robot = new Robot(hardwareMap, telemetry, this);


        robot.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.extention.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.extention.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


         */

        pixyCam = this.hardwareMap.i2cDeviceSynch.get("Pixy-Cam"); //

        pixyCam.engage();


        telemetry.addLine("Ready for start");
        telemetry.update();


        waitForStart();

        while(opModeIsActive()) {

            duck_x = 0xff & pixyCam.read(0x52, 5)[1];
            duck_y = 0xff & pixyCam.read(0x52, 5)[2];
            telemetry.addData("Pixy Health : ", pixyCam.getHealthStatus());
            telemetry.addLine(" " + duck_x);
            telemetry.addLine(" " + duck_y);
            telemetry.update();


// think about it logically
        }
    }
}
