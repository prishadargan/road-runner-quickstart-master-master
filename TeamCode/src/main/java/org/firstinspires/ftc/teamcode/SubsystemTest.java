package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
public class SubsystemTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, telemetry, this);

        robot.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.extention.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.extention.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        telemetry.addLine("Ready for start");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {


            telemetry.addData("Lift encoder", robot.lift.getCurrentPosition());
            telemetry.addData("Turret encoder", robot.turret.getCurrentPosition());
            telemetry.addData("Extension encoder", robot.extention.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("Control Hub Limit (Blue) is pressed?", robot.cLimit.getState());
            telemetry.addData("left_stick_x", gamepad1.left_stick_x);
            telemetry.addData("left_stick_y", gamepad1.left_stick_y);
            telemetry.addLine();
            telemetry.addData("Collector Current", robot.collector.getCurrent(CurrentUnit.MILLIAMPS));

            telemetry.addData("Expansion Hub Limit (Red) is pressed?", robot.expanLimit.getState());
            telemetry.update();
        }
    }
}
