package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
public class SubsystemTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, telemetry, this);
        boolean test = false;


        StickyButton gamepada = new StickyButton();
        StickyButton gamepadx = new StickyButton();
        StickyButton gamepady = new StickyButton();


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



            gamepady.update(gamepad1.y);
            if(gamepady.getState()){
                Log.d("BrainSTEM 17895", "Gamepad 1 - y");
            }


            gamepada.update(gamepad1.a);
            if(gamepada.getState()){
                Log.d("BrainSTEM 17895", "Gamepad 2- A-1");
                robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.lift.setTargetPosition(680);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift.setPower(1);
                Log.d("BrainSTEM 17895", "Gamepad 2- A-2");
                test = true;
                Log.d("BrainSTEM 17895", "Gamepad 2- A-2.5");

            }


            /*
            if (test) {
                robot.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.turret.setTargetPosition(340);
                robot.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.turret.setPower(0.75);
                Log.d("BrainSTEM 17895", "Gamepad 2- A-3");
                test = false;
                Log.d("BrainSTEM 17895", "Gamepad 2- A-4");
            }

             */

            gamepadx.update(gamepad1.x);
            if(gamepadx.getState()){
                Log.d("BrainSTEM 17895", "Gamepad 2- X-1");
                robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.lift.setTargetPosition(0);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift.setPower(1);
                Log.d("BrainSTEM 17895", "Gamepad 2- X-2");
                test = true;
                Log.d("BrainSTEM 17895", "Gamepad 2- X-2.5");

            }


        }


    }


}
