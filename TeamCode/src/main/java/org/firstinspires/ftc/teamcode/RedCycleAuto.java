package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Red Cycle Auto")
public class RedCycleAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        WharehouseAuto auto = new WharehouseAuto(WharehouseAuto.AllianceColor.RED, this);
        auto.run();
    }
}