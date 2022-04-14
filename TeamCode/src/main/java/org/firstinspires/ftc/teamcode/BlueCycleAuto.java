package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Blue Cycle Auto")
public class BlueCycleAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        WharehouseAuto auto = new WharehouseAuto(WharehouseAuto.AllianceColor.BLUE, this);
        auto.run();
    }
}
