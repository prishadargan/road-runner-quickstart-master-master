package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="New Blue Cycle Auto")
public class BlueCycleAuto2 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        WharehouseAuto2 auto = new WharehouseAuto2(WharehouseAuto2.AllianceColor.BLUE, this);
        auto.run();
    }
}
