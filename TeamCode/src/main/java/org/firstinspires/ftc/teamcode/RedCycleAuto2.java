package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Red Cycle Auto 2")
public class RedCycleAuto2 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        WharehouseAuto2 auto = new WharehouseAuto2(WharehouseAuto2.AllianceColor.RED, this);
        auto.run();
    }
}
