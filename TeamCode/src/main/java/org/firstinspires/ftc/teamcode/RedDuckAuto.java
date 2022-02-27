package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Red Ducks Auto")
public class RedDuckAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DuckAuto auto = new DuckAuto(DuckAuto.AllianceColor.RED);
        auto.runOpMode();
    }
}