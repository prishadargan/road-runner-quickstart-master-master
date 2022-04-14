package org.firstinspires.ftc.teamcode;




import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Red Duck Storage Auto")
public class RedDuckStorageAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DuckStorageAuto duckStorageAuto = new DuckStorageAuto(DuckStorageAuto.AllianceColor.RED, this);
        duckStorageAuto.run();
    }
}
