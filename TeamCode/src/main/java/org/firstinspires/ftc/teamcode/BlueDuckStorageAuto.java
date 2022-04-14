package org.firstinspires.ftc.teamcode;




import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Blue Duck Storage Auto")
public class BlueDuckStorageAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DuckStorageAuto duckStorageAuto = new DuckStorageAuto(DuckStorageAuto.AllianceColor.BLUE, this);
        duckStorageAuto.run();
    }
}
