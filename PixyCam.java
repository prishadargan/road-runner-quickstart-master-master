package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;

public class PixyCam extends I2cDeviceSynchDevice<I2cDeviceSynch>
{
    @Override
    public Manufacturer getManufacturer()
    {

        return Manufacturer.Adafruit;
    }

    @Override
    protected synchronized boolean doInitialize()
    {
        return true;
    }

    @Override
    public String getDeviceName()
    {

        return "17895 PixyCam";
    }

    public PixyCam(I2cDeviceSynch deviceClient)
    {
        super(deviceClient, true);
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    @I2cSensor(name = "MCP9808 Temperature Sensor", description = "PixyCam", xmlTag = "PixyCam")
    public class MCP9808 extends I2cDeviceSynchDevice<I2cDeviceSynch>
}
