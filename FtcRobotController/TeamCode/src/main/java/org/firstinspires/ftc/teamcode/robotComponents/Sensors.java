package org.firstinspires.ftc.teamcode.robotComponents;/* package RobotComponets.Sensors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import net.frogbots.skystone.drivers.FrogBNO055;
import net.frogbots.skystone.drivers.MaxSonarI2CXL;
import net.frogbots.skystone.hardware.RobotComponent;

public class Sensors implements RobotComponent
{
    public FrogBNO055 imu;
    public MaxSonarI2CXL rearSonar;
    public MaxSonarI2CXL leftSonar;
    public MaxSonarI2CXL rightSonar;
    public MaxSonarI2CXL frontSonarLeftSide;
    public MaxSonarI2CXL frontSonarRightSide;
    public AnalogInput hopperLoadingSensor;

    @Override
    public void init(HardwareMap hardwareMap)
    {
        imu = new FrogBNO055(hardwareMap.get(BNO055IMU.class, "external_IMU"));

        rearSonar = hardwareMap.get(MaxSonarI2CXL.class, "rearSonar");
        leftSonar = hardwareMap.get(MaxSonarI2CXL.class, "leftSonar");
        rightSonar = hardwareMap.get(MaxSonarI2CXL.class, "rightSonar");

        //frontSonarLeftSide = hardwareMap.get(MaxSonarI2CXL.class, "frontSonar");

        frontSonarLeftSide = hardwareMap.get(MaxSonarI2CXL.class, "frontSonarLeftSide");
        frontSonarRightSide = hardwareMap.get(MaxSonarI2CXL.class, "frontSonarRightSide");

        hopperLoadingSensor = hardwareMap.analogInput.get("hopperLoadingSensor");
    }
}

 */
