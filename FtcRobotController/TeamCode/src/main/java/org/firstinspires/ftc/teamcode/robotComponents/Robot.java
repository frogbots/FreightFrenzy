package org.firstinspires.ftc.teamcode.robotComponents;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robotComponents.drivebase.SkyStoneDriveBase;


/**
 * Created by michael on 10/2/18.
 */

public class Robot
{

    public SkyStoneDriveBase driveTrain = new SkyStoneDriveBase();

    //public Intake intake = new Intake();

    public void init(HardwareMap hardwareMap, boolean resetEncoders)
    {
        //hubEx = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        // module = (LynxModule) hardwareMap.get(LynxModule.class, "Expansion Hub 3"); // I add this one

        driveTrain.init(hardwareMap);

        if(resetEncoders)
        {
            driveTrain.resetEncoders();

        }

    }
}

