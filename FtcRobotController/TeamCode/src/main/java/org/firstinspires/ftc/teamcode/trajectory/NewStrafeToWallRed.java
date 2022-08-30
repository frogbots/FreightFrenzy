package org.firstinspires.ftc.teamcode.trajectory;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Globals;

import static org.firstinspires.ftc.teamcode.Globals.*;
import static org.firstinspires.ftc.teamcode.Globals.ARML;
import static org.firstinspires.ftc.teamcode.Globals.ARMR;
import static org.firstinspires.ftc.teamcode.Globals.FL;
import static org.firstinspires.ftc.teamcode.Globals.FR;
import static org.firstinspires.ftc.teamcode.Globals.HEXCLAW;
import static org.firstinspires.ftc.teamcode.Globals.RR;
import static org.firstinspires.ftc.teamcode.trajectory.StateMachine.ReturnState.PROCEED;

public class NewStrafeToWallRed extends StateMachine<NewStrafeToWallRed.State> implements StateMMovmentPerformer {

    public double angle;


    public enum State {
        START,
        STRAFE,
        IDLE,


    }


    public NewStrafeToWallRed.State getStAte()
    {
        return state;
    }

    @Override
    public boolean run() {
        return runIteration() == PROCEED;
    }

    @Override
    public void reset() {
        state = State.START;

    }

    @Override
    public String getName() {
        return "NewStrafeToWallRed";
    }
    public NewStrafeToWallRed() {
        state = State.START;

    }
    @Override
    public ReturnState runIteration() {
        switch (state) {

            case START: {
                switchState(State.STRAFE);
                 break;
            }
            case STRAFE: {
               FL.setTargetPosition(-50);
               FR.setTargetPosition(50);
               RR.setTargetPosition(-50);
               RL.setTargetPosition(50);
               FL.setPower(-.5);
               FR.setPower(.5);
               RR.setPower(-.5);
               RL.setPower(.5);
                if (getElapsedStateTime() > 250) {
                    switchState(State.IDLE);
                }
                break;
            }
            case IDLE: {
                if(getElapsedStateTime() > 10) {
                    return PROCEED;
                }
                break;
            }

        }
        return ReturnState.KEEP_RUNNING_ME;
    }
}
