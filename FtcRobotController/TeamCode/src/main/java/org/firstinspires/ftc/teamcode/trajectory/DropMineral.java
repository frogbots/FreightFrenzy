package org.firstinspires.ftc.teamcode.trajectory;

import android.telephony.gsm.GsmCellLocation;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Globals;

import ftc.teamcode.FreightFrenzy.FreightFrenzyMainAutoBlueStateM;

import static org.firstinspires.ftc.teamcode.trajectory.StateMachine.ReturnState.PROCEED;

public class DropMineral extends StateMachine<DropMineral.State> implements StateMMovmentPerformer {

    public double angle;


    public enum State {
        START,
        OPEN,
        IDLE,


    }


    public DropMineral.State getStAte()
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
        return "AutoTransfer";
    }
    public DropMineral() {
        state = State.START;

    }
    @Override
    public ReturnState runIteration() {
        switch (state) {

            case START: {
                switchState(State.OPEN);
                 break;
            }
            case OPEN: {
                Globals.HEXCLAW.setPosition(.1);
                if (getElapsedStateTime() > 500) {
                    switchState(State.IDLE);
                }
                break;
            }
            case IDLE: {
                Globals.HEXCLAW.setPosition(0);
                Globals.FirstMoving = false;
                Globals.WeHaveTheGoods = false;
                Globals.MineralInClaw = false;
                Globals.Cycle = Globals.Cycle+1;
                if(getElapsedStateTime() > 1) {
                    return PROCEED;
                }
                break;
            }

        }
        return ReturnState.KEEP_RUNNING_ME;
    }
}
