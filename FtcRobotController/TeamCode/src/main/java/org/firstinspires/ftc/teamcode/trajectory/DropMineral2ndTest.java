package org.firstinspires.ftc.teamcode.trajectory;

import org.firstinspires.ftc.teamcode.Globals;


import static org.firstinspires.ftc.teamcode.trajectory.StateMachine.ReturnState.PROCEED;

public class DropMineral2ndTest extends StateMachine<DropMineral2ndTest.State> implements StateMMovmentPerformer {

    public double angle;


    public enum State {
        START,
        OPEN,
        IDLE,


    }


    public DropMineral2ndTest.State getStAte()
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
        return "DropMineral2ndTest";
    }
    public DropMineral2ndTest() {
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
                Globals.HEXCLAW.setPosition(.2);
                if (getElapsedStateTime() > 500) {
                    switchState(State.IDLE);
                }
                break;
            }
            case IDLE: {
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
