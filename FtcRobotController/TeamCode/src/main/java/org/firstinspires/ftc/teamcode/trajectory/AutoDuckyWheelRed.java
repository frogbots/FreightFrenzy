package org.firstinspires.ftc.teamcode.trajectory;

import org.firstinspires.ftc.teamcode.Globals;

import static org.firstinspires.ftc.teamcode.Globals.DUCKwheel;
import static org.firstinspires.ftc.teamcode.trajectory.StateMachine.ReturnState.PROCEED;

public class AutoDuckyWheelRed extends StateMachine<AutoDuckyWheelRed.State> implements StateMMovmentPerformer {

    public double angle;


    public enum State {
        START,
        SPIN,
        IDLE,


    }


    public AutoDuckyWheelRed.State getStAte()
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
    public AutoDuckyWheelRed() {
        state = State.START;

    }
    @Override
    public ReturnState runIteration() {
        switch (state) {

            case START: {
                switchState(State.SPIN);
                 break;
            }
            case SPIN: {
                DUCKwheel.setPower(-.7);
                if (getElapsedStateTime() > 3000) {
                    DUCKwheel.setPower(0);
                    Globals.GoDucks = false;
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
