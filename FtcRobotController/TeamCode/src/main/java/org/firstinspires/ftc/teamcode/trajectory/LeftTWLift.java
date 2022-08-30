package org.firstinspires.ftc.teamcode.trajectory;

import static org.firstinspires.ftc.teamcode.Globals.ATW;
import static org.firstinspires.ftc.teamcode.Globals.LTW;
import static org.firstinspires.ftc.teamcode.trajectory.StateMachine.ReturnState.PROCEED;

public class LeftTWLift extends StateMachine<LeftTWLift.State> implements StateMMovmentPerformer {

    public double angle;


    public enum State {
        START,
        STOP,
        IDLE,


    }


    public LeftTWLift.State getStAte()
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
    public LeftTWLift() {
        state = State.START;

    }
    @Override
    public ReturnState runIteration() {
        switch (state) {

            case START: {
                switchState(State.STOP);
                 break;
            }
            case STOP: {
                LTW.setPower(0); //ATW Set to reverse Power fore 100 ms
                if (getElapsedStateTime() > 100) {
                    LTW.setPower(0);
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
