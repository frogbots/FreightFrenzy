package org.firstinspires.ftc.teamcode.trajectory;

import org.firstinspires.ftc.teamcode.R;

import static org.firstinspires.ftc.teamcode.Globals.ATW;
import static org.firstinspires.ftc.teamcode.Globals.RTW;
import static org.firstinspires.ftc.teamcode.trajectory.StateMachine.ReturnState.PROCEED;

public class RightTWLift extends StateMachine<RightTWLift.State> implements StateMMovmentPerformer {

    public double angle;


    public enum State {
        START,
        STOP,
        IDLE,


    }


    public RightTWLift.State getStAte()
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
    public RightTWLift() {
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
                RTW.setPower(0); //RTW Set to reverse Power fore 100 ms
                if (getElapsedStateTime() > 100) {
                    RTW.setPower(0);
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
