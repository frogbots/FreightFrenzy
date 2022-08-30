package org.firstinspires.ftc.teamcode.trajectory;

import org.firstinspires.ftc.teamcode.Globals;

import static org.firstinspires.ftc.teamcode.Globals.ATW;
import static org.firstinspires.ftc.teamcode.Globals.DUCKwheel;
import static org.firstinspires.ftc.teamcode.trajectory.StateMachine.ReturnState.PROCEED;

public class AUXLift extends StateMachine<AUXLift.State> implements StateMMovmentPerformer {

    public double angle;


    public enum State {
        START,
        STOP,
        IDLE,


    }


    public AUXLift.State getStAte()
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
    public AUXLift() {
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
                ATW.setPower(0); //ATW Set to reverse Power fore 100 ms
                if (getElapsedStateTime() > 100) {
                    ATW.setPower(0);
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
