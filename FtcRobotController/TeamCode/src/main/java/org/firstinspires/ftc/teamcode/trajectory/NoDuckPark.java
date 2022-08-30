package org.firstinspires.ftc.teamcode.trajectory;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.TrackingWheelIntegrator;

import static org.firstinspires.ftc.teamcode.trajectory.StateMachine.ReturnState.PROCEED;

public class NoDuckPark extends StateMachine<NoDuckPark.State> implements StateMMovmentPerformer {

    double DisToWall;
    TrackingWheelIntegrator trackingWheelIntegrator = new TrackingWheelIntegrator();

    public enum State {
        START,
        RESET,
        IDLE,


    }


    public NoDuckPark.State getStAte()
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
    public NoDuckPark() {
        state = State.START;

    }
    @Override
    public ReturnState runIteration() {
        switch (state) {

            case START: {
                switchState(State.RESET);
                 break;
            }
            case RESET: {

                Globals.NoDuckPark = true;
                if (getElapsedStateTime() > 10) {
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
