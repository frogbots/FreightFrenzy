package org.firstinspires.ftc.teamcode.trajectory;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.TrackingWheelIntegrator;

import ftc.teamcode.FreightFrenzy.FreightFrenzyMainAutoRedStateM;

import static org.firstinspires.ftc.teamcode.Globals.LastY;
import static org.firstinspires.ftc.teamcode.trajectory.StateMachine.ReturnState.PROCEED;

public class AutoResetOdoWarehouse extends StateMachine<AutoResetOdoWarehouse.State> implements StateMMovmentPerformer {

    double DisToWall;
    TrackingWheelIntegrator trackingWheelIntegrator = new TrackingWheelIntegrator();

    public enum State {
        START,
        RESET,
        IDLE,


    }


    public AutoResetOdoWarehouse.State getStAte()
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
    public AutoResetOdoWarehouse() {
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
                Globals.LastX = Globals.X;
                LastY = Globals.Y;
                FreightFrenzyMainAutoRedStateM.clearEnc();
                if (LastY < 90){
                    LastY = 94;
                }
                trackingWheelIntegrator.SetX(Globals.LastX);
                trackingWheelIntegrator.SetY(LastY);
                if (getElapsedStateTime() > 100) {
                    switchState(State.IDLE);
                }
                break;
            }
            case IDLE: {
                if(getElapsedStateTime() > 200) {
                    return PROCEED;
                }
                break;
            }

        }
        return ReturnState.KEEP_RUNNING_ME;
    }
}
