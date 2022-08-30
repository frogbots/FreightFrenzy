package org.firstinspires.ftc.teamcode.trajectory;

import org.firstinspires.ftc.teamcode.Globals;

import static org.firstinspires.ftc.teamcode.trajectory.StateMachine.ReturnState.PROCEED;

public class DuckyWheelSpeedUp extends StateMachine<DuckyWheelSpeedUp.State> implements StateMMovmentPerformer {

    public double angle;


    enum State {
        START,
        SLOW,
        SLOWISH,
        MID,
        FAST,
        END,

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
        return "DuckyWheelSpeedUp";
    }
    public DuckyWheelSpeedUp() {
        state = State.START;

    }
    @Override
    public ReturnState runIteration() {
        switch (state) {

            case START: {
                switchState(State.SLOW);
                 break;
            }
            case SLOW: {
                Globals.DUCKwheel.setPower(.35);
                if (getElapsedStateTime() > 100) {
                    switchState(State.SLOWISH);
                }
                break;
            }
            case SLOWISH:   {
                Globals.DUCKwheel.setPower(.45);
                if (getElapsedStateTime() > 300) {
                    switchState(State.MID);
                }
                break;
            }
            case MID: {
                Globals.DUCKwheel.setPower(.60);
                if (getElapsedStateTime() > 600) {
                    switchState(State.FAST);
                }
                break;
            }
            case FAST: {
                Globals.DUCKwheel.setPower(1);
                if (getElapsedStateTime() > 1500) {
                    switchState(State.END);
                }
                break;
            }
            case END: {
                if(getElapsedStateTime() > 1000) {
                    return PROCEED;
                }
                break;
            }
        }
        return ReturnState.KEEP_RUNNING_ME;
    }
}
