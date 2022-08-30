package org.firstinspires.ftc.teamcode.trajectory;

import org.firstinspires.ftc.teamcode.Globals;

import static org.firstinspires.ftc.teamcode.trajectory.FMovementDone.State.IDLE;
import static org.firstinspires.ftc.teamcode.trajectory.StateMachine.ReturnState.PROCEED;

public class IntakeOnSM extends StateMachine<IntakeOnSM.State> implements StateMMovmentPerformer {

    public double angle;


    public enum State {
        START,
        IDLE,

    }


    public IntakeOnSM.State getStAte()
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
        return "FMovemnentDone";
    }
    public IntakeOnSM() {
        state = State.START;

    }
    @Override
    public ReturnState runIteration() {
        switch (state) {

            case START: {
                Globals.Intake.setPower(1);
                Globals.RotationI.setPosition(.125);
                switchState(State.IDLE);
                 break;
            }
            case IDLE: {
                return PROCEED;

            }
        }
        return ReturnState.KEEP_RUNNING_ME;
    }
}
