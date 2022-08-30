package org.firstinspires.ftc.teamcode.trajectory;

import org.firstinspires.ftc.teamcode.Globals;

import ftc.teamcode.FreightFrenzy.FrogTeleOpFF;

import static org.firstinspires.ftc.teamcode.Globals.ARML;
import static org.firstinspires.ftc.teamcode.Globals.ARMR;
import static org.firstinspires.ftc.teamcode.Globals.HEXCLAW;
import static org.firstinspires.ftc.teamcode.Globals.RotationI;
import static org.firstinspires.ftc.teamcode.trajectory.FMovementDone.State.IDLE;
import static org.firstinspires.ftc.teamcode.trajectory.StateMachine.ReturnState.PROCEED;

public class FMovementDone extends StateMachine<FMovementDone.State> implements StateMMovmentPerformer {

    public double angle;


    public enum State {
        START,
        IDLE,

    }


    public FMovementDone.State getStAte()
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
    public FMovementDone() {
        state = State.START;

    }
    @Override
    public ReturnState runIteration() {
        switch (state) {

            case START: {
                Globals.TurretTurn = true;
                Globals.TurretTARGET = .6;
                switchState(IDLE);
                 break;
            }
            case IDLE: {
                return PROCEED;

            }
        }
        return ReturnState.KEEP_RUNNING_ME;
    }
}
