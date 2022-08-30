package org.firstinspires.ftc.teamcode.trajectory;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Globals;

import ftc.teamcode.FreightFrenzy.FrogTeleOpFF;

import static org.firstinspires.ftc.teamcode.Globals.ARML;
import static org.firstinspires.ftc.teamcode.Globals.ARMR;
import static org.firstinspires.ftc.teamcode.Globals.HEXCLAW;
import static org.firstinspires.ftc.teamcode.trajectory.StateMachine.ReturnState.PROCEED;

public class AutoLiftDownWithWaitTele extends StateMachine<AutoLiftDownWithWaitTele.State> implements StateMMovmentPerformer {



    public enum State {
        START,
        TURRET,
        FLIP,
        DOWN,
        IDLE,

    }


    public AutoLiftDownWithWaitTele.State getStAte()
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
    public AutoLiftDownWithWaitTele() {
        state = State.START;

    }
    @Override
    public ReturnState runIteration() {
        switch (state) {

            case START: {
                switchState(State.TURRET);
                 break;
            }
            case TURRET:   {
                HEXCLAW.setPosition(0);
                FrogTeleOpFF.TurretTurn = true;
                FrogTeleOpFF.TurretTARGET=1.29;

                if (getElapsedStateTime() > 400) {
                    switchState(State.FLIP);
                }
                break;
            }
            case FLIP: {
                ARML.setPosition(.95);
                ARMR.setPosition(.05);
                HEXCLAW.setPosition(0);
                if (getElapsedStateTime() > 250) {
                    switchState(State.DOWN);
                }
                break;
            }

            case DOWN: {
                Globals.LiftTarget = 0;
                //Globals.LiftLevel = 3;
                Globals.Lift.setTargetPosition(0);
                Globals.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Globals.Lift.setPower(1);
                FrogTeleOpFF.LiftGoDown = false;
                if (getElapsedStateTime() > 500) {
                    switchState(State.IDLE);
                }
                break;
            }

            case IDLE: {
                return PROCEED;

            }
        }
        return ReturnState.KEEP_RUNNING_ME;
    }
}
