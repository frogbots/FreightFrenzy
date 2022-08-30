package org.firstinspires.ftc.teamcode.trajectory;

import org.firstinspires.ftc.teamcode.Globals;

import ftc.teamcode.FreightFrenzy.FrogTeleOpFF;

import static ftc.teamcode.FreightFrenzy.FrogTeleOpFF.RotationIntake;
import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.Globals.ARML;
import static org.firstinspires.ftc.teamcode.Globals.ARMR;
import static org.firstinspires.ftc.teamcode.Globals.HEXCLAW;
import static org.firstinspires.ftc.teamcode.Globals.RotationI;
import static org.firstinspires.ftc.teamcode.trajectory.StateMachine.ReturnState.PROCEED;

public class AutoTransfer extends StateMachine<AutoTransfer.State> implements StateMMovmentPerformer {

    public double angle;


    public enum State {
        START,
        OFF,
        FLIP,
        SPIT,
        GRAB,
        RDYPOS,
        IDLE,

    }


    public AutoTransfer.State getStAte()
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
    public AutoTransfer() {
        state = State.START;

    }
    @Override
    public ReturnState runIteration() {
        switch (state) {

            case START: {
                Globals.TRANSFERGoing = true;
                switchState(State.FLIP);
                 break;
            }
            case FLIP: {
                ARML.setPosition(.95);
                ARMR.setPosition(.05);
                HEXCLAW.setPosition(.1);
                RotationI.setPosition(.79);
                if (getElapsedStateTime() > 100) {
                    switchState(State.OFF);
                }
                break;
            }
            case OFF:   {
                Globals.Intake.setPower(0);
                if (getElapsedStateTime() > 400) {
                    switchState(State.SPIT);
                }
                break;
            }
            case SPIT: {
                Globals.Intake.setPower(-1);
                if (getElapsedStateTime() > 170) {
                    switchState(State.GRAB);
                }
                break;
            }
            case GRAB: {
                HEXCLAW.setPosition(0);
                if (getElapsedStateTime() > 50) {
                    switchState(State.RDYPOS);
                }
                break;
            }
            case RDYPOS: {
                Globals.Intake.setPower(0);
                //Globals.ARML.setPosition(.5);
                //Globals.ARMR.setPosition(.5);
                if (getElapsedStateTime() > 300) {
                    FrogTeleOpFF.FrightDetection = false;
                    switchState(State.IDLE);
                }
                break;
            }
            case IDLE: {

                Globals.TRANSFERGoing = false;
                return PROCEED;

            }
        }
        return ReturnState.KEEP_RUNNING_ME;
    }
}
