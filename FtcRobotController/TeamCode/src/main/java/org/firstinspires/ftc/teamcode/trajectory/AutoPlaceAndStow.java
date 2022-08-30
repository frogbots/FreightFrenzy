package org.firstinspires.ftc.teamcode.trajectory;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.NotFF.FrogTeleOp;

import ftc.teamcode.FreightFrenzy.FrogTeleOpFF;

import static org.firstinspires.ftc.teamcode.Globals.ARML;
import static org.firstinspires.ftc.teamcode.Globals.ARMR;
import static org.firstinspires.ftc.teamcode.Globals.HEXCLAW;
import static org.firstinspires.ftc.teamcode.Globals.RotationI;
import static org.firstinspires.ftc.teamcode.trajectory.StateMachine.ReturnState.PROCEED;

public class AutoPlaceAndStow extends StateMachine<AutoPlaceAndStow.State> implements StateMMovmentPerformer {

    public double angle;


    enum State {
        START,
        PLACE,
        STOW,
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
        return "AutoPlaceAndStow";
    }
    public AutoPlaceAndStow() {
        state = State.START;

    }
    @Override
    public ReturnState runIteration() {
        switch (state) {

            case START: {
                switchState(State.PLACE);
                 break;
            }
            case PLACE: {
                HEXCLAW.setPosition(0);
                if (getElapsedStateTime() > 400) {
                    switchState(State.STOW);
                }
                break;
            }
            case STOW:   {
                ARML.setPosition(.95);
                ARMR.setPosition(.05);
                Globals.LiftTarget = 80;
                Globals.Lift.setTargetPosition(1);
                Globals.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Globals.Lift.setPower(0.7);//(1+LiftAdjustment) - LiftPos)/400
                FrogTeleOpFF.TurretTARGET = 1.29;
                FrogTeleOpFF.TurretTurn = true;


                if (getElapsedStateTime() > 1000) {
                    switchState(State.END);
                }
                break;
            }
            case END: {
                if(getElapsedStateTime() > 1000) {
                    FrogTeleOpFF.FrightDetection = false;
                    return PROCEED;
                }
                break;
            }
        }
        return ReturnState.KEEP_RUNNING_ME;
    }
}
