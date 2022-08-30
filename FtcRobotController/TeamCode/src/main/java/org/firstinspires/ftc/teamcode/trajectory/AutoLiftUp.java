package org.firstinspires.ftc.teamcode.trajectory;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Globals;

import ftc.teamcode.FreightFrenzy.FreightFrenzyMainAutoBlueStateM;
import ftc.teamcode.FreightFrenzy.FrogTeleOpFF;


import static org.firstinspires.ftc.teamcode.Globals.ARML;
import static org.firstinspires.ftc.teamcode.Globals.ARMR;
import static org.firstinspires.ftc.teamcode.Globals.HEXCLAW;
import static org.firstinspires.ftc.teamcode.Globals.RotationI;
import static org.firstinspires.ftc.teamcode.trajectory.StateMachine.ReturnState.PROCEED;

public class AutoLiftUp extends StateMachine<AutoLiftUp.State> implements StateMMovmentPerformer {

    public double angle;


    public enum State {
        START,
        UP,
        TURRET,
        CLAW,
        IDLE,


    }


    public AutoLiftUp.State getStAte()
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
    public AutoLiftUp() {
        state = State.START;

    }
    @Override
    public ReturnState runIteration() {
        switch (state) {

            case START: {
                switchState(State.UP);
                 break;
            }
            case UP: {
                //HEXCLAW.setPosition(0);
                Globals.ARML.setPosition(.2);
                Globals.ARMR.setPosition(.8);
                Globals.Intake.setPower(0);
                Globals.LiftTarget = 330;
                //Globals.LiftLevel = 3;
                Globals.Lift.setTargetPosition(330);
                Globals.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Globals.Lift.setPower(1);
                if (getElapsedStateTime() > 500) {
                    switchState(State.TURRET);
                }
                break;
            }
            case TURRET: {
                Globals.TurretTARGET = .65;
                Globals.TurretTurn = true;
                if(Globals.ReadyToPlace && Globals.currentVoltage < .7) {
                    HEXCLAW.setPosition(.1);
                    switchState(State.CLAW);
                }
                break;
            }
            case CLAW: {
                HEXCLAW.setPosition(.1);
                if (getElapsedStateTime() > 300) {
                    ARMR.setPosition(.5);
                    ARML.setPosition(.5);
                    Globals.FirstMoving = false;
                    Globals.WeHaveTheGoods = false;
                    Globals.MineralInClaw = false;
                    Globals.ReadyToPlace = false;
                    Globals.Cycle = Globals.Cycle + 1;
                    switchState(State.IDLE);
                }
                break;
            }
            case IDLE: {
                if(getElapsedStateTime() > 1000) {
                    return PROCEED;
                }
                break;
            }

        }
        return ReturnState.KEEP_RUNNING_ME;
    }
}
