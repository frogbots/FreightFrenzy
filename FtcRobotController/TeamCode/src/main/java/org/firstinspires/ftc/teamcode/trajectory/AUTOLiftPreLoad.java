package org.firstinspires.ftc.teamcode.trajectory;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Globals;

import ftc.teamcode.FreightFrenzy.FreightFrenzyMainAutoBlueStateM;


import static org.firstinspires.ftc.teamcode.Globals.ARML;
import static org.firstinspires.ftc.teamcode.Globals.ARMR;
import static org.firstinspires.ftc.teamcode.Globals.HEXCLAW;
import static org.firstinspires.ftc.teamcode.trajectory.StateMachine.ReturnState.PROCEED;

public class AUTOLiftPreLoad extends StateMachine<AUTOLiftPreLoad.State> implements StateMMovmentPerformer {

    public double angle;


    public enum State {
        START,
        UP,
        CLAW,
        TURRET,
        IDLE,



    }


    public AUTOLiftPreLoad.State getStAte()
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
    public AUTOLiftPreLoad() {
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
                if (Globals.LOWERARM) {
                    Globals.ARML.setPosition(.15);
                    Globals.ARMR.setPosition(.85);
                }
                else {
                    Globals.ARML.setPosition(.2);
                    Globals.ARMR.setPosition(.8);
                }
                HEXCLAW.setPosition(0);
                Globals.Intake.setPower(0);
                Globals.LiftTarget = LiftPreLoad.Lvl;
                //Globals.LiftLevel = 3;
                Globals.Lift.setTargetPosition(LiftPreLoad.Lvl);
                Globals.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Globals.Lift.setPower(1);

                if (getElapsedStateTime() > 1200) {
                    switchState(State.TURRET);
                }
                break;
            }
            case TURRET: {

                Globals.TurretTurn = true;
                if(getElapsedStateTime() > 700) {
                    HEXCLAW.setPosition(.1);
                    switchState(State.CLAW);
                }
                break;
            }
            case CLAW: {
                HEXCLAW.setPosition(.1);
                ARMR.setPosition(.7);
                ARML.setPosition(.3);
                if (getElapsedStateTime() > 500) {
                    Globals.FirstMoving = false;
                    Globals.WeHaveTheGoods = false;
                    Globals.MineralInClaw = false;
                    Globals.Cycle = Globals.Cycle + 1;
                    switchState(State.IDLE);
                }
                break;
            }

            case IDLE: {
                if(getElapsedStateTime() > 100) {
                    return PROCEED;
                }
                break;
            }

        }
        return ReturnState.KEEP_RUNNING_ME;
    }
}
