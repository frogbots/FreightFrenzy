package org.firstinspires.ftc.teamcode.trajectory;

import android.telephony.gsm.GsmCellLocation;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.TrackingWheelIntegrator;

import ftc.teamcode.FreightFrenzy.FreightFrenzyMainAutoBlueStateM;
import ftc.teamcode.FreightFrenzy.FreightFrenzyMainAutoRedStateM;

import static org.firstinspires.ftc.teamcode.Globals.HEXCLAW;
import static org.firstinspires.ftc.teamcode.Globals.LastX;
import static org.firstinspires.ftc.teamcode.Globals.LastY;
import static org.firstinspires.ftc.teamcode.trajectory.StateMachine.ReturnState.PROCEED;

public class AutoResetOdo extends StateMachine<AutoResetOdo.State> implements StateMMovmentPerformer {

    double DisToWall;
    TrackingWheelIntegrator trackingWheelIntegrator = new TrackingWheelIntegrator();

    public enum State {
        START,
        RESET,
        IDLE,


    }


    public AutoResetOdo.State getStAte()
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
    public AutoResetOdo() {
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
                trackingWheelIntegrator.SetX(Globals.LastX);
                trackingWheelIntegrator.SetY(Globals.LastY);
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
