package org.firstinspires.ftc.teamcode.trajectory;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Globals;

import static org.firstinspires.ftc.teamcode.Globals.HEXCLAW;
import static org.firstinspires.ftc.teamcode.Globals.Turret;
import static org.firstinspires.ftc.teamcode.trajectory.StateMachine.ReturnState.PROCEED;

public class TurretSMAUTO extends StateMachine<TurretSMAUTO.State> implements StateMMovmentPerformer {

    double Kp=1.2; //2.2
    double Ki=.008; //.02
    double Kd=1;  //15
    double integral;
    double error;
    double turnPower;
    boolean MotorGo;
    double derivative;
    double lastError;
    double RealPot;
    double targetPosition;


    public enum State {
        START,
        TURRETTURN,
        IDLE,


    }


    public TurretSMAUTO.State getStAte()
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
    public TurretSMAUTO() {
        state = State.START;

    }
    @Override
    public ReturnState runIteration() {
        switch (state) {

            case START: {
                switchState(State.TURRETTURN);
                Globals.TurretTurn = true;
                 break;
            }
            case TURRETTURN: {


                    if (error < 0 && lastError > 0 || error > 0 && lastError < 0 ){
                        integral = 0;
                    }
                    error = .6 - RealPot;
                    integral = integral + error;
                    derivative = error - lastError;
                    turnPower = (Kp*(error) + Ki * (integral) + Kd * (derivative));
                    if (Math.abs(turnPower) > 1) {
                        integral = integral - error;
                    }
                    if ((RealPot > 2.8 && turnPower < 0)|| (RealPot < .34 && turnPower > 0)) {
                        //consider adding a Method to make it be able to tun back into range
                        Globals.TurretTurn = false;
                        Turret.setPower(0);
                    }
                    if (Globals.TurretTurn) {
                        Turret.setPower(turnPower);
                    }
                    lastError = error;
                if (Globals.TurretPos == Globals.TurretTARGET) {
                    switchState(State.IDLE);
                }
                else {
                    switchState(State.TURRETTURN);
                }
                if (getElapsedStateTime() > 500) {
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
