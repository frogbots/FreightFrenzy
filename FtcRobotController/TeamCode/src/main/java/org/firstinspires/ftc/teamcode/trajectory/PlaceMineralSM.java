package org.firstinspires.ftc.teamcode.trajectory;

import org.firstinspires.ftc.teamcode.Globals;

import static org.firstinspires.ftc.teamcode.Globals.Booper;
import static org.firstinspires.ftc.teamcode.Globals.TSELift;
import static org.firstinspires.ftc.teamcode.trajectory.SMtransfer.State.INTAKEOFF;
import static org.firstinspires.ftc.teamcode.trajectory.SMtransfer.State.ROTATION;
import static org.firstinspires.ftc.teamcode.trajectory.StateMachine.ReturnState.PROCEED;


public class PlaceMineralSM extends StateMachine<PlaceMineralSM.State> implements StateMMovmentPerformer {


enum State {
  START,
  OPEN,
  END,
}

//@Override
public boolean run() {
return runIteration() == PROCEED;
}

@Override
public void reset() {
state = State.START;

}

@Override
public String getName() {
return "PlaceMineralSM";
}


@Override
public ReturnState runIteration() {
switch (state) {

    case START: {

        switchState(State.OPEN);
        break;


    }
    case OPEN: {
        Globals.HEXCLAW.setPosition(.1);
        if (getElapsedStateTime() > 200) {
            switchState(State.END);
        }
        break;
    }
    case END: {
        if(getElapsedStateTime() > 1)
        {
            return PROCEED;
        }
    }
}
return ReturnState.KEEP_RUNNING_ME;
}
}
