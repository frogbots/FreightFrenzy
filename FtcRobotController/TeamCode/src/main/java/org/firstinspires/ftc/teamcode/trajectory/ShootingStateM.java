/*package org.firstinspires.ftc.teamcode.trajectory;

import org.firstinspires.ftc.teamcode.Globals;

import static trajectory.StateMachine.ReturnState.KEEP_RUNNING_ME;
import static trajectory.StateMachine.ReturnState.PROCEED;

public class ShootingStateM extends StateMachine<ShootingStateM.State>{

    enum State {
      START,
      RDY_POS,
      SHOOT,
      END
    }

    @Override
    public String getName() {
        return "ShootingStateM";
    }

    public ShootingStateM()  {
        state = State.START;
    }
    @Override
    public ReturnState runIteration() {
        switch (state) {

            case START:{
                switchState(State.SHOOT);
                break;
            }
            case SHOOT: {
                Globals.Tbooper.setPosition(.33);
                if(getElapsedStateTime() > 75)
                {
                    switchState(State.RDY_POS);
                }
                break;
            }
            case RDY_POS: {
                Globals.Tbooper.setPosition(.16);
                if(getElapsedStateTime() > 180)
                {
                    switchState(State.END);
                }
                break;
            }
            case END: {
                return PROCEED;
            }
        }
        return KEEP_RUNNING_ME;
    }

    @Override
    public void reset() {
        state = State.START;
    }
}


 */