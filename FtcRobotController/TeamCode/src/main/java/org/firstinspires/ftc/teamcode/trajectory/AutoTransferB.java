/*package org.firstinspires.ftc.teamcode.trajectory;

import org.firstinspires.ftc.teamcode.Globals;
import static org.firstinspires.ftc.teamcode.trajectory.StateMachine.ReturnState.KEEP_RUNNING_ME;
import static org.firstinspires.ftc.teamcode.trajectory.StateMachine.ReturnState.PROCEED;

public class AutoTransferB extends StateMachine<AutoTransferB.State>{

    enum State {
        START,
        RDY_POS,
        SHOOT,
        END
    }

    @Override
    public String getName() {
        return "AutoTransferB";
    }

    public AutoTransferB()  {
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

                if(getElapsedStateTime() > 75)
                {
                    switchState(State.RDY_POS);
                }
                break;
            }
            case RDY_POS: {

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