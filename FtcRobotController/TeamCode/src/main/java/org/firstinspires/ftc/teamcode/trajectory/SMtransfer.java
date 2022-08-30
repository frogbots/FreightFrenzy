package org.firstinspires.ftc.teamcode.trajectory;

        import org.firstinspires.ftc.teamcode.Globals;

        import static org.firstinspires.ftc.teamcode.trajectory.SMtransfer.State.INTAKEOFF;
        import static org.firstinspires.ftc.teamcode.trajectory.SMtransfer.State.ROTATION;
        import static org.firstinspires.ftc.teamcode.trajectory.StateMachine.ReturnState.PROCEED;


        public class SMtransfer extends StateMachine<SMtransfer.State> implements StateMMovmentPerformer {


        enum State {
          START,
          INTAKEOFF,
          ROTATION,
          TRANSFER,
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
        return "SMtransfer";
    }


    @Override
    public ReturnState runIteration() {
        switch (state) {

            case START: {
                switchState(INTAKEOFF);
                break;


            }
            case INTAKEOFF: {
                Globals.Intake.setPower(0);
                if (getElapsedStateTime() > 100) {
                    switchState(ROTATION);
                }
                break;
            }
            case ROTATION: {
                Globals.RotationI.setPosition(.86);
                if (getElapsedStateTime() > 400) {
                    switchState(State.TRANSFER);
                }
                break;
            }
            case TRANSFER: {
                Globals.Intake.setPower(-1);
                if (getElapsedStateTime() > 500) {
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

    //public interface SMtransfer {
    //public boolean run();
    //public void reset();
//}
