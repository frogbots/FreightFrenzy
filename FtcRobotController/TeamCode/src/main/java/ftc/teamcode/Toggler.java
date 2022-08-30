package ftc.teamcode;

public class Toggler
{
    private boolean lastState = false;

    public boolean shouldToggle(boolean currentBtnState)
    {
        if(currentBtnState && !lastState)
        {
            lastState = true;

            return true;
        }
        else if (!currentBtnState)
        {
            lastState = false;
        }
        return false;
    }
}
