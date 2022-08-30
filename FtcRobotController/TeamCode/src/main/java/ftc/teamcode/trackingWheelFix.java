package ftc.teamcode;


public class trackingWheelFix {

    public short last = 0;
    public int counter = 0;
    public short diff = 0;

    public int acc (short counts) {
       // diff = counts-last;
        counter += diff;
        return counter;

    }

}
