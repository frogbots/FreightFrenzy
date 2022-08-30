package ftc.teamcode;

public class AbortAutoNowException extends Throwable {
    private static final String msg = ("Hello") ;

    public AbortAutoNowException(String s) {
        super(msg);
    }
}
        