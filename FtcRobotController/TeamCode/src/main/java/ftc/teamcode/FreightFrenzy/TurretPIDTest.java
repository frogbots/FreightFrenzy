package ftc.teamcode.FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import net.frogbots.ftcopmodetunercommon.opmode.TunableLinearOpMode;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.robotComponents.drivebase.MotorPowers;

import static org.firstinspires.ftc.teamcode.Globals.Turret;
import static org.firstinspires.ftc.teamcode.Globals.potentiometer;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import  java.nio.ByteBuffer;



@TeleOp
public class TurretPIDTest extends TunableLinearOpMode
{

    double[][]	logdata;
    final	int	MAXFRAMES = 1024*8;
    final	int	NITEMS = 6;
    final	double	INCHES = 1.0;
    final	double	WIDTH = 18.0 * INCHES;
    final	double	COUNTS_PER_ROTATION = 4;
    final	double	DIAMETER = 4 * INCHES;
    int		frame, frphase;
    FileOutputStream	ofp;
    boolean global_error;

    //Kp = .60*Kc = 5.52
    //Ki = 2*Kp*dT/Pc = 0.02208
    //Kd = KpPc/(8dT) = 345
//    double Kc = 9.2;
//    double dT = .0010;
//    double Pc = .5;

    double Kp=0;
    double Ki=0;
    double Kd=0;
    double integral;
    double error;
    double turnPower;
    boolean MotorGo;
    double derivative;
    double lastError;
    double RealPot;
    double targetPosition;


    public void openfile() {
        String	filename = "/sdcard/telelog.bin";
        // File	filp = new File("/sdcard", "telelog.bin");

        try {
            ofp = new FileOutputStream(filename);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
            global_error = true;
        }
        logdata = new double[MAXFRAMES][NITEMS];
        frame = 0;
        frphase = 0;
    }
    public	void	recordstate() {
        int k = 0;
        if (frame < MAXFRAMES) {

            frphase++;
            if (frphase >= 1) {
                frphase = 0;
                logdata[frame][k++] = integral;
                logdata[frame][k++] = derivative;
                logdata[frame][k++] = error;
                logdata[frame][k++] = targetPosition;
                logdata[frame][k++] = RealPot;
                logdata[frame][k++] = turnPower;
                frame++;
            }
        }
    }
    public	void	savefile() {
        ByteBuffer	bb = ByteBuffer.allocate(frame * NITEMS * 8);
        for(int f = 0; f < frame; f++) {
            for(double d : logdata[f])
                bb.putDouble(d);
        }
        try {
            ofp.write(bb.array());
        } catch (IOException e) {
            e.printStackTrace();
            telemetry.addLine("Write Error" + e.toString());
            telemetry.update();
            global_error = true;
        }

        try {
            ofp.close();
        } catch (IOException e) {
            e.printStackTrace();
            telemetry.addLine("Close Error" + e.toString());
            telemetry.update();
            global_error = true;
        }
    }


    //System.out.println(integral+","+derivative+","+error +","+targetPosition+","+RealPot+","+turnPower);

    @Override
    public void runOpMode() throws InterruptedException {

        Globals.potentiometer = hardwareMap.get(AnalogInput.class, "Potentiometer");
        Turret = hardwareMap.get(DcMotorEx.class, "Turret");
        Turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        openfile();
        waitForStart();


        while (opModeIsActive())
        {
            RealPot = potentiometer.getVoltage();
            Kp = getDouble("Kp");
            Ki = getDouble("Ki");
            Kd = getDouble("Kd");
            targetPosition = getDouble("Targetposition");
            MotorGo = getBoolean("MotorGo");



            telemetry.addData("Kp" ,Kp);
            telemetry.addData("Ki" ,Ki);
            telemetry.addData("Kd" ,Kd);
            telemetry.addData("Integral" ,integral);
            telemetry.addData("derivative" , derivative);
            telemetry.addData("TargetPosiotion" ,targetPosition);
            telemetry.addData("Real position" , RealPot);
            telemetry.addData("Error" ,error);
            telemetry.addData("Turn Power" ,turnPower);
            telemetry.addData("MotorGo" ,MotorGo);
            telemetry.update();

            //System.out.println(integral+","+derivative+","+error +","+targetPosition+","+RealPot+","+turnPower);
            if (MotorGo) {
                if (error < 0 && lastError > 0 || error > 0 && lastError < 0 ){
                    integral = 0;
                }

                error = targetPosition - RealPot;
                integral = integral + error;
                derivative = error - lastError;
                turnPower = (Kp*(error) + Ki * (integral) + Kd * (derivative));
                if (Math.abs(turnPower) > 1) {
                    integral = integral - error;
                }
                if ((RealPot > 2.8 && turnPower < 0)|| (RealPot < .34 && turnPower > 0)) {
                    //consider adding a Method to make it be able to tun back into range
                    MotorGo = false;
                    Turret.setPower(0);
                }
                if (MotorGo) {
                    Turret.setPower(turnPower);
                }
                lastError = error;
                recordstate();
            }
            else {
                integral = 0;
                Turret.setPower(0);
            }

        }
        savefile();
    }

}
