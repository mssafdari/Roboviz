package com.roboviz;

import android.app.Activity;
import android.os.Bundle;
import java.io.InputStream;
import java.io.IOException;
import org.xmlpull.v1.XmlPullParser;
import org.xmlpull.v1.XmlPullParserFactory;
import org.xmlpull.v1.XmlPullParserException;
import android.widget.TextView;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.Map;
import android.util.Log;
import android.widget.Button;
import android.view.View;
import com.tests.*;
import android.Manifest;
import android.os.Build;
import android.content.pm.PackageManager;
import android.annotation.NonNull;
import android.view.Gravity;
import java.util.Arrays;
import com.tests.kinematicsTests;
import java.util.List;
import android.content.Intent;
import android.widget.ToggleButton;
import android.os.strictmode.CleartextNetworkViolation;

public class MainActivity extends Activity
{

    private TextView debug;
    public static boolean doLog=true;
    private Boolean Tso3=false,Tse3=false,Tkinematics=false;

    private static final String TAG= "activity";
    private static final int REQUEST_EXTERNAL_STORAGE = 1;
    private static String[] PERMISSIONS_STORAGE = {
        Manifest.permission.READ_EXTERNAL_STORAGE,
        Manifest.permission.WRITE_EXTERNAL_STORAGE
    };
    ToggleButton t1,t2,t3;
    @Override
    protected void onCreate(Bundle savedInstanceState)
	{
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
        matrixTests.init(this);
        debug = findViewById(R.id.myText);
		debug.setTextIsSelectable(true);
        Button myButton = findViewById(R.id.testButton);
        t1=findViewById(R.id.toggleSo3t);
        t2=findViewById(R.id.toggleSe3t);
        t3=findViewById(R.id.toggleKinematicst);

		Button goToRobotButton = findViewById(R.id.goToRobotButton);

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M)
		{
            if (checkSelfPermission(Manifest.permission.WRITE_EXTERNAL_STORAGE) != PackageManager.PERMISSION_GRANTED)
			{
                requestPermissions(PERMISSIONS_STORAGE, REQUEST_EXTERNAL_STORAGE);
            }
        }

        t1.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v)
                {
                    if (t1.isChecked())
                    {
                        Tso3=true;
                    }
                    else
                    {
                        Tso3=false;
                    }
                }
                });
        t2.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v)
                {
                    if (t2.isChecked())
                    {
                        Tse3=true;
                    }
                    else
                    {
                        Tse3=false;
                    }
                }
            });
        t3.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v)
                {
                    if (t3.isChecked())
                    {
                        Tkinematics=true;
                    }
                    else
                    {
                        Tkinematics=false;
                    }
                }
            });

        myButton.setOnClickListener(new View.OnClickListener() {
				@Override
				public void onClick(View v)
				{
                    clearLog();
					try
					{
						matrixTests mT = new matrixTests();
						So3Test so3T = new So3Test();
						se3tests se3T = new se3tests();
						kinematicsTests kT=new kinematicsTests();
						String msg = mT.run(false) + so3T.run(Tso3) + se3T.run(Tse3) + kT.run(Tkinematics);
						debug.setGravity(Gravity.CENTER);
						appendLog(msg, true);
                        updateLogDisplay();

						Log.d(TAG, "Test result: " + msg);
					}
					catch (Exception e)
					{
						Log.e(TAG, "Error in test: " + e.getMessage());
						appendLog("Error: " + e.getMessage(), true);
                        updateLogDisplay();
						e.printStackTrace();
					}
				}
			});
		goToRobotButton.setOnClickListener(new View.OnClickListener() {
				@Override
				public void onClick(View v)
                {
					openRobotControlActivity();
				}
			});
        try
        {
            // Check if file exists using utility
            String status = URDFUtils.getURDFFileStatus(this, "ur5.urdf.xml");
            appendLog(status, true);

            // Load robot using utility
            Robot robot = URDFUtils.loadDefaultRobot(this);
            if (robot != null) {
                //debug.setText(robot.robot_init());
                String txt= robot.robot_init();
                clearLog();
                appendLog(txt,true);
            } else {
                debug.setText("Failed to load robot\n" + URDFUtils.getAvailableURDFFilesString(this));
            }
            updateLogDisplay();
        }
        catch (Exception e)
        {
            appendLog("Error: " + e.getMessage() + "\n" + URDFUtils.getAvailableURDFFilesString(this),true);
            updateLogDisplay();
        }
    }

	// In MainActivity - Sending data
    private void openRobotControlActivity()
    {
        try
        {
            // Parse URDF and get joints
            InputStream is = getAssets().open("ur5.urdf.xml");
            Robot robot = new UrdfParser().parse(is);
            List<Joint> joints = robot.joints;

            if (joints == null || joints.isEmpty())
            {
                debug.setText("No joints found in URDF!");
                return;
            }

            Intent intent = new Intent(this, RobotControlActivity.class);

            // ✅ Pass as ParcelableArrayList
            intent.putParcelableArrayListExtra("joints", (ArrayList<Joint>) joints);

            startActivity(intent);

        }
        catch (Exception e)
        {
            Log.e(TAG, "Error: " + e.getMessage());
            debug.setText("Error: " + e.getMessage());
        }
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults)
	{
        if (requestCode == REQUEST_EXTERNAL_STORAGE)
		{
            if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED)
			{
                Log.d(TAG, "Storage permission granted!");
            }
			else
			{
                Log.e(TAG, "Storage permission denied!");
            }
        }
    }

    // Public static log variable - accessible from anywhere
    public static String debugLog = "";
    public static final String EL = "\n";

    // Optional: Method to clear or display the log
    public static void clearLog()
    {
        debugLog = "";
    }

    public static void appendLog(String text, Boolean verbose)
    {
        if (doLog && verbose)
        {
            debugLog += text + EL;
        }
    }

    public static void appendTitle(String text, Boolean verbose)
    {
        if (doLog && verbose)
        {
            debugLog += text + EL;
            debugLog += "______________________________________\n";
        }
    }

    // In your UI, you can display it
    private void updateLogDisplay()
    { 
        debug.setText(debugLog);
    }
}
