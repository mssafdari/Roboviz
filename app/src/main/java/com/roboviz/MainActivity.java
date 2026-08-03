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

public class MainActivity extends Activity
{

    private TextView debug;
    public static boolean doLog=true;

    private static final String TAG= "activity";
    private static final int REQUEST_EXTERNAL_STORAGE = 1;
    private static String[] PERMISSIONS_STORAGE = {
        Manifest.permission.READ_EXTERNAL_STORAGE,
        Manifest.permission.WRITE_EXTERNAL_STORAGE
    };

    @Override
    protected void onCreate(Bundle savedInstanceState)
	{
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
        matrixTests.init(this);
        debug = findViewById(R.id.myText);
		debug.setTextIsSelectable(true);
        Button myButton = findViewById(R.id.testButton);
		Button goToRobotButton = findViewById(R.id.goToRobotButton);

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M)
		{
            if (checkSelfPermission(Manifest.permission.WRITE_EXTERNAL_STORAGE) != PackageManager.PERMISSION_GRANTED)
			{
                requestPermissions(PERMISSIONS_STORAGE, REQUEST_EXTERNAL_STORAGE);
            }
        }
        myButton.setOnClickListener(new View.OnClickListener() {
				@Override
				public void onClick(View v)
				{
					try
					{
						matrixTests mT = new matrixTests();
						So3Test so3T = new So3Test();
						se3tests se3T = new se3tests();
						kinematicsTests kT=new kinematicsTests();
						String msg = mT.run() + so3T.run() + se3T.run() + kT.run();
						debug.setGravity(Gravity.CENTER);
						appendLog(msg,true);
                        updateLogDisplay();

						Log.d(TAG, "Test result: " + msg);
					}
					catch (Exception e)
					{
						Log.e(TAG, "Error in test: " + e.getMessage());
						appendLog("Error: " + e.getMessage(),true);
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
            checkFile();
            InputStream is = getAssets().open("ur5.urdf.xml");
            Robot robot = new UrdfParser().parse(is);
            debug.setText(robot.toDebugString());
        }
		catch (Exception e)
		{
            debug.setText(e.getMessage());
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
            List<Joint> joints = getJointsFromRobot(robot);

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

    // ✅ Helper method to extract joints from Robot object
    private List<Joint> getJointsFromRobot(Robot robot)
    {
        List<Joint> joints = new ArrayList<>();

        // This depends on your Robot class structure
        // Example: if Robot has a getJoints() method
        // return robot.getJoints();

        // OR if you need to manually extract:
        // for (JointObject jo : robot.getJointObjects()) {
        //     joints.add(new Joint(jo.getName(), 
        //                         jo.getDefaultPosition(),
        //                         jo.getMinPosition(),
        //                         jo.getMaxPosition(),
        //                         jo.getType()));
        // }

        // For now, return sample joints if parsing fails
        // (Replace this with actual extraction from your Robot class)
        joints.add(new Joint("shoulder_pan", 0.0, -3.14, 3.14, "revolute"));
        joints.add(new Joint("shoulder_lift", 0.5, -2.5, 2.5, "revolute"));
        joints.add(new Joint("elbow", -0.3, -2.0, 2.0, "revolute"));
        joints.add(new Joint("wrist_1", 1.2, -1.5, 1.5, "revolute"));
        joints.add(new Joint("wrist_2", -0.8, -1.5, 1.5, "revolute"));
        joints.add(new Joint("wrist_3", 0.2, -3.14, 3.14, "revolute"));

        return joints;
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
    private void checkFile()
	{
        try
		{
            // First check if file exists in assets
            String[] files = getAssets().list("");
            boolean fileFound = false;
            for (String file : files)
			{
                if (file.equals("ur5.urdf.xml"))
				{
                    fileFound = true;
                    break;
                }
            }

            if (!fileFound)
			{
                debug.setText("File not found in assets! Available files: " + Arrays.toString(files));
                return;
            }
        }
		catch (Exception e)
		{
            debug.setText(e.getMessage());
        }
    }
    
    
        // Public static log variable - accessible from anywhere
        public static String debugLog = "";
        public static final String EL = "\n";

        // Optional: Method to clear or display the log
        public static void clearLog() {
            debugLog = "";
        }

        public static void appendLog(String text,Boolean verbose) {
            if(doLog && verbose){
                debugLog += text + EL;
            }
        }

        public static void appendTitle(String text,Boolean verbose) {
            if(doLog && verbose){
                debugLog += text + EL;
                debugLog +="______________________________________\n";
            }
        }
        
        // In your UI, you can display it
        private void updateLogDisplay() { 
            debug.setText(debugLog);
        }
}
