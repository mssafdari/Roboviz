package com.roboviz;

import java.util.*;
import android.widget.*;
import android.os.Bundle;
import java.io.InputStream;
import android.app.Activity;
import android.view.View;
import android.view.LayoutInflater;
import android.content.Intent;
import com.visualisation.RobotCanvasView;
import com.math.se3group;
import com.math.Matrix;
import com.math.Vector;

// RobotControlActivity.java
public class RobotControlActivity extends Activity
{
    public static String debugLog="";
    private static final String EL="\n";
    private static boolean doLog=true;
    private TextView debug;
    private RobotCanvasView canvasView; 
    private LinearLayout slidersContainer;
    private List<Joint> joints = new ArrayList<>();
    private Map<String, SeekBar> sliderMap = new HashMap<>();
    private Map<String, TextView> valueMap = new HashMap<>();

    private Robot robot;

    @Override
    protected void onCreate(Bundle savedInstanceState)
	{
        super.onCreate(savedInstanceState);
        setContentView(R.layout.robot_view);
        debug = findViewById(R.id.myText);
        debug.setTextIsSelectable(true);
        canvasView = findViewById(R.id.robot_canvas); 

        slidersContainer = findViewById(R.id.sliders_container);

        // Setup buttons
        findViewById(R.id.btn_reset).setOnClickListener(new View.OnClickListener() {
				@Override
				public void onClick(View v)
                {
					resetToDefault(v);
				}
			});

        findViewById(R.id.btn_apply).setOnClickListener(new View.OnClickListener() {
				@Override
				public void onClick(View v)
                {
					applyJointAngles(v);
				}
			});
        try
        {
            // Check if file exists using utility
            String status = URDFUtils.getURDFFileStatus(this, "ur5.urdf.xml");
            debugLog += status;

            // Load robot using utility
            robot = URDFUtils.loadDefaultRobot(this);
            robot.robot_init();
            canvasView.setJointPoses(robot.T0i);

            if (robot != null)
            {
                //debug.setText(robot.toDebugString()+"\n"+log+"\n");
                joints = robot.joints;
                createJointSliders();
            }
            else
            {
                //debug.setText("Failed to load robot\n" + URDFUtils.getAvailableURDFFilesString(this));
            }
            canvasView.post(new Runnable() {
                    @Override
                    public void run()
                    {
                        canvasView.zoomToFit();
                    }
                });
            debug.setText(debugLog);

        }
        catch (Exception e)
        {
            debug.setText("Error: " + e.getMessage() + "\n" + URDFUtils.getAvailableURDFFilesString(this));
        }
    }
	private void resetToDefault(View v)
    {
        for (Joint joint : joints)
        {
            SeekBar slider = sliderMap.get(joint.name);
            TextView valueView = valueMap.get(joint.name);

            if (slider != null)
            {
                int progress = mapValueToProgress(joint.getDefaultPosition(), 
												  joint.getMinPosition(), 
												  joint.getMaxPosition());
                slider.setProgress(progress);

                joint.setCurrentPosition(joint.getDefaultPosition());
                double deg = Math.toDegrees(joint.getDefaultPosition());
                valueView.setText(String.format("%.2f°", deg));
            }
        }

        // Update robot visualization
        applyJointAngles(v);
    }

	private void applyJointAngles(View v)
    {
        appendTitle("entering apply joint angles",true);
        for (Joint joint : joints)
        {
            SeekBar slider = sliderMap.get(joint.name);
            TextView valueView = valueMap.get(joint.name);

            if (slider != null)
            {
                int progress = slider.getProgress();
                double radians = mapProgressToValue(progress, 
                                                    joint.getMinPosition(), 
                                                    joint.getMaxPosition());

                joint.setCurrentPosition(radians);
                double deg = Math.toDegrees(radians);
                valueView.setText(String.format("%.2f°", deg));
            }
        }
        // 1. Get all current joint angles from the sliders
        double[] thetaList = new double[joints.size()];
        for (int i = 0; i < joints.size(); i++)
        {
            thetaList[i] = joints.get(i).getCurrentPosition();
        }
        Vector thetaL=new Vector(thetaList);
        appendLog("thetalist="+EL+thetaL.toString()+EL,true);
        updateLogDisplay();
        // 2. Recalculate the new T0i poses based on these angles
        robot.calculateJointPositions(thetaL);

        // 3. Filter the new poses and update the Canvas
        ArrayList<se3group> filteredPoses = new ArrayList<>();
        for (int i = 1; i < robot.T0i.size(); i++) {
            filteredPoses.add(robot.T0i.get(i));
        }

        // 4. Update the data immediately
        canvasView.setJointPoses(filteredPoses);

        // 5. Force an immediate redraw (so the robot moves to the new position)
        canvasView.invalidate(); 

        // 6. Use .post() to safely re-center and re-zoom ONLY AFTER the layout is done
        canvasView.post(new Runnable() {
                @Override
                public void run() {
                    canvasView.zoomToFit();
                }
            });
    }
    private void createJointSliders()
	{
        LayoutInflater inflater = LayoutInflater.from(this);

        for (Joint joint : joints)
		{
            // Inflate the slider item
            View sliderView = inflater.inflate(R.layout.item_joint_slider, 
											   slidersContainer, false);
            // Get references
            TextView nameView = sliderView.findViewById(R.id.joint_name);
            final TextView valueView = sliderView.findViewById(R.id.joint_value);
            TextView rangeView = sliderView.findViewById(R.id.joint_range);
            SeekBar slider = sliderView.findViewById(R.id.joint_slider);

            // Set joint name
            nameView.setText(joint.name);

            // Convert radians to degrees for display
            double minDeg = Math.toDegrees(joint.getMinPosition());
            double maxDeg = Math.toDegrees(joint.getMaxPosition());
            double defaultDeg = Math.toDegrees(joint.getDefaultPosition());

            // Set range display
            rangeView.setText(String.format("Min: %.1f°  Max: %.1f°", minDeg, maxDeg));

            // Set initial value
            int progress = mapValueToProgress(joint.getDefaultPosition(), 
											  joint.getMinPosition(), 
											  joint.getMaxPosition());
            slider.setProgress(progress);
            valueView.setText(String.format("%.2f°", defaultDeg));

            // Store references
            sliderMap.put(joint.name, slider);
            valueMap.put(joint.name, valueView);

            // Set listener
            final Joint finalJoint = joint;
            slider.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
					@Override
					public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser)
					{
						double value = mapProgressToValue(progress, 
														  finalJoint.getMinPosition(), 
														  finalJoint.getMaxPosition());
						finalJoint.setCurrentPosition(value);


						double deg = Math.toDegrees(value);
						valueView.setText(String.format("%.2f°", deg));
					}

					@Override
					public void onStartTrackingTouch(SeekBar seekBar)
					{}

					@Override
					public void onStopTrackingTouch(SeekBar seekBar)
					{}
				});

            // Add to container
            slidersContainer.addView(sliderView);
        }
    }

    // Map progress (0-1000) to actual joint angle
    private double mapProgressToValue(int progress, double min, double max)
	{
        return min + (progress / 1000.0) * (max - min);
    }

    // Map actual joint angle to progress (0-1000)
    private int mapValueToProgress(double value, double min, double max)
	{
        return (int) (((value - min) / (max - min)) * 1000);
    }

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
    public void updateLogDisplay()
    { 
        debug.setText(debugLog);
    }
}
