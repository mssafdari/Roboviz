package com.roboviz;

import java.util.*;
import android.widget.*;
import android.os.Bundle;
import java.io.InputStream;
import android.app.Activity;
import android.view.View;
import android.view.LayoutInflater;
import android.content.Intent;

// RobotControlActivity.java
public class RobotControlActivity extends Activity
{
    private LinearLayout slidersContainer;
    private List<Joint> joints = new ArrayList<>();
    private Map<String, SeekBar> sliderMap = new HashMap<>();
    private Map<String, TextView> valueMap = new HashMap<>();

    @Override
    protected void onCreate(Bundle savedInstanceState)
	{
        super.onCreate(savedInstanceState);
        setContentView(R.layout.robot_view);

        slidersContainer = findViewById(R.id.sliders_container);
        // Load and parse URDF
		// ✅ STEP 1: Receive joints from Intent
        receiveJointsFromIntent();

        // ✅ STEP 2: Create sliders from received joints
        if (joints != null && !joints.isEmpty())
        {
            createJointSliders();
        }
        else
        {
            // If no joints received, use sample joints
            addSampleJoints();
        }

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
    }

	// In RobotControlActivity - Receiving data
    private void receiveJointsFromIntent()
    {
        Intent intent = getIntent();

        if (intent != null && intent.hasExtra("joints"))
        {
            // ✅ Use getParcelableArrayListExtra
            ArrayList<Joint> receivedJoints = intent.getParcelableArrayListExtra("joints");
            if (receivedJoints != null)
            {
                joints = receivedJoints;
                Toast.makeText(this, "Received " + joints.size() + " joints", Toast.LENGTH_SHORT).show();
            }
        }
        else
        {
            Toast.makeText(this, "No joint data received", Toast.LENGTH_SHORT).show();
            addSampleJoints();
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
        updateRobot();
    }

	private void applyJointAngles(View v)
    {
        // Get all current joint angles
        double[] thetaList = new double[joints.size()];
        for (int i = 0; i < joints.size(); i++)
        {
            thetaList[i] = joints.get(i).getCurrentPosition();
        }

        // Here you would update your robot model
        // For example, pass to your IK solver or 3D viewer
        updateRobotWithJoints(thetaList);

        Toast.makeText(this, "Applied joint angles to robot", Toast.LENGTH_SHORT).show();
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

    private void resetToDefault()
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
        updateRobot();
    }

    private void applyJointAngles()
	{
        // Get all current joint angles
        double[] thetaList = new double[joints.size()];
        for (int i = 0; i < joints.size(); i++)
		{
            thetaList[i] = joints.get(i).getCurrentPosition();
        }

        // Here you would update your robot model
        // For example, pass to your IK solver or 3D viewer
        updateRobotWithJoints(thetaList);

        Toast.makeText(this, "Applied joint angles to robot", Toast.LENGTH_SHORT).show();
    }

    private void updateRobot()
	{
        // Update 3D visualization with current joint angles
        // This depends on your robot visualization library
    }

    private void updateRobotWithJoints(double[] thetaList)
	{
        // Update robot model with joint angles
        // Example: call your kinematics solver or update 3D scene
        for (int i = 0; i < joints.size(); i++)
		{
            String jointName = joints.get(i).name;
            double angle = thetaList[i];
            // Update your 3D model here
            //Log.d("RobotControl", "Joint: " + jointName + " -> " + Math.toDegrees(angle) + "°");
        }
    }

    private void addSampleJoints()
	{
        // For testing when no URDF is available
        joints.add(new Joint("joint1", 0.0, -3.14, 3.14, "revolute"));
        joints.add(new Joint("joint2", 0.5, -2.0, 2.0, "revolute"));
        joints.add(new Joint("joint3", -0.3, -1.5, 1.5, "revolute"));
        joints.add(new Joint("joint4", 0.0, -0.5, 0.5, "prismatic"));
        createJointSliders();
    }
}
