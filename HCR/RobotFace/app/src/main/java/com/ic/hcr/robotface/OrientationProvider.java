package com.ic.hcr.robotface;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.util.Log;

/**
 * Created by Cian on 06/12/2015.
 */
public class OrientationProvider implements SensorEventListener {
    private static final String TAG = "OrientationProvider";

    private static final double CHANGE_THRESHOLD = 10.0f;

    private SensorManager sensorManager;
    private Sensor accelerometer, magnetometer;

    private double currentOrientation = 0.0f;

    private float[] gravity;
    private float[] magneticField;

    private OrientationCallback orientationCallback;

    private Context context;

    public OrientationProvider(Context context, OrientationCallback orientationCallback) {
        this.context = context;
        this.orientationCallback = orientationCallback;

        sensorManager = (SensorManager) context.getSystemService(Context.SENSOR_SERVICE);
        accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        magnetometer = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
    }

    public void connect() {
        sensorManager.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_NORMAL);
        sensorManager.registerListener(this, magnetometer, SensorManager.SENSOR_DELAY_NORMAL);
    }

    public void disconnect() {
        sensorManager.unregisterListener(this);
    }

    private void updateDirection() {
        float[] temp = new float[9];
        float[] R = new float[9];

        SensorManager.getRotationMatrix(temp, null, gravity, magneticField);
        SensorManager.remapCoordinateSystem(temp, SensorManager.AXIS_X, SensorManager.AXIS_Z, R);

        float[] values = new float[3];
        SensorManager.getOrientation(R, values);

        double newOrientation = (values[0] * 180) / Math.PI; // Only take change in the Z axis for azimuth change when tablet is up right.
        if(Math.abs(currentOrientation - newOrientation) > CHANGE_THRESHOLD) {
            currentOrientation = newOrientation;
            orientationCallback.handleNewOrientation(newOrientation);
        }
    }


    @Override
    public void onSensorChanged(SensorEvent event) {
        switch(event.sensor.getType()) {
            case Sensor.TYPE_ACCELEROMETER:
                gravity = event.values.clone();
                break;
            case Sensor.TYPE_MAGNETIC_FIELD:
                magneticField = event.values.clone();
                break;
            default:
                return;
        }

        if(gravity != null && magneticField != null) {
            updateDirection();
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // Do nothing.
    }

    public abstract interface OrientationCallback {
        public void handleNewOrientation(double orientation);
    }
}
