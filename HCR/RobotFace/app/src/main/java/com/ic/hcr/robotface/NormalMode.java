package com.ic.hcr.robotface;

import android.annotation.SuppressLint;
import android.location.Location;
import android.support.v7.app.ActionBar;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;

import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.MapView;
import com.google.android.gms.maps.OnMapReadyCallback;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.Polyline;
import com.google.android.gms.maps.model.PolylineOptions;

import java.util.ArrayList;
import java.util.List;

/**
 * An example full-screen activity that shows and hides the system UI (i.e.
 * status bar and navigation/system bar) with user interaction.
 */
public class NormalMode extends FaceMode implements OnMapReadyCallback, FaceMode.NormalModeCallback {
    private static final String TAG = "NormalMode";

    private MapView mapView;
    private FaceView faceView;

    private boolean mapViewEnabled;

    private Polyline polyLine;
    private PolylineOptions polyLineOptions;
    private LatLng currentLocation;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        Log.i(TAG, "Started Normal Mode");
        this.requestWindowFeature(Window.FEATURE_NO_TITLE);
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_normal_mode);
        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN, WindowManager.LayoutParams.FLAG_FULLSCREEN);
        setNormalModeCallback(this);
        mapView = (MapView) findViewById(R.id.mapView);
        faceView = (FaceView) findViewById(R.id.faceView);
        mapView.getMapAsync(this);
    }

    @Override
    public void onResume() {
        View decorView = getWindow().getDecorView();
        int uiOptions = View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
                | View.SYSTEM_UI_FLAG_FULLSCREEN;
        decorView.setSystemUiVisibility(uiOptions);
        super.onResume();
    }

    private void setMapViewEnabled(boolean mapViewEnabled) {
        if(mapViewEnabled) {
            mapView.setVisibility(View.VISIBLE);
            faceView.setVisibility(View.GONE);
        } else {
            mapView.setVisibility(View.GONE);
            faceView.setVisibility(View.VISIBLE);
        }
    }

    @Override
    public void onMapReady(GoogleMap googleMap) {
        Log.d(TAG, "Map ready.");
        googleMap.setMyLocationEnabled(true);
        googleMap.moveCamera(CameraUpdateFactory.newLatLngZoom(currentLocation, 13));
        List<LatLng> latLng = decodePoly("okiyHb`a@dLqAd@Q`Es@|BQfBYDb@RpB");
        Log.d(TAG, "Found: " + latLng.size() + " way points.");
        polyLineOptions.addAll(latLng);
        polyLine = googleMap.addPolyline(polyLineOptions);
        setMapViewEnabled(mapViewEnabled);
    }

    @Override
    public void getMap(boolean mapViewEnabled) {
        mapView.getMapAsync(this);
        this.mapViewEnabled = mapViewEnabled;
    }

    @Override
    public void setCurrentLocation(Location currentLocation) {
        this.currentLocation = new LatLng(currentLocation.getLatitude(), currentLocation.getLongitude());
    }

    private List<LatLng> decodePoly(String encoded) {
        List<LatLng> poly = new ArrayList<LatLng>();
        int index = 0, len = encoded.length();
        int lat = 0, lng = 0;

        while (index < len) {
            int b, shift = 0, result = 0;
            do {
                b = encoded.charAt(index++) - 63;
                result |= (b & 0x1f) << shift;
                shift += 5;
            } while (b >= 0x20);
            int dlat = ((result & 1) != 0 ? ~(result >> 1) : (result >> 1));
            lat += dlat;

            shift = 0;
            result = 0;
            do {
                b = encoded.charAt(index++) - 63;
                result |= (b & 0x1f) << shift;
                shift += 5;
            } while (b >= 0x20);
            int dlng = ((result & 1) != 0 ? ~(result >> 1) : (result >> 1));
            lng += dlng;

            LatLng p = new LatLng((((double) lat / 1E5)),
                    (((double) lng / 1E5)));
            poly.add(p);
        }
        return poly;
    }
}
