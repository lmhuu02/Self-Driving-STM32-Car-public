package com.example.bluetooth_stm32;


import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.Set;
import java.util.UUID;

import android.app.Activity;
import android.app.AlertDialog;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;

public class MainActivity extends Activity {
	private static final UUID MY_UUID = UUID
			.fromString("00001101-0000-1000-8000-00805F9B34FB");
	private static String deviceBt;
	private BroadcastReceiver mReceiver = null;
	
	private static boolean connected = false;
	public static final int CONNECTED = 6;
	private AlertDialog.Builder builder;
	 StringBuilder sb = new StringBuilder();
	
	InputStream mmInputStream;
	OutputStream mmOutputStream;
	BluetoothSocket mmSocket;

	private ConnectThread ct;

	private ConnectedThread ctd;
	private BluetoothAdapter mBluetoothAdapter;

	public static final int MESSAGE_RECEIVED = 2;
	private ArrayList<String> btResult = new ArrayList<String>();
	private ArrayList<BluetoothDevice> btResultDevices = new ArrayList<BluetoothDevice>();
	private ArrayAdapter<String> adapter;
	private IntentFilter filter;
	
	public static TextView receivedMessage;
	private static Button searchDevices, activateBt,send, quit,clear;
	private static EditText et1;
	
	private Thread getControl;

	 Handler mHandler = new Handler() {
	    	public void handleMessage(android.os.Message msg) {
	    		switch (msg.what) {
	            case MESSAGE_RECEIVED:		
	            	byte[] readBuf = (byte[]) msg.obj;
					// construct a string from the valid bytes in the buffer
					String readMessage = new String(readBuf, 0, msg.arg1);
					
					receivedMessage.setText(readMessage+"\n");
					break;
	            	
	            case CONNECTED:
					searchDevices.setText("Disconnect:"+deviceBt.split("\n")[0]);
					
					connected = true;
					break;
	            	
	    		}
	        };
		};
	     
	
	private static final int REQUEST_ENABLE_BT = 1;


	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_main);
		
		et1=(EditText)findViewById(R.id.text1);
		
		receivedMessage=(TextView)findViewById(R.id.textView1);
		
		mReceiver = new BroadcastReceiver() {
			@Override
			public void onReceive(Context context, Intent intent) {
				String action = intent.getAction();
				if (BluetoothDevice.ACTION_FOUND.equals(action)) {
					BluetoothDevice device = intent
							.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
					addDevice(device);
					adapter.notifyDataSetChanged();
				}
			}
		};
		filter = new IntentFilter(BluetoothDevice.ACTION_FOUND);
		registerReceiver(mReceiver, filter); 
		
		//=========================================================================//
		clear=(Button)findViewById(R.id.button_Clear);
		clear.setOnClickListener(new OnClickListener() {
			
			@Override
			public void onClick(View v) {
				receivedMessage.setText("");
				
			}
		});
		//======================================================================//
		quit = (Button) findViewById(R.id.button_quit);
		quit.setOnClickListener(new OnClickListener() {
			@Override
			public void onClick(View v) {
				quitApp();
			}
		});
		adapter = new ArrayAdapter<String>(this,
				android.R.layout.simple_list_item_1, btResult);
		
		
		builder = new AlertDialog.Builder(this);
		builder.setTitle("Chọn thiết bị:");
		builder.setAdapter(adapter, new DialogInterface.OnClickListener() {
			@Override
			public void onClick(DialogInterface dialog, int which) {
				deviceBt = btResult.get(which);
				Toast.makeText(getApplicationContext(),
						"Connection to :" + deviceBt,
						Toast.LENGTH_SHORT).show();
				ct = new ConnectThread(btResultDevices.get(which));
				ct.start();
			}
		});
		builder.setCancelable(true);
		builder.create();

		
		//==========================================================================//
		searchDevices = (Button) findViewById(R.id.button_search_devices);
		searchDevices.setOnClickListener(new OnClickListener() {

			@Override
			public void onClick(View v) {
				if (connected) {
					connected = false;
					searchDevices
							.setText("searchBt");
				} else {
					if (mBluetoothAdapter.startDiscovery()) {
						builder.show();
						Toast.makeText(getApplicationContext(),"Tìm thiết bị",Toast.LENGTH_SHORT).show();
					}
				}
			}
		});
		searchDevices.setEnabled(false);

		mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
		if (mBluetoothAdapter == null) {
		}

		Set<BluetoothDevice> pairedDevices = mBluetoothAdapter
				.getBondedDevices();
		if (pairedDevices.size() > 0) {
			for (BluetoothDevice device : pairedDevices) {
				addDevice(device);
			
		}
	}
		
		//===================================================================================//
		send=(Button)findViewById(R.id.button_send);
		send.setOnClickListener(new OnClickListener() {
			
			@Override
			public void onClick(View v) {
				
		      	ctd.write(et1.getText().toString().getBytes());	
				
			}
		});
		
		
		//=========================================================================================//
		activateBt = (Button)findViewById(R.id.button_bt);
		activateBt.setOnClickListener(new OnClickListener() {
			
			@Override
			public void onClick(View arg0) {
				if(!mBluetoothAdapter.isEnabled()){
					Intent turnOnIntent= new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);//cho phép kết nối bl
					startActivityForResult(turnOnIntent, REQUEST_ENABLE_BT);//phương thức dùng để kích hoạt bl nếu tr đó nó chưa được bấtk
				}
				
				
			}
		});
	}
	
	public void addDevice(BluetoothDevice device) {
		// First we check if the device already exists
		int index = -1;
		for (int i = 0; i < btResultDevices.size(); i++) {
			System.out.println();
			if (btResultDevices.get(i).getAddress().equals(device.getAddress())) {
				index = i;
				break;
			}
		}
		if (index < 0) {
			btResultDevices.add(device);
			btResult.add(device.getName() + "\n" + device.getAddress());
		} else {
			btResultDevices.remove(index);
			btResultDevices.add(index, device);
			btResult.remove(index);
			btResult.add(index, device.getName() + "\n" + device.getAddress());
		}

	}

	@Override
	public void onActivityResult(int requestCode, int resultCode, Intent data) {
		switch (requestCode) {
		case REQUEST_ENABLE_BT:
			if (resultCode == RESULT_CANCELED){
				Toast.makeText(getApplicationContext(),"Disable",Toast.LENGTH_SHORT).show();
			}else {
				Toast.makeText(getApplicationContext(),"Enable",Toast.LENGTH_SHORT).show();
				searchDevices.setEnabled(true);
			}
			break;
		}
	}

	@Override
	public void onStop() {
		if (mReceiver != null)
			unregisterReceiver(mReceiver);
		super.onStop();
	}

	@Override
	public void onDestroy() {
		super.onDestroy();
	}
	
	public void quitApp(){
		//Ask to switch off Bluetooth
	    AlertDialog.Builder builder = new AlertDialog.Builder(MainActivity.this);//tao danh sach hien thi 
        builder.setMessage("Are you sure you want to quit?")
               .setPositiveButton("Ok", new DialogInterface.OnClickListener() {
                   public void onClick(DialogInterface dialog, int id) {
                	   mBluetoothAdapter.disable();
                	   finish();
       				   System.exit(0);
                   }
               })
               .setNegativeButton("Cancel", new DialogInterface.OnClickListener() {
                   public void onClick(DialogInterface dialog, int id) {
                       // User cancelled the dialog
                	   mBluetoothAdapter.enable();
       				
                   }
               })
               .setCancelable(true);
        // Create the AlertDialog object and return it
        builder.create().show();
	}
	
	private class ConnectThread extends Thread {
	    private final BluetoothSocket mmSocket;
	    private final BluetoothDevice mmDevice;
	 
	    public ConnectThread(BluetoothDevice device) {
	        // Use a temporary object that is later assigned to mmSocket,
	        // because mmSocket is final
	        BluetoothSocket tmp = null;
	        mmDevice = device;
	 
	        // Get a BluetoothSocket to connect with the given BluetoothDevice
	        try {
	            // MY_UUID is the app's UUID string, also used by the server code
	            tmp = device.createRfcommSocketToServiceRecord(MY_UUID);
	        } catch (IOException e) { }
	        mmSocket = tmp;
	    }
	    public void run() {
	        // Cancel discovery because it will slow down the connection
	        mBluetoothAdapter.cancelDiscovery();
	 
	        try {
	            // Connect the device through the socket. This will block
	            // until it succeeds or throws an exception
	            mmSocket.connect();
	        } catch (IOException connectException) {
	            // Unable to connect; close the socket and get out
	            try {
	                mmSocket.close();
	            } catch (IOException closeException) { }
	            return;
	        }
	 
	        Message mes = mHandler.obtainMessage(CONNECTED, null);
			mHandler.sendMessage(mes);
			ctd = new ConnectedThread(mmSocket);
			ctd.start();
	    }
	 
	    /** Will cancel an in-progress connection, and close the socket */
	    public void cancel() {
	        try {
	            mmSocket.close();
	        } catch (IOException e) { }
	    }
	}
	private class ConnectedThread extends Thread {
	    private final BluetoothSocket mmSocket;
	    private final InputStream mmInStream;
	    private final OutputStream mmOutStream;
	 
	    public ConnectedThread(BluetoothSocket socket) {
	        mmSocket = socket;
	        InputStream tmpIn = null;
	        OutputStream tmpOut = null;
	 
	        // Get the input and output streams, using temp objects because
	        // member streams are final
	        try {
	            tmpIn = socket.getInputStream();
	            tmpOut = socket.getOutputStream();
	        } catch (IOException e) { }
	 
	        mmInStream = tmpIn;
	        mmOutStream = tmpOut;
	    }
	 
	    public void run() {
	        byte[] buffer = new byte[1024];  // buffer store for the stream
	        int bytes; // bytes returned from read()
	 
	        // Keep listening to the InputStream until an exception occurs
	        while (true) {
	            try {
	                // Read from the InputStream
	                bytes = mmInStream.read(buffer);
	                // Send the obtained bytes to the UI activity
	                mHandler.obtainMessage(MESSAGE_RECEIVED, bytes, -1, buffer)
	                        .sendToTarget();
	            } catch (IOException e) {
	                break;
	            }
	        }
	    }
	 
	    /* Call this from the main activity to send data to the remote device */
	    public void write(byte[] bytes) {
	        try {
	            mmOutStream.write(bytes);
	        } catch (IOException e) { }
	    }
	 
	    /* Call this from the main activity to shutdown the connection */
	    public void cancel() {
	        try {
	            mmSocket.close();
	        } catch (IOException e) { }
	    }
	}
	}
