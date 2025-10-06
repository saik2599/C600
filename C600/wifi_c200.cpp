/*
   Author: Christopher Glanville
   Date: 14/4/24
   updated : 3/4/24
   WiFi connection handled class for Arduino Giga. 
*/

#include <SPI.h>
#include <WiFi.h>

#include "C200.h"
#include "modbus_c200.h"

#define   MAX_WIFI_CONNECT_RETRIES      5

#if defined(HARDWARE_GIGA)
// WiFi SSID and Password
char ssid[] = "One H 2";              // your network SSID (name)
char pass[] = "HGmLBJvC3QKCcb7j";     // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;          // the WiFi radio's status

WiFiServer server(502);               // MODBUS Server port
boolean alreadyConnected = false;     // whether or not the client was connected previously

unsigned long wiFiSocketDataTiming = 0;


/*
    Name:         printMacAddress
    Arguments:    uint8_t*    A byte poiner to the 4 mac address bytes 
    Returns:      void        
    Description:  For simple diagnostics it is sometimes useful to know the MAC address of the radio module as it can be used in some network routers 
                  for NAT and DHCP configurations. 
*/
void printMacAddress(byte mac[]) {
  for (int i = 5; i >= 0; i--) {
    if (mac[i] < 16) {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
    if (i > 0) {
      Serial.print(":");
    }
  }
  Serial.println();
}

/*
    Name:         printCurrentNet
    Arguments:    void
    Returns:      nil        
    Description:  Print the details of the current WiFi network we are attached to. 
*/
void printCurrentNet() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print the MAC address of the router you're attached to:
  byte bssid[6];
  WiFi.BSSID(bssid);
  Serial.print("BSSID: ");
  printMacAddress(bssid);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.println(rssi);

  // print the encryption type:
  byte encryption = WiFi.encryptionType();
  Serial.print("Encryption Type:");
  Serial.println(encryption, HEX);
  Serial.println();
}

/*
    Name:         printEncryptionType
    Arguments:    void
    Returns:      nil        
    Description:  Print the encryption details of the current WiFi network we are attached to. 
*/
void printEncryptionType(int thisType) {
  // read the encryption type and print out the name:
  switch (thisType) {
    case ENC_TYPE_WEP:
      Serial.println("WEP");
      break;
    case ENC_TYPE_TKIP:
      Serial.println("WPA");
      break;
    case ENC_TYPE_CCMP:
      Serial.println("WPA2");
      break;
    case ENC_TYPE_NONE:
      Serial.println("None");
      break;
    case ENC_TYPE_AUTO:
      Serial.println("Auto");
      break;
    case ENC_TYPE_UNKNOWN:
    default:
      Serial.println("Unknown");
      break;
  }
}

/*
    Name:         listNetworks
    Arguments:    void
    Returns:      nil        
    Description:  Print the details of the networks that have been discovered
*/
void listNetworks() {
  // scan for nearby networks:
  Serial.println("** Scan Networks **");
  int numSsid = WiFi.scanNetworks();
  if (numSsid == -1) {
    Serial.println("Couldn't get a WiFi connection");
    return;
  }

  // print the list of networks seen:
  Serial.print("number of available networks:");
  Serial.println(numSsid);

  // print the network number and name for each network found:
  for (int thisNet = 0; thisNet < numSsid; thisNet++) {
    Serial.print(thisNet);
    Serial.print(") ");
    Serial.print(WiFi.SSID(thisNet));
    Serial.print("\tSignal: ");
    Serial.print(WiFi.RSSI(thisNet));
    Serial.print(" dBm");
    Serial.print("\tEncryption: ");
    printEncryptionType(WiFi.encryptionType(thisNet));
  }
}

/*
    Name:         printWifiData
    Arguments:    void
    Returns:      nil        
    Description:  For simple diagnostics print the IP address that was assigned during DHCP
*/
void printWifiData() {
  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  Serial.println(ip);

  // print your MAC address:
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("MAC address: ");
  printMacAddress(mac);
}


/*
    Name:         wifi_init
    Arguments:    void
    Returns:      nil
    Description:  Initialize the radio module for WiFi and look for a known SSID that we might be able to connect with
*/
void wifi_init() {
  int attempt = 0;

  #if defined(DEBUG_MODE)
    Serial.println("Scanning available networks...");
    listNetworks();
  #endif

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    return;
  }

  // attempt to connect to WiFi network:
  while ((status != WL_CONNECTED) && (attempt < MAX_WIFI_CONNECT_RETRIES)) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
    attempt++;    // Increment the attempt number so that we don't block here if no WiFi is available. 
  }

  if (attempt >= MAX_WIFI_CONNECT_RETRIES){
    Serial.print("Failed to connecto to the WiFi network");
  }else{
    // you're connected now, so print out the data:
    Serial.print("You're connected to the network");
    printCurrentNet();
    printWifiData();

    // start the server:
    server.begin();
  }
  return;
}

/*
    Name:         wifi_loop
    Arguments:    void
    Returns:      nil
    Description:  The main data processing loop of the WiFi class. This function must be called regularly as any data that is received 
                  from the WiFi connection and the connection itself will be processed here. 
                  IMPORTANT! This is a blocking function - ARGH!?!??!!?!! Once the connection is made this function 
                  will no longer return. The only way I have managed to get around this was to call a second loop_two()
                  function once the connection is made. This will cause the stack to always have a depth of 1 while the WiFi
                  interface is connected, however THERE WAS NO OTHER SOLUTION THAT COULD BE FOUND!!
*/
void wifi_loop()
{
  WiFiClient client = server.available();   // This connection status will presist, even when the returned client object goes out of scope
  
  // Check to see if the client is valid
  while (client) {
    if (!alreadyConnected) {
      // clear out the input buffer:
      client.flush();
      Serial.println("We have a new client");
      alreadyConnected = true;
      wiFiSocketDataTiming = millis();    // Reset the timer on connection, otherwise it will be immediatly disconnected if t>15s
    }

    while(client.available() > 0){
      wiFiSocketDataTiming = millis();    // Reset the timeout timer every time new data is received on the socket
      // read the bytes incoming from the client, and add them to the MODBUS processing buffer
      char thisChar = client.read();
      modbuxRxData(thisChar, TCP_BUFFER);
      
      #if defined(DEBUG_MODE)
        // echo the bytes to the server as well:
        Serial.write(thisChar);
      #endif
    }
    // Process the buffer for any new incoming packets

    if (modbus_loop(TCP_BUFFER)){ 
      int tx_bytes = get_tx_bytes(TCP_BUFFER);
      client.write(get_tx_buffer(TCP_BUFFER), tx_bytes);

      #if defined(DEBUG_MODE)
        Serial.write(get_tx_buffer(TCP_BUFFER), tx_bytes);
      #endif
    }

    // if the server's disconnected, stop the client:
    if (!client.connected()) {
      Serial.println();
      Serial.println("TCP Client disconnected");
      client.stop();
      alreadyConnected = false;   // Reset ready for the next connection
    // If there is more than 15 seconds since the last data byte was received on the socket, DISCONNECT
    // This is required because Arduino does not handle the socket disconnection well and it can hang for a 
    // very long time which prevents any new connections. By putting this here we can allow a recovery mechanism
    }else if ((millis() - wiFiSocketDataTiming) >= 15000){
      Serial.println();
      Serial.println("TCP Client TIMEOUT disconnected");
      client.stop();
      alreadyConnected = false;   // Reset ready for the next connection
    }
 loop_two();

  }// End while connection loop
  return;   // Return from the WiFi loop - essential to allow for other processing
}

#endif    // if defined HARDWARE_GIGA