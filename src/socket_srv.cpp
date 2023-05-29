/*
* Author: Rafael Rodrigues da Silva
* Date: May 13, 2023
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "socket_srv.h" 

SocketSrv SocketSrv::socket(ESP8266_RX, ESP8266_TX, ESP8266_BAUDRATE, SOCKET_SERVER_PORT, SOCKET_SERVER_SAMPLING_TIME_MILLIS, SOCKET_SERVER_CLIENT_TIMEOUT_MILLIS);


void SocketSrv::Initialize(Stream *p_debug) {
  int status;     // the Wifi radio's status
  
  // initialize serial for ESP module
  SocketSrv::socket.beginEsp();

  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    if (p_debug != NULL) {
      p_debug->println("WiFi shield not present");
    }
    // don't continue
    while (true);
  }

  // attempt to connect to WiFi network
  status = WL_IDLE_STATUS; 
  while (status != WL_CONNECTED) {
    if (p_debug != NULL) {
      p_debug->print("Attempting to connect to WPA SSID: ");
      p_debug->println(WIFI_SSID);
    }
    // Connect to WPA/WPA2 network
    status = WiFi.begin(WIFI_SSID, WIFI_PWD);
  }

  if (p_debug != NULL) {
    p_debug->println("You're connected to the network");
    // print the SSID of the network you're attached to
    p_debug->print("SSID: ");
    p_debug->println(WiFi.SSID());

    // print your WiFi shield's IP address
    IPAddress ip = WiFi.localIP();
    p_debug->print("IP Address: ");
    p_debug->println(ip);  
  }
  
  // start the socket server
  SocketSrv::socket.beginServer();
}

SocketSrv::SocketSrv(int esp_rx_pin, int esp_tx_pin, long esp_baudrate, unsigned int socket_port, unsigned long sampling_time, unsigned long client_timeout) : 
            _esp(esp_rx_pin, esp_tx_pin), _server(socket_port), _client(255), _sampler(sampling_time), _client_timeout(client_timeout) {
  _esp_baudrate = esp_baudrate;
  _cli_cmd = CLICMD_NONE;
}


void SocketSrv::beginEsp() {
  _esp.begin(_esp_baudrate);
  
  // initialize ESP module
  WiFi.init(&_esp);
}


void SocketSrv::beginServer() {
  _server.begin();
}


void SocketSrv::setSrvData(unsigned int idx, float value) {
  _srv_data[idx] = value;  
}


void SocketSrv::setCliData(unsigned int idx, float value) {
  _cli_data[idx] = value;
}


float SocketSrv::getSrvData(unsigned int idx) {
  return _srv_data[idx];
}


float SocketSrv::getCliData(unsigned int idx) {
  return _cli_data[idx];
}


CliCmd SocketSrv::getCliCmd() {
  return _cli_cmd;
}


bool SocketSrv::_Read(Stream *p_debug) {   
  size_t data_sz_max = BYTES_PER_DATA * CLIDATA_LEN_MAX;
  unsigned char data[data_sz_max];
  size_t data_sz, data_sz_tgt;
  unsigned char header;
  float *pf_data;
  int i;

  header = (unsigned char)_client.read();
  if ((header < 0) || (header >= CLICMD_LEN)) {
    if (p_debug != NULL) {
      p_debug->print("SocketSrv::_Read: Invalid Command! command is ");
      p_debug->print(header);
      p_debug->print(" and should be between 0 and ");
      p_debug->print(CLICMD_LEN);
      p_debug->println(".");
    }
    return false;
  }
  _cli_cmd = (CliCmd)header;

  
  switch(_cli_cmd) {
    case CLICMD_MOTOR_CTRL:
      data_sz_tgt = BYTES_PER_DATA * CLIDATA_MOTOR_CTRL_LEN;
      data_sz = _client.read(data, data_sz_max);
      if (data_sz != data_sz_tgt) {
        if (p_debug != NULL) {
          p_debug->print("SocketSrv::_Read: Invalid Data Lenth! length is ");
          p_debug->print(data_sz);
          p_debug->print(" and should be ");
          p_debug->print(data_sz_tgt);
          p_debug->println(".");
        }
        return false;
      }
      pf_data = reinterpret_cast<float *>(data);
      for(i=0;i<CLIDATA_MOTOR_CTRL_LEN;i++) {
        setCliData(i, pf_data[i]);
      }
      break;
    case CLICMD_PID_LEFT:
    case CLICMD_PID_RIGHT:
      data_sz_tgt = BYTES_PER_DATA * CLIDATA_PID_LEN;
      data_sz = _client.read(data, data_sz_max);
      if (data_sz != data_sz_tgt) {
        if (p_debug != NULL) {
          p_debug->print("SocketSrv::_Read: Invalid Data Lenth! length is ");
          p_debug->print(data_sz);
          p_debug->print(" and should be ");
          p_debug->print(data_sz_tgt);
          p_debug->println(".");
        }
        return false;
      }
      pf_data = reinterpret_cast<float *>(data);
      for(i=0;i<CLIDATA_PID_LEN;i++) {
        setCliData(i, pf_data[i]);
      }
      break;
    case CLICMD_SPEED_CTRL:
      data_sz_tgt = BYTES_PER_DATA * CLIDATA_SPEED_CTRL_LEN;
      data_sz = _client.read(data, data_sz_max);
      if (data_sz != data_sz_tgt) {
        if (p_debug != NULL) {
          p_debug->print("SocketSrv::_Read: Invalid Data Lenth! length is ");
          p_debug->print(data_sz);
          p_debug->print(" and should be ");
          p_debug->print(data_sz_tgt);
          p_debug->println(".");
        }
        return false;
      }
      pf_data = reinterpret_cast<float *>(data);
      for(i=0;i<CLIDATA_SPEED_CTRL_LEN;i++) {
        setCliData(i, pf_data[i]);
      }
      break;
    default:
      break;
  }
  return true;
}


bool SocketSrv::_Write() {
  size_t msg_sz_tgt = SRVDATA_LEN * BYTES_PER_DATA; 
  unsigned char msg[msg_sz_tgt], *pb_srv_data;
  size_t msg_sz;
  int i;

  pb_srv_data = reinterpret_cast<unsigned char *>(_srv_data);
  for(i=0;i<msg_sz_tgt;i++) {
    msg[i] = pb_srv_data[i];
  }

  msg_sz = _client.write(msg, msg_sz_tgt);

  return msg_sz == msg_sz_tgt;
}


void SocketSrv::_clientStop() {
  _client.flush();
  _client.stop();
  _cli_cmd = CLICMD_NONE;
}


bool SocketSrv::Listen() {
  if (_sampler) {
    if (!_client) {
      _client = SocketSrv::_server.available();
      if (!_client) {
        return false;
      }
      _client_timeout.Initialize();
    }    
    return true;
  }
  return false;
}

bool SocketSrv::Talk(Stream *p_debug) {
  if (!_client.available()) {
    if (_client_timeout) {
      if (p_debug != NULL) {
        p_debug->println("SocketSrv: No data from Client Timeout!");
      }      
      _clientStop();      
    } else if (p_debug != NULL) {
      p_debug->print("SocketSrv: No data from Client for ");
      p_debug->print(_client_timeout.getElapsedTimeInMillis());
      p_debug->println(" msec.");
    }
    return false;
  } else {
    _client_timeout.Initialize();
  }
  if (!_Read(p_debug)) {
    if (p_debug != NULL) {
      p_debug->println("SocketSrv: Read Failed!");
    }
    _clientStop();
    return false;
  }  
  if (getCliCmd() == CLICMD_CONN_END) {
    if (p_debug != NULL) {
      p_debug->println("SocketSrv: Client ended connection.");
    }
    _clientStop();
    return true;
  }
  _client.flush();
  if (!_Write()) {
    if (p_debug != NULL) {
      p_debug->println("SocketSrv: Write Failed!");
    }
    _clientStop();
    return false;
  }
  return true;
}

bool SocketSrv::hasClient() {
  return _client == true;  
}
