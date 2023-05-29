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
 
#ifndef SOCKET_SRV_h
#define SOCKET_SRV_h

#include <Arduino.h>

#include "WiFiEsp.h"
#include "SoftwareSerial.h"

#include "sampling_ctrl.h" /* SamplingController */

#define ESP8266_RX 5
#define ESP8266_TX 4
#define ESP8266_BAUDRATE 9600u

#define SOCKET_SERVER_PORT 8080u
#define WIFI_SSID "xfrshome"
#define WIFI_PWD  "RaceHome@21$"

#define SOCKET_SERVER_SAMPLING_TIME_MILLIS  1000u
#define SOCKET_SERVER_CLIENT_TIMEOUT_MILLIS 5000u

/*
 * Data to send current measurements to the client
 */
enum SrvData {
  SRVDATA_DURATION_IDX,
  SRVDATA_SPEED_LEFT_IDX,
  SRVDATA_SPEED_RIGHT_IDX,
  SRVDATA_IR_LEFT_IDX,
  SRVDATA_IR_RIGHT_IDX,
  SRVDATA_CONTROL_LEFT_IDX,
  SRVDATA_CONTROL_RIGHT_IDX,
  SRVDATA_LEN
};


enum CliCmd {
  CLICMD_NONE,
  CLICMD_CONN_END,
  CLICMD_MOTOR_CTRL,
  CLICMD_PID_LEFT,
  CLICMD_PID_RIGHT,
  CLICMD_SPEED_CTRL,
  CLICMD_LEN
};

/*
 * Data used to test motor drive controls
 */
enum CliDataMotorCtrl {
  CLIDATA_MOTOR_CTRL_LEFT_IDX,
  CLIDATA_MOTOR_CTRL_RIGHT_IDX,
  CLIDATA_MOTOR_CTRL_LEN
};

/*
 * Data used to optimize PID parameters.
 */
enum CliDataPID {
  CLIDATA_PID_KP_IDX,
  CLIDATA_PID_KI_IDX,
  CLIDATA_PID_KD_IDX,
  CLIDATA_PID_TAU_IDX,
  CLIDATA_PID_LEN
};

/*
 * Data used to speed controller.
 */
enum CliDataSpeedCtrl {
  CLIDATA_SPEED_CTRL_TGT_LEFT_IDX,
  CLIDATA_SPEED_CTRL_TGT_RIGHT_IDX,
  CLIDATA_SPEED_CTRL_LEN
};

#define BYTES_PER_DATA      4u
#define CLIDATA_LEN_MAX     4u

class SocketSrv {
  private:      
    WiFiEspServer _server;   
    WiFiEspClient _client;   
    SoftwareSerial _esp;
    float _srv_data[SRVDATA_LEN];
    float _cli_data[CLIDATA_LEN_MAX];
    CliCmd _cli_cmd;
    long _esp_baudrate;
    SamplingController _sampler;
    SamplingController _client_timeout;

    bool _Read(Stream *p_debug);
    bool _Write();
    void _clientStop();
        

  public:
    SocketSrv(int esp_rx_pin, int esp_tx_pin, long esp_baudrate, unsigned int socket_port, unsigned long sampling_time, unsigned long client_timeout);
    void setSrvData(unsigned int idx, float value);
    void setCliData(unsigned int idx, float value);
    float getSrvData(unsigned int idx);
    float getCliData(unsigned int idx);
    CliCmd getCliCmd();
    void beginEsp();
    void beginServer();
    bool hasClient();
    bool Listen();
    bool Talk(Stream *p_debug);
//////////////////////////////////////////////
// Sniffer Robot
    static SocketSrv socket;
    static void Initialize(Stream *p_debug);
//////////////////////////////////////////////    
};




#endif
