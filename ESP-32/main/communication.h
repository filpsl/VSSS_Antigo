	/**********************************************************************************************/
/*                                                                                            */
/*                                                                                            */
/*        communication.h                              Author  : Luiz Felipe                  */
/*                                                     Email   :                              */
/*                                                     address : DF, BRAZIL                   */
/*        Created: 2023/02/26          by Luiz F.                                             */
/*        Updated: 2024/09/13          by Luiz F.                                             */
/*                                                                       All rights reserved  */
/**********************************************************************************************/

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <WiFi.h>

/**
 * @brief Class that manages WiFi communication for ESP32.
 * 
 * The Communication class handles the WiFi network setup and data exchange
 * between the ESP32 and a client connected via TCP. It allows receiving 
 * combined 32-bit data and sending data in the same format.
 */
class Communication {
public:
    /**
     * @brief Constructor for the Communication class.
     * 
     * Initializes the WiFi communication with the specified SSID, password, and port.
     * 
     * @param ssid The WiFi network name (SSID).
     * @param password The WiFi network password.
     * @param port The TCP port to be used for communication.
     */
    Communication(const char* ssid, const char* password, uint32_t port);

    /**
     * @brief Starts the WiFi communication and server.
     * 
     * This function must be called to connect to the WiFi network and start the server
     * that will accept connections from clients.
     */
    void begin();

    /**
     * @brief Receives data from a client.
     * 
     * This function waits for the client to send a 32-bit data value. If data is available,
     * it will be received and returned. If no data is available, it returns 0xFFFFFFFF.
     * 
     * @return uint32_t The value received from the client, or 0xFFFFFFFF if no data is available.
     */
    uint32_t receiveData();

    
    uint32_t waitData();

    /**
     * @brief Sends data to the client.
     * 
     * This function sends a 32-bit value to the connected client.
     * 
     * @param value The 32-bit value to be sent.
     */
    void sendData(uint32_t value);


private:
    const char* ssid_;       /**< WiFi network name (SSID). */
    const char* password_;   /**< WiFi network password. */
    uint16_t port_;          /**< TCP port used for communication. */
    WiFiServer server_;      /**< WiFi server used to accept client connections. */
    WiFiClient client_;      /**< WiFi client used for communication with the client. */
};

#endif
