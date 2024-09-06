#include "warble/warble.h"
#include <string>
#include <chrono>
#include <future>
#include <iostream>
#include <thread>

void connect_to_sensor(const std::string& mac_address) {
    WarbleGatt* gatt = nullptr;
    
    // Initialize the Warble GATT connection
    warble_gatt_connect_async(mac_address.c_str(), &gatt, [](WarbleGatt* gatt, WarbleError* error) {
        if (error == nullptr) {
            std::cout << "Connected to sensor!" << std::endl;
            
            // Here you would continue to discover services and characteristics
            // and interact with the sensor.
        } else {
            std::cerr << "Failed to connect: " << warble_error_message(error) << std::endl;
        }
    });
    
    // Keep the program running to maintain the connection
    std::this_thread::sleep_for(std::chrono::seconds(10));
    
    if (gatt) {
        warble_gatt_disconnect(gatt);
        warble_gatt_delete(gatt);
    }
}

int main() {
    std::string sensor_mac = "EE:1B:72:FA:BF:E8";  // Replace with your sensor's MAC address
    connect_to_sensor(sensor_mac);
    
    return 0;
}
