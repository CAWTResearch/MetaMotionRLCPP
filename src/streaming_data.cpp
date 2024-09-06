// compile: g++ -o connect_and_stream connect_and_stream.cpp -std=c++14 -Isrc -Ldist/release/lib/x64 -lwarble -lmetawear

#include "warble/warble.h"


#include "metawear/core/datasignal.h"
#include "metawear/core/status.h"
#include "metawear/core/types.h"
#include "metawear/core/data.h"
#include "metawear/core/metawearboard.h"
#include "metawear/sensor/accelerometer.h"

#include <chrono>
#include <future>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <utility>
#include <unistd.h> 
#include <stdio.h>

using namespace std;
using namespace std::chrono;
using namespace std::chrono_literals;


void enable_acc_sampling(MblMwMetaWearBoard* board) {
    // Set ODR to 25Hz or closest valid frequency
    mbl_mw_acc_set_odr(board, 25.f);

    // Set range to +/-4g or closest valid range
    mbl_mw_acc_set_range(board, 4.f);

    // Write the config to the sensor
    mbl_mw_acc_write_acceleration_config(board);

    auto data_handler = [](void* context, const MblMwData* data) -> void {
        // Cast value to MblMwCartesianFloat*
        auto acceleration = (MblMwCartesianFloat*)data->value;
        printf("(%.3fg, %.3fg, %.3fg)\n", acceleration->x, acceleration->y, acceleration->z);
    };

    auto acc_signal = mbl_mw_acc_get_acceleration_data_signal(board);
    mbl_mw_datasignal_subscribe(acc_signal, nullptr, data_handler);

    mbl_mw_acc_enable_acceleration_sampling(board);
    mbl_mw_acc_start(board);
}

void disable_acc_sampling(MblMwMetaWearBoard* board) {
    auto acc_signal = mbl_mw_acc_get_acceleration_data_signal(board);
    mbl_mw_datasignal_unsubscribe(acc_signal);

    mbl_mw_acc_disable_acceleration_sampling(board);
    mbl_mw_acc_stop(board);
}

int main(int argc, char** argv) {
    if (argc < 2) {
        cerr << "usage: connect_and_stream [device mac] [hci mac](optional)" << endl;
        return 1;
    }

    // Setup GATT connection
    WarbleOption config_options[2] = {
        {"mac", argv[1]}
    };
    if (argc >= 3) {
        config_options[1].key = "hci";
        config_options[1].value = argv[2];
    }

    auto gatt = warble_gatt_create_with_options(argc - 1, config_options);

    promise<void> connect_task;
    cout << "Connecting to " << argv[1] << endl;
    warble_gatt_connect_async(gatt, &connect_task, [](void* context, WarbleGatt* caller, const char* value) {
        auto task = (promise<void>*)context;

        if (value != nullptr) {
            cerr << "Connection error: " << value << endl;
            task->set_exception(make_exception_ptr(runtime_error(value)));
        } else {
            cout << "Connected to device." << endl;
            task->set_value();
        }
    });
    connect_task.get_future().get();

    sleep(5);

    promise<int32_t> dc_task;
    cout << "Disconnecting..." << endl;
    warble_gatt_on_disconnect(gatt, &dc_task, [](void* context, WarbleGatt* caller, int32_t status) {
        ((promise<int32_t>*) context)->set_value(status);
    });
    warble_gatt_disconnect(gatt);
    cout << "disconnected, status = " << dc_task.get_future().get() << endl;
    
    cout << "Am I connected? " << warble_gatt_is_connected(gatt) << endl;
}



   


//     // Initialize the board
//     mbl_mw_metawearboard_initialize(board, [](MblMwMetaWearBoard* board, int32_t status) {
//     if (status == 0) {
//         // Initialization successful
//     } else {
//         // Handle initialization failure
//     }



//     // Initialize MetaWear board
//     // auto board = mbl_mw_metawearboard_create(gatt);
//     // mbl_mw_metawearboard_initialize(board, nullptr, [](void* context, MblMwMetaWearBoard* caller, int32_t status) {
//     //     if (status != MBL_MW_STATUS_OK) {
//     //         cerr << "Failed to initialize board (" << status << ")" << endl;
//     //     } else {
//     //         cout << "Board initialized" << endl;
//     //     }
//     // });

//     // Give some time for board initialization
//     this_thread::sleep_for(1s);

//     // Start accelerometer data sampling
//     enable_acc_sampling(board);

//     // Stream for 10 seconds (adjust as needed)
//     this_thread::sleep_for(10s);

//     // Stop accelerometer data sampling
//     disable_acc_sampling(board);

//     // Clean up and disconnect
//     mbl_mw_metawearboard_free(board);
//     promise<int32_t> dc_task;
//     cout << "Disconnecting..." << endl;
//     warble_gatt_on_disconnect(gatt, &dc_task, [](void* context, WarbleGatt* caller, int32_t status) {
//         ((promise<int32_t>*)context)->set_value(status);
//     });
//     warble_gatt_disconnect(gatt);
//     cout << "Disconnected, status = " << dc_task.get_future().get() << endl;

//     return 0;
// }
