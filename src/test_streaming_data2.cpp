#include "warble/warble.h"
#include "warble/gattchar_fwd.h"
#include "warble/gatt_fwd.h"

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>

#include "metawear/core/datasignal.h"
#include "metawear/core/status.h"
#include "metawear/core/types.h"
#include "metawear/core/data.h"
#include "metawear/core/metawearboard.h"
#include "metawear/sensor/accelerometer.h"

#include <chrono>
#include <future>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <thread>
#include <utility>
#include <unistd.h> 
#include <stdio.h>

#include <cstdio>  // Para printf
#include <cstring> // Para snprintf

#include <unordered_map>
#include <inttypes.h>

using namespace std;
using namespace std::chrono;
using namespace std::chrono_literals;

using std::unordered_map;

static const char* MetaMotionMAC;
static const char* DongleBluetHCI;
int NumberArgc;
static WarbleGatt* gatt = nullptr;  // Inicializar gatt como nullptr

static MblMwBtleConnection MetaMotionBtleConnection;
static MblMwMetaWearBoard* board = nullptr;

static unordered_map<const void*, MblMwFnIntVoidPtrArray> notify_handlers;
static unordered_map<const void*, MblMwFnVoidVoidPtrInt> dc_handlers;

struct writeElement{
    WarbleGattChar* gattChar;
};

// enabling notification if GATT characteristic changes
struct enableNotifyContext
{
    std::promise<int>* task;
    const void* caller;
    MblMwFnIntVoidPtrArray handler;
    WarbleGattChar* gattChar;
};
static enableNotifyContext* s_enableNotifyContext;
static std::mutex enableNotifyMutex;

// Función para convertir los UUIDs de alto y bajo a una cadena UUID de 128 bits
std::string convert_uuid_to_string(uint64_t high, uint64_t low) {
    char uuid_str[37];  // 36 caracteres + terminador nulo
    snprintf(uuid_str, sizeof(uuid_str),
             "%08x-%04x-%04x-%04x-%012lx",
             static_cast<uint32_t>(high >> 32),
             static_cast<uint16_t>(high >> 16),
             static_cast<uint16_t>(high),
             static_cast<uint16_t>(low >> 48),
             static_cast<uint64_t>(low & 0xFFFFFFFFFFFFULL));
    return std::string(uuid_str);
}

static void connectionWarbleGatt(){
    WarbleOption config_options[2] = {
        {"mac", MetaMotionMAC}
    };
    if (NumberArgc >= 3) {
        config_options[1].key = "hci";
        config_options[1].value = DongleBluetHCI;
    }

    gatt = warble_gatt_create_with_options(NumberArgc - 1, config_options);  // Asignar a la variable global gatt

    promise<void> connect_task;
    cout << "Connecting to " << MetaMotionMAC << endl;
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
}

//_____________________________________________________________________________________________________________________________________________________________________________
//                              WRITING CHARACTERISTICS
//_____________________________________________________________________________________________________________________________________________________________________________
static void write_gatt_char_callback(void* context, WarbleGattChar* characteristic, const char* error){
    if (error) {
        std::cerr << "Error writing the characteristic: " << error << std::endl;
    }
    else{
        std::cout << "Writing the characteristic complete" << std::endl;
    }
}

static void write_gatt_char(void *context, const void *caller, MblMwGattCharWriteType writeType, const MblMwGattChar *characteristic, const uint8_t *value, uint8_t length){
    // Convert UUID to string UUID
    std::string uuid_str = convert_uuid_to_string(characteristic->uuid_high, characteristic->uuid_low);

    WarbleGattChar* gattChar = warble_gatt_find_characteristic(gatt, uuid_str.c_str());
    
    // Verify if the connection exist
    if (gattChar == nullptr){
        std::cerr << "Error: Characteristic not found for UUID: " << uuid_str << std::endl;
        return;
    }

    // Write the charactetistic
    warble_gattchar_write_async(gattChar, value, length, nullptr, write_gatt_char_callback);
}
//_____________________________________________________________________________________________________________________________________________________________________________

//_____________________________________________________________________________________________________________________________________________________________________________
//                              READING CHARACTERISTICS
//_____________________________________________________________________________________________________________________________________________________________________________
static void read_gatt_char_callback(void* context, WarbleGattChar* characteristic, const uint8_t* value, uint8_t length, const char* error){
    if (error) {
        std::cerr << "Error reading the characteristic: " << error << std::endl;
    }
    else{
        std::cout << "Reading the characteristic complete, value: ";
        for(int i=0; i<length; ++i){
            std::cout << std::hex << (int)value[i] << " ";
        }
        std::cout << std::dec << std::endl;
    }
    // printf("Reading Characteristics Task Complete\n");
}

static void read_gatt_char(void *context, const void *caller, const MblMwGattChar *characteristic, MblMwFnIntVoidPtrArray handler){
    // Convert UUID to string UUID
    std::string uuid_str = convert_uuid_to_string(characteristic->uuid_high, characteristic->uuid_low);

    WarbleGattChar* gattChar = warble_gatt_find_characteristic(gatt, uuid_str.c_str());
    
    // Verify if the connection exist
    if (gattChar == nullptr){
        printf("Error: Characteristic not found for UUID: %s\n", uuid_str.c_str());
        handler(caller, nullptr, 0);
        return;
    }
    else{
        printf("Characteristics for UUID: %s\n", uuid_str.c_str());
    }
    
    // Read the charactetistic
    // warble_gattchar_read_async(gattChar, nullptr, read_gatt_char_callback);
    std::promise<void> waitReadTask; // Promise give me a value in any moment of the future
    warble_gattchar_read_async(gattChar, &waitReadTask, read_gatt_char_callback);
    waitReadTask.get_future().get(); // Wait until the promise is completed. void-> Block the thread until waitReadTask is completed
}
//_____________________________________________________________________________________________________________________________________________________________________________

//_____________________________________________________________________________________________________________________________________________________________________________
//                              ENABLE NOFITICATIONS
//_____________________________________________________________________________________________________________________________________________________________________________
// Callback for when we are receiving characteristics notifications
void on_notification_received(void* context, WarbleGattChar* characteristic, const uint8_t* value, uint8_t length) {
    (void) characteristic;
    
    auto _context = static_cast<enableNotifyContext*>(context);
    
    _context->handler(_context->caller, value, length);

    // std::cout << "Notification received from the feature with value: ";
    // for (int i = 0; i < length; i++) {
    //     std::cout << std::hex << (int)value[i] << " ";
    // }
    // std::cout << std::dec << std::endl;
}

// Callback for enable notifications
void on_enable_notifications_complete(void* context, WarbleGattChar* characteristic, const char* error) {
    (void) characteristic;
    
    auto _context = static_cast<enableNotifyContext*>(context);

    if (error != nullptr)
    {
	_context->task->set_value(0);
    }
    else
    {
	{
	    std::unique_lock<std::mutex> lock(enableNotifyMutex);

	    std::cout << "Enabling notification of characteristic: {" << warble_gattchar_get_uuid(_context->gattChar) << "}\n";
	    warble_gattchar_on_notification_received(_context->gattChar, _context, on_notification_received);
	}
	_context->task->set_value(1);
    }

    // if (error) {
    //     std::cerr << "Error enable notifications: " << error << std::endl;
    // } else {
    //     std::cout << "Enable notifications success." << std::endl;
    // }

    // // Configure callback function for receiving notifications
    // warble_gattchar_on_notification_received(gattChar, context, on_notification_received);
    // notify_handlers.insert({ caller, handler });
    // // call the 'ready' function pointer when the enable notification requeset has finished
    // ready(caller, MBL_MW_STATUS_OK);
}

static void  enable_notifications(void *context, const void *caller, const MblMwGattChar *characteristic, MblMwFnIntVoidPtrArray handler, MblMwFnVoidVoidPtrInt ready){
    // Convert UUID to string UUID
    std::string uuid_str = convert_uuid_to_string(characteristic->uuid_high, characteristic->uuid_low);

    WarbleGattChar* gattChar = warble_gatt_find_characteristic(gatt, uuid_str.c_str());
    
    // Verify if the connection exist
    if (gattChar == nullptr){
        std::cerr << "Error: Characteristic not found for UUID: " << uuid_str << std::endl;
        return;
    }

    std::promise<int> enableNotifyTask;
    {
	std::unique_lock<std::mutex> lock(enableNotifyMutex);

	s_enableNotifyContext = new enableNotifyContext;
	s_enableNotifyContext->task = &enableNotifyTask;
	s_enableNotifyContext->caller = caller;
	s_enableNotifyContext->handler = handler;
	s_enableNotifyContext->gattChar = gattChar;
    }

    // Habilite notifications the characteristics GATT asyncrone (ESTO NO PUEDE SER NULLPTR)
    // warble_gattchar_enable_notifications_async(gattChar, nullptr, on_enable_notifications_complete);
    warble_gattchar_enable_notifications_async(gattChar, s_enableNotifyContext, on_enable_notifications_complete);
    int success = s_enableNotifyContext->task->get_future().get();
    if(success == 1){
        std::cerr << "Succeded in enabling notifications.\n";
        ready(caller, MBL_MW_STATUS_OK);
    }
    else if(success == 0){
        std::cerr << "Failed to enable notifications.\n";
        ready(caller, MBL_MW_STATUS_ERROR_ENABLE_NOTIFY);
    }
    {
	std::unique_lock<std::mutex> lock(enableNotifyMutex);
	
	delete s_enableNotifyContext;
    }
    // if(MBL_MW_STATUS_OK == 0){
    //     std::cerr << "Succeded in enabling notifications.\n";
    //     ready(caller, MBL_MW_STATUS_OK);
    // }
    // else if(MBL_MW_STATUS_ERROR_ENABLE_NOTIFY == 64){
    //     std::cerr << "Failed to enable notifications.\n";
    //     ready(caller, MBL_MW_STATUS_ERROR_ENABLE_NOTIFY);
    // }
}
//_____________________________________________________________________________________________________________________________________________________________________________

//_____________________________________________________________________________________________________________________________________________________________________________
//                                 ON DISCONNECT
//_____________________________________________________________________________________________________________________________________________________________________________

static void on_disconnect(void *context, const void *caller, MblMwFnVoidVoidPtrInt handler){
// static void on_disconnect(void *context, const void *caller, int status){
    dc_handlers.insert({ caller, handler});
    // if(status == MBL_MW_STATUS_OK){
    //     std::cout << "Disconnect device success." << std::endl;
    // }
    // else{
    //     std::cout << "Unexpected disconnection. State Code: " << status << std::endl;
    // }
}

// static void on_disconnect_callback(void* context, WarbleGatt* gatt, int status){
//     if(status == 0){
//         std::cout << "Disconnect device success." << std::endl;
//     }
//     else{
//         std::cout << "Unexpected disconnection. State Code: " << status << std::endl;
//     }
// }
// static void on_disconnect(WarbleGatt* gatt){
//     warble_gatt_on_disconnect(gatt, nullptr, on_disconnect_callback);
// }
//_____________________________________________________________________________________________________________________________________________________________________________

static void disconnectWarbleGatt(){
    if (gatt == nullptr) {
        cerr << "No device to disconnect." << endl;
        return;
    }

    promise<int32_t> dc_task;
    cout << "Disconnecting..." << endl;
    warble_gatt_on_disconnect(gatt, &dc_task, [](void* context, WarbleGatt* caller, int32_t status) {
        ((promise<int32_t>*) context)->set_value(status);
    });
    warble_gatt_disconnect(gatt);
    cout << "disconnected, status = " << dc_task.get_future().get() << endl;
    
    cout << "Am I connected? " << warble_gatt_is_connected(gatt) << endl;

    // Libera la memoria de gatt después de la desconexión
    warble_gatt_delete(gatt);
    gatt = nullptr;
}

static void MetaMotionInitializeCallback(void* context, MblMwMetaWearBoard* board, int32_t status){
    (void) board;

    auto initBoardCalbackTask = static_cast<std::promise<void>*>(context);
    if (!status)
    {
	std::cerr << "Error initializing board: " << status << "\n";
    }
    else
    {
	std::cerr << "Board initialized\n";
    }
    initBoardCalbackTask->set_value();
}

static void MetaMotionInitialize(){
    // mbl_mw_metawearboard_set_time_for_response(board, 2500);
    std::promise<void> initBoardTask;
    mbl_mw_metawearboard_initialize(board, NULL, MetaMotionInitializeCallback);
    std::cout << "model = " << mbl_mw_metawearboard_get_model(board) << std::endl;
    std::cout << "model = " << mbl_mw_metawearboard_get_model_name(board) << std::endl;
    initBoardTask.get_future().get();

    auto check = mbl_mw_metawearboard_is_initialized(board);

    if(check == MBL_MW_STATUS_OK){
        printf("Initialized Ok\n");
    }
    else{
        printf("Error on initialized?\n");
    }
}

void enable_acc_sampling(MblMwMetaWearBoard* board) {
    // Set ODR to 25Hz or closest valid frequency
    mbl_mw_acc_set_odr(board, 25.f);

    // Set range to +/-4g or closest valid range
    mbl_mw_acc_set_range(board, 4.f);

    // Write the config to the sensor
    mbl_mw_acc_write_acceleration_config(board);

    auto data_handler = [](void* context, const MblMwData* data) -> void {
        // Cast value to MblMwCartesianFloat*
        auto acceleration = (MblMwCartesianFloat*) data->value;
        printf("(%.3fg, %.3fg, %.3fg)\n", acceleration->x, acceleration->y, acceleration->z);
    };

    auto acc_signal= mbl_mw_acc_get_acceleration_data_signal(board);
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

    MetaMotionMAC = argv[1];
    DongleBluetHCI = argc >= 3 ? argv[2] : nullptr;  // Ajuste de DongleBluetHCI
    NumberArgc = argc;

    connectionWarbleGatt();
    // MetaMotionBtleConnection = {gatt, write_gatt_char, read_gatt_char, enable_notifications, on_disconnect};
    MetaMotionBtleConnection = {write_gatt_char, read_gatt_char, enable_notifications, on_disconnect};
    board = mbl_mw_metawearboard_create(&MetaMotionBtleConnection);
    MetaMotionInitialize();
    printf("Hola Mundo\n");
    // sleep(2);
    // // enable_acc_sampling(board);
    // // disable_acc_sampling(board);
    // sleep(5);
    // disconnectWarbleGatt();

    return 0; // Retornar 0 al final de main
}
