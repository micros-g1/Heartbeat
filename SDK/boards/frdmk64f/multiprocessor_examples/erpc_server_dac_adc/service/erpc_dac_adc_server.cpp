/*
 * Copyright (c) 2014-2016, Freescale Semiconductor, Inc.
 * Copyright 2016 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*
 * Generated by erpcgen 1.7.4 on Thu Apr 16 10:59:26 2020.
 *
 * AUTOGENERATED - DO NOT EDIT
 */


#include "erpc_dac_adc_server.h"
#include <new>
#include "erpc_port.h"
#include "erpc_manually_constructed.h"

#if 10704 != ERPC_VERSION_NUMBER
#error "The generated shim code version is different to the rest of eRPC code."
#endif

using namespace erpc;
using namespace std;

#if ERPC_NESTED_CALLS_DETECTION
extern bool nestingDetection;
#endif

static ManuallyConstructed<dac_adc_service> s_dac_adc_service;



// Constant variable definitions
#pragma weak StringMaxSize
extern const uint8_t StringMaxSize = 11;

//! @brief Function to write struct AdcConfig
static void write_AdcConfig_struct(erpc::Codec * codec, const AdcConfig * data);

//! @brief Function to write struct Vector
static void write_Vector_struct(erpc::Codec * codec, const Vector * data);


// Write struct AdcConfig function implementation
static void write_AdcConfig_struct(erpc::Codec * codec, const AdcConfig * data)
{
    codec->write(data->vref);

    codec->write(data->atomicSteps);
}

// Write struct Vector function implementation
static void write_Vector_struct(erpc::Codec * codec, const Vector * data)
{
    codec->write(data->A_x);

    codec->write(data->A_y);

    codec->write(data->A_z);

    codec->write(data->M_x);

    codec->write(data->M_y);

    codec->write(data->M_z);
}



// Call the correct server shim based on method unique ID.
erpc_status_t dac_adc_service::handleInvocation(uint32_t methodId, uint32_t sequence, Codec * codec, MessageBufferFactory *messageFactory)
{
    switch (methodId)
    {
        case kdac_adc_adc_get_config_id:
            return adc_get_config_shim(codec, messageFactory, sequence);

        case kdac_adc_convert_dac_adc_id:
            return convert_dac_adc_shim(codec, messageFactory, sequence);

        case kdac_adc_set_led_id:
            return set_led_shim(codec, messageFactory, sequence);

        case kdac_adc_read_senzor_mag_accel_id:
            return read_senzor_mag_accel_shim(codec, messageFactory, sequence);

        case kdac_adc_board_get_name_id:
            return board_get_name_shim(codec, messageFactory, sequence);

        default:
            return kErpcStatus_InvalidArgument;
    }
}

// Server shim for adc_get_config of dac_adc interface.
erpc_status_t dac_adc_service::adc_get_config_shim(Codec * codec, MessageBufferFactory *messageFactory, uint32_t sequence)
{
    erpc_status_t err = kErpcStatus_Success;

    AdcConfig *config = NULL;

    // startReadMessage() was already called before this shim was invoked.

    config = (AdcConfig *) erpc_malloc(sizeof(AdcConfig));
    if (config == NULL)
    {
        codec->updateStatus(kErpcStatus_MemoryError);
    }

    err = codec->getStatus();
    if (!err)
    {
        // Invoke the actual served function.
#if ERPC_NESTED_CALLS_DETECTION
        nestingDetection = true;
#endif
        adc_get_config(config);
#if ERPC_NESTED_CALLS_DETECTION
        nestingDetection = false;
#endif

        // preparing MessageBuffer for serializing data
        err = messageFactory->prepareServerBufferForSend(codec->getBuffer());
    }

    if (!err)
    {
        // preparing codec for serializing data
        codec->reset();

        // Build response message.
        codec->startWriteMessage(kReplyMessage, kdac_adc_service_id, kdac_adc_adc_get_config_id, sequence);

        write_AdcConfig_struct(codec, config);

        err = codec->getStatus();
    }

    if (config)
    {
        erpc_free(config);
    }

    return err;
}

// Server shim for convert_dac_adc of dac_adc interface.
erpc_status_t dac_adc_service::convert_dac_adc_shim(Codec * codec, MessageBufferFactory *messageFactory, uint32_t sequence)
{
    erpc_status_t err = kErpcStatus_Success;

    uint32_t numberToConvert;
    uint32_t result;

    // startReadMessage() was already called before this shim was invoked.

    codec->read(&numberToConvert);

    err = codec->getStatus();
    if (!err)
    {
        // Invoke the actual served function.
#if ERPC_NESTED_CALLS_DETECTION
        nestingDetection = true;
#endif
        convert_dac_adc(numberToConvert, &result);
#if ERPC_NESTED_CALLS_DETECTION
        nestingDetection = false;
#endif

        // preparing MessageBuffer for serializing data
        err = messageFactory->prepareServerBufferForSend(codec->getBuffer());
    }

    if (!err)
    {
        // preparing codec for serializing data
        codec->reset();

        // Build response message.
        codec->startWriteMessage(kReplyMessage, kdac_adc_service_id, kdac_adc_convert_dac_adc_id, sequence);

        codec->write(result);

        err = codec->getStatus();
    }

    return err;
}

// Server shim for set_led of dac_adc interface.
erpc_status_t dac_adc_service::set_led_shim(Codec * codec, MessageBufferFactory *messageFactory, uint32_t sequence)
{
    erpc_status_t err = kErpcStatus_Success;

    uint8_t whichLed;

    // startReadMessage() was already called before this shim was invoked.

    codec->read(&whichLed);

    err = codec->getStatus();
    if (!err)
    {
        // Invoke the actual served function.
#if ERPC_NESTED_CALLS_DETECTION
        nestingDetection = true;
#endif
        set_led(whichLed);
#if ERPC_NESTED_CALLS_DETECTION
        nestingDetection = false;
#endif

        // preparing MessageBuffer for serializing data
        err = messageFactory->prepareServerBufferForSend(codec->getBuffer());
    }

    if (!err)
    {
        // preparing codec for serializing data
        codec->reset();

        // Build response message.
        codec->startWriteMessage(kReplyMessage, kdac_adc_service_id, kdac_adc_set_led_id, sequence);

        err = codec->getStatus();
    }

    return err;
}

// Server shim for read_senzor_mag_accel of dac_adc interface.
erpc_status_t dac_adc_service::read_senzor_mag_accel_shim(Codec * codec, MessageBufferFactory *messageFactory, uint32_t sequence)
{
    erpc_status_t err = kErpcStatus_Success;

    Vector *results = NULL;

    // startReadMessage() was already called before this shim was invoked.

    results = (Vector *) erpc_malloc(sizeof(Vector));
    if (results == NULL)
    {
        codec->updateStatus(kErpcStatus_MemoryError);
    }

    err = codec->getStatus();
    if (!err)
    {
        // Invoke the actual served function.
#if ERPC_NESTED_CALLS_DETECTION
        nestingDetection = true;
#endif
        read_senzor_mag_accel(results);
#if ERPC_NESTED_CALLS_DETECTION
        nestingDetection = false;
#endif

        // preparing MessageBuffer for serializing data
        err = messageFactory->prepareServerBufferForSend(codec->getBuffer());
    }

    if (!err)
    {
        // preparing codec for serializing data
        codec->reset();

        // Build response message.
        codec->startWriteMessage(kReplyMessage, kdac_adc_service_id, kdac_adc_read_senzor_mag_accel_id, sequence);

        write_Vector_struct(codec, results);

        err = codec->getStatus();
    }

    if (results)
    {
        erpc_free(results);
    }

    return err;
}

// Server shim for board_get_name of dac_adc interface.
erpc_status_t dac_adc_service::board_get_name_shim(Codec * codec, MessageBufferFactory *messageFactory, uint32_t sequence)
{
    erpc_status_t err = kErpcStatus_Success;

    char * name = NULL;

    // startReadMessage() was already called before this shim was invoked.

    name = (char *) erpc_malloc((StringMaxSize + 1) * sizeof(char));
    if (name == NULL)
    {
        codec->updateStatus(kErpcStatus_MemoryError);
    }
    else
    {
        name[StringMaxSize] = '\0';
    }

    err = codec->getStatus();
    if (!err)
    {
        // Invoke the actual served function.
#if ERPC_NESTED_CALLS_DETECTION
        nestingDetection = true;
#endif
        board_get_name(name);
#if ERPC_NESTED_CALLS_DETECTION
        nestingDetection = false;
#endif

        // preparing MessageBuffer for serializing data
        err = messageFactory->prepareServerBufferForSend(codec->getBuffer());
    }

    if (!err)
    {
        // preparing codec for serializing data
        codec->reset();

        // Build response message.
        codec->startWriteMessage(kReplyMessage, kdac_adc_service_id, kdac_adc_board_get_name_id, sequence);

        codec->writeString(strlen(name), name);

        err = codec->getStatus();
    }

    if (name)
    {
        erpc_free(name);
    }

    return err;
}

erpc_service_t create_dac_adc_service()
{
    s_dac_adc_service.construct();
    return s_dac_adc_service.get();
}

void destroy_dac_adc_service()
{
    s_dac_adc_service.destroy();
}
