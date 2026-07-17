/*
 * UAVCAN data structure definition for libcanard.
 *
 * Hand-written -- see ArrayCommand.h for provenance/verification notes.
 */
#include "uavcan/equipment/actuator/ArrayCommand.h"
#include "canard.h"

#if defined(__GNUC__)
# define CANARD_MAYBE_UNUSED(x) x __attribute__((unused))
#else
# define CANARD_MAYBE_UNUSED(x) x
#endif

/**
  * @brief uavcan_equipment_actuator_ArrayCommand_encode_internal
  * @param source : pointer to source data struct
  * @param msg_buf: pointer to msg storage
  * @param offset: bit offset to msg storage
  * @param root_item: for detecting if TAO should be used
  * @retval returns offset
  */
uint32_t uavcan_equipment_actuator_ArrayCommand_encode_internal(uavcan_equipment_actuator_ArrayCommand* source,
  void* msg_buf,
  uint32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(root_item))
{
    uint32_t c = 0;

    // Dynamic Array (commands)
    if (! root_item)
    {
        // - Add array length (ceil(log2(15+1)) = 4 bits)
        canardEncodeScalar(msg_buf, offset, 4, (void*)&source->commands.len);
        offset += 4;
    }

    // - Add array items (compound type)
    for (c = 0; c < source->commands.len; c++)
    {
        offset = uavcan_equipment_actuator_Command_encode_internal(&source->commands.data[c], msg_buf, offset, 0);
    }

    return offset;
}

/**
  * @brief uavcan_equipment_actuator_ArrayCommand_encode
  * @param source : Pointer to source data struct
  * @param msg_buf: Pointer to msg storage
  * @retval returns message length as bytes
  */
uint32_t uavcan_equipment_actuator_ArrayCommand_encode(uavcan_equipment_actuator_ArrayCommand* source, void* msg_buf)
{
    uint32_t offset = 0;

    offset = uavcan_equipment_actuator_ArrayCommand_encode_internal(source, msg_buf, offset, 1);

    return (offset + 7 ) / 8;
}

/**
  * @brief uavcan_equipment_actuator_ArrayCommand_decode_internal
  * @param transfer: Pointer to CanardRxTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     uavcan_equipment_actuator_ArrayCommand dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @param offset: Call with 0, bit offset to msg storage
  * @retval offset or ERROR value if < 0
  */
int32_t uavcan_equipment_actuator_ArrayCommand_decode_internal(
  const CanardRxTransfer* transfer,
  uint16_t CANARD_MAYBE_UNUSED(payload_len),
  uavcan_equipment_actuator_ArrayCommand* dest,
  uint8_t** CANARD_MAYBE_UNUSED(dyn_arr_buf),
  int32_t offset)
{
    int32_t ret = 0;
    uint32_t c = 0;

    // Dynamic Array (commands)
    //  - Last item in struct & Root item, tail array optimization
    if (payload_len)
    {
        //  - Calculate Array length from MSG length (32 bit fixed size per Command element)
        dest->commands.len = ((payload_len * 8) - offset) / 32;
    }
    else
    {
        // - Array length 4 bits (ceil(log2(15+1)))
        ret = canardDecodeScalar(transfer, (uint32_t)offset, 4, false, (void*)&dest->commands.len);
        if (ret != 4)
        {
            goto uavcan_equipment_actuator_ArrayCommand_error_exit;
        }
        offset += 4;
    }

    //  - Get Array
    if (dyn_arr_buf)
    {
        dest->commands.data = (uavcan_equipment_actuator_Command*)*dyn_arr_buf;
    }

    for (c = 0; c < dest->commands.len; c++)
    {
        if (dyn_arr_buf)
        {
            offset = uavcan_equipment_actuator_Command_decode_internal(transfer, payload_len,
                        &dest->commands.data[c], dyn_arr_buf, offset);
            if (offset < 0)
            {
                ret = offset;
                goto uavcan_equipment_actuator_ArrayCommand_error_exit;
            }
        }
        else
        {
            offset += 32;
        }
    }

    if (dyn_arr_buf)
    {
        *dyn_arr_buf = (uint8_t*)(((uavcan_equipment_actuator_Command*)*dyn_arr_buf) + dest->commands.len);
    }

    return offset;

uavcan_equipment_actuator_ArrayCommand_error_exit:
    if (ret < 0)
    {
        return ret;
    }
    else
    {
        return -CANARD_ERROR_INTERNAL;
    }
}

/**
  * @brief uavcan_equipment_actuator_ArrayCommand_decode
  * @param transfer: Pointer to CanardRxTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     uavcan_equipment_actuator_ArrayCommand dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @retval offset or ERROR value if < 0
  */
int32_t uavcan_equipment_actuator_ArrayCommand_decode(const CanardRxTransfer* transfer,
  uint16_t payload_len,
  uavcan_equipment_actuator_ArrayCommand* dest,
  uint8_t** dyn_arr_buf)
{
    const int32_t offset = 0;
    int32_t ret = 0;

    // Clear the destination struct
    for (uint32_t c = 0; c < sizeof(uavcan_equipment_actuator_ArrayCommand); c++)
    {
        ((uint8_t*)dest)[c] = 0x00;
    }

    ret = uavcan_equipment_actuator_ArrayCommand_decode_internal(transfer, payload_len, dest, dyn_arr_buf, offset);

    return ret;
}
