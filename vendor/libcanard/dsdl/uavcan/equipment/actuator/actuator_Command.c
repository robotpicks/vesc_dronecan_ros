/*
 * UAVCAN data structure definition for libcanard.
 *
 * Hand-written -- see Command.h for provenance/verification notes.
 */
#include "uavcan/equipment/actuator/Command.h"
#include "canard.h"

#if defined(__GNUC__)
# define CANARD_MAYBE_UNUSED(x) x __attribute__((unused))
#else
# define CANARD_MAYBE_UNUSED(x) x
#endif

/**
  * @brief uavcan_equipment_actuator_Command_encode_internal
  * @param source : pointer to source data struct
  * @param msg_buf: pointer to msg storage
  * @param offset: bit offset to msg storage
  * @param root_item: for detecting if TAO should be used (unused -- Command is never a root item)
  * @retval returns offset
  */
uint32_t uavcan_equipment_actuator_Command_encode_internal(uavcan_equipment_actuator_Command* source,
  void* msg_buf,
  uint32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(root_item))
{
#ifndef CANARD_USE_FLOAT16_CAST
    uint16_t tmp_float = 0;
#else
    CANARD_USE_FLOAT16_CAST tmp_float = 0;
#endif

    canardEncodeScalar(msg_buf, offset, 8, (void*)&source->actuator_id); // 255
    offset += 8;

    canardEncodeScalar(msg_buf, offset, 8, (void*)&source->command_type); // 255
    offset += 8;

    // float16 special handling
#ifndef CANARD_USE_FLOAT16_CAST
    tmp_float = canardConvertNativeFloatToFloat16(source->command_value);
#else
    tmp_float = (CANARD_USE_FLOAT16_CAST)source->command_value;
#endif
    canardEncodeScalar(msg_buf, offset, 16, (void*)&tmp_float); // 32767
    offset += 16;

    return offset;
}

/**
  * @brief uavcan_equipment_actuator_Command_decode_internal
  * @param transfer: Pointer to CanardRxTransfer transfer
  * @param payload_len: Payload message length (unused -- Command has no dynamic arrays)
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays (unused)
  * @param offset: Call with current bit offset to msg storage
  * @retval offset or ERROR value if < 0
  */
int32_t uavcan_equipment_actuator_Command_decode_internal(
  const CanardRxTransfer* transfer,
  uint16_t CANARD_MAYBE_UNUSED(payload_len),
  uavcan_equipment_actuator_Command* dest,
  uint8_t** CANARD_MAYBE_UNUSED(dyn_arr_buf),
  int32_t offset)
{
    int32_t ret = 0;
#ifndef CANARD_USE_FLOAT16_CAST
    uint16_t tmp_float = 0;
#else
    CANARD_USE_FLOAT16_CAST tmp_float = 0;
#endif

    ret = canardDecodeScalar(transfer, (uint32_t)offset, 8, false, (void*)&dest->actuator_id);
    if (ret != 8)
    {
        goto uavcan_equipment_actuator_Command_error_exit;
    }
    offset += 8;

    ret = canardDecodeScalar(transfer, (uint32_t)offset, 8, false, (void*)&dest->command_type);
    if (ret != 8)
    {
        goto uavcan_equipment_actuator_Command_error_exit;
    }
    offset += 8;

    // float16 special handling
    ret = canardDecodeScalar(transfer, (uint32_t)offset, 16, false, (void*)&tmp_float);
    if (ret != 16)
    {
        goto uavcan_equipment_actuator_Command_error_exit;
    }
#ifndef CANARD_USE_FLOAT16_CAST
    dest->command_value = canardConvertFloat16ToNativeFloat(tmp_float);
#else
    dest->command_value = (float)tmp_float;
#endif
    offset += 16;

    return offset;

uavcan_equipment_actuator_Command_error_exit:
    if (ret < 0)
    {
        return ret;
    }
    else
    {
        return -CANARD_ERROR_INTERNAL;
    }
}
