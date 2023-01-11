@{from dronecan_dsdlc_helpers import *}@
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>
@{
dep_headers = set()
for field in msg_fields:
    if field.type.category == field.type.CATEGORY_COMPOUND:
        dep_headers.add(msg_header_name(field.type))
    if field.type.category == field.type.CATEGORY_ARRAY and field.type.value_type.category == field.type.value_type.CATEGORY_COMPOUND:
        dep_headers.add(msg_header_name(field.type.value_type))
}@
@[  for header in dep_headers]@
#include <@(header)>
@[  end for]@

#ifdef __cplusplus
extern "C"
{
#endif

#define @(msg_define_name.upper())_MAX_SIZE @(int((msg_max_bitlen+7)/8))
#define @(msg_define_name.upper())_SIGNATURE @('(0x%08XULL)' % (msg_dt_sig,))
@[  if msg_default_dtid is not None]@
#define @(msg_define_name.upper())_ID @(msg_default_dtid)
@[  end if]@
@[  if msg_constants]
@[    for constant in msg_constants]@
#define @(msg_define_name.upper())_@(constant.name.upper()) @(constant.value)
@[    end for]@
@[  end if]@
@[  if msg_union]
enum @(msg_underscored_name)_type_t {
@[    for field in msg_fields]@
    @(msg_underscored_name.upper())_@(field.name.upper()),
@[    end for]@
};
@[  end if]@

@(msg_c_type) {
@[  if msg_union]@
    enum @(msg_underscored_name)_type_t union_tag;
    union {
@[    for field in msg_fields]@
@[      if field.type.category != field.type.CATEGORY_VOID]@
        @(field_cdef(field));
@[      end if]@
@[    end for]@
    };
@[  else]@
@[    for field in msg_fields]@
@[      if field.type.category != field.type.CATEGORY_VOID]@
    @(field_cdef(field));
@[      end if]@
@[    end for]@
@[  end if]@
};

uint32_t @(msg_underscored_name)_encode(@(msg_c_type)* msg, uint8_t* buffer);
bool @(msg_underscored_name)_decode(const CanardRxTransfer* transfer, @(msg_c_type)* msg);

#if defined(CANARD_DSDLC_INTERNAL)
@{indent = 0}@{ind = '    '*indent}@
static inline void _@(msg_underscored_name)_encode(uint8_t* buffer, uint32_t* bit_ofs, @(msg_c_type)* msg, bool tao);
static inline void _@(msg_underscored_name)_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, @(msg_c_type)* msg, bool tao);
void _@(msg_underscored_name)_encode(uint8_t* buffer, uint32_t* bit_ofs, @(msg_c_type)* msg, bool tao) {
@{indent += 1}@{ind = '    '*indent}@
@(ind)(void)buffer;
@(ind)(void)bit_ofs;
@(ind)(void)msg;
@(ind)(void)tao;

@[  if msg_union]@
@(ind)@(union_msg_tag_uint_type_from_num_fields(len(msg_fields))) union_tag = msg->union_tag;
@(ind)canardEncodeScalar(buffer, *bit_ofs, @(union_msg_tag_bitlen_from_num_fields(len(msg_fields))), &union_tag);
@(ind)*bit_ofs += @(union_msg_tag_bitlen_from_num_fields(len(msg_fields)));

@(ind)switch(msg->union_tag) {
@{indent += 1}@{ind = '    '*indent}@
@[  end if]@
@[    for field in msg_fields]@
@[      if msg_union]@
@(ind)case @(msg_underscored_name.upper())_@(field.name.upper()): {
@{indent += 1}@{ind = '    '*indent}@
@[      end if]@
@[      if field.type.category == field.type.CATEGORY_COMPOUND]@
@(ind)_@(underscored_name(field.type))_encode(buffer, bit_ofs, &msg->@(field.name), @('tao' if field == msg_fields[-1] else 'false'));
@[      elif field.type.category == field.type.CATEGORY_PRIMITIVE]@
@[        if field.type.kind == field.type.KIND_FLOAT and field.type.bitlen == 16]@
@(ind){
@(ind)    uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->@(field.name));
@(ind)    canardEncodeScalar(buffer, *bit_ofs, @(field.type.bitlen), &float16_val);
@(ind)}
@[        else]@
@(ind)canardEncodeScalar(buffer, *bit_ofs, @(field.type.bitlen), &msg->@(field.name));
@[        end if]@
@(ind)*bit_ofs += @(field.type.bitlen);
@[      elif field.type.category == field.type.CATEGORY_ARRAY]@
@[        if field.type.mode == field.type.MODE_DYNAMIC]@
@[          if field == msg_fields[-1] and field.type.value_type.get_min_bitlen() >= 8]@
@(ind)if (!tao) {
@{indent += 1}@{ind = '    '*indent}@
@[          end if]@
@(ind)canardEncodeScalar(buffer, *bit_ofs, @(array_len_field_bitlen(field.type)), &msg->@(field.name).len);
@(ind)*bit_ofs += @(array_len_field_bitlen(field.type));
@[          if field == msg_fields[-1] and field.type.value_type.get_min_bitlen() >= 8]@
@{indent -= 1}@{ind = '    '*indent}@
@(ind)}
@[          end if]@
@(ind)for (size_t i=0; i < msg->@(field.name).len; i++) {
@[        else]@
@(ind)for (size_t i=0; i < @(field.type.max_size); i++) {
@[        end if]@
@{indent += 1}@{ind = '    '*indent}@
@[        if field.type.value_type.category == field.type.value_type.CATEGORY_PRIMITIVE]@
@[          if field.type.value_type.kind == field.type.value_type.KIND_FLOAT and field.type.value_type.bitlen == 16]@
@(ind){
@(ind)    uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->@(field_get_data(field))[i]);
@(ind)    canardEncodeScalar(buffer, *bit_ofs, @(field.type.value_type.bitlen), &float16_val);
@(ind)}
@[          else]@
@(ind)canardEncodeScalar(buffer, *bit_ofs, @(field.type.value_type.bitlen), &msg->@(field_get_data(field))[i]);
@[          end if]@
@(ind)*bit_ofs += @(field.type.value_type.bitlen);
@[        elif field.type.value_type.category == field.type.value_type.CATEGORY_COMPOUND]@
@(ind)_@(underscored_name(field.type.value_type))_encode(buffer, bit_ofs, &msg->@(field_get_data(field))[i], @[if field == msg_fields[-1] and field.type.value_type.get_min_bitlen() < 8]tao && i==msg->@(field.name).len@[else]false@[end if]@);
@[        end if]@
@{indent -= 1}@{ind = '    '*indent}@
@(ind)}
@[      elif field.type.category == field.type.CATEGORY_VOID]@
@(ind)*bit_ofs += @(field.type.bitlen);
@[      end if]@
@[      if msg_union]@
@(ind)break;
@{indent -= 1}@{ind = '    '*indent}@
@(ind)}
@[      end if]@
@[    end for]@
@[  if msg_union]@
@{indent -= 1}@{ind = '    '*indent}@
@(ind)}
@[  end if]@
@{indent -= 1}@{ind = '    '*indent}@
@(ind)}

void _@(msg_underscored_name)_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, @(msg_c_type)* msg, bool tao) {
@{indent += 1}@{ind = '    '*indent}@
@(ind)(void)transfer;
@(ind)(void)bit_ofs;
@(ind)(void)msg;
@(ind)(void)tao;

@[  if msg_union]@
@(ind)@(union_msg_tag_uint_type_from_num_fields(len(msg_fields))) union_tag;
@(ind)canardDecodeScalar(transfer, *bit_ofs, @(union_msg_tag_bitlen_from_num_fields(len(msg_fields))), false, &union_tag);
@(ind)msg->union_tag = union_tag;
@(ind)*bit_ofs += @(union_msg_tag_bitlen_from_num_fields(len(msg_fields)));

@(ind)switch(msg->union_tag) {
@{indent += 1}@{ind = '    '*indent}@
@[  end if]@
@[    for field in msg_fields]@
@[      if msg_union]@
@(ind)case @(msg_underscored_name.upper())_@(field.name.upper()): {
@{indent += 1}@{ind = '    '*indent}@
@[      end if]@
@[      if field.type.category == field.type.CATEGORY_COMPOUND]@
@(ind)_@(underscored_name(field.type))_decode(transfer, bit_ofs, &msg->@(field.name), @('tao' if field == msg_fields[-1] else 'false'));
@[      elif field.type.category == field.type.CATEGORY_PRIMITIVE]@
@[        if field.type.kind == field.type.KIND_FLOAT and field.type.bitlen == 16]@
@(ind){
@(ind)    uint16_t float16_val;
@(ind)    canardDecodeScalar(transfer, *bit_ofs, @(field.type.bitlen), @('true' if dronecan_type_is_signed(field.type) else 'false'), &float16_val);
@(ind)    msg->@(field.name) = canardConvertFloat16ToNativeFloat(float16_val);
@(ind)}
@[        else]@
@(ind)canardDecodeScalar(transfer, *bit_ofs, @(field.type.bitlen), @('true' if dronecan_type_is_signed(field.type) else 'false'), &msg->@(field.name));
@[        end if]@
@(ind)*bit_ofs += @(field.type.bitlen);
@[      elif field.type.category == field.type.CATEGORY_ARRAY]@
@[        if field.type.mode == field.type.MODE_DYNAMIC]@
@[          if field == msg_fields[-1] and field.type.value_type.get_min_bitlen() >= 8]@
@(ind)if (!tao) {
@{indent += 1}@{ind = '    '*indent}@
@[          end if]@
@(ind)canardDecodeScalar(transfer, *bit_ofs, @(array_len_field_bitlen(field.type)), false, &msg->@(field.name).len);
@(ind)*bit_ofs += @(array_len_field_bitlen(field.type));
@[          if field == msg_fields[-1] and field.type.value_type.get_min_bitlen() >= 8]@
@{indent -= 1}@{ind = '    '*indent}@
@[              if field.type.value_type.category == field.type.value_type.CATEGORY_PRIMITIVE]@
@(ind)} else {
@{indent += 1}@{ind = '    '*indent}@
@(ind)msg->@(field.name).len = ((transfer->payload_len*8)-*bit_ofs)/@(field.type.value_type.bitlen);
@{indent -= 1}@{ind = '    '*indent}@
@[              end if]@
@(ind)}

@[          end if]@
@[              if field.type.value_type.category == field.type.value_type.CATEGORY_COMPOUND]@

@(ind)if (tao) {
@{indent += 1}@{ind = '    '*indent}@
msg->@(field.name).len = 0;
@(ind)while (((transfer->payload_len*8)-*bit_ofs) > 0) {
@{indent += 1}@{ind = '    '*indent}@
@(ind)_@(underscored_name(field.type.value_type))_decode(transfer, bit_ofs, &msg->@(field_get_data(field))[msg->@(field.name).len], @[if field == msg_fields[-1] and field.type.value_type.get_min_bitlen() < 8]tao && i==msg->@(field.name).len@[else]false@[end if]@);
@(ind)msg->@(field.name).len++;
@{indent -= 1}@{ind = '    '*indent}@
@(ind)}
@{indent -= 1}@{ind = '    '*indent}@
@(ind)} else {
@{indent += 1}@{ind = '    '*indent}@
@[              end if]@
@(ind)for (size_t i=0; i < msg->@(field.name).len; i++) {
@[        else]@
@(ind)for (size_t i=0; i < @(field.type.max_size); i++) {
@[        end if]@
@{indent += 1}@{ind = '    '*indent}@
@[        if field.type.value_type.category == field.type.value_type.CATEGORY_PRIMITIVE]@
@[          if field.type.value_type.kind == field.type.value_type.KIND_FLOAT and field.type.value_type.bitlen == 16]@
@(ind){
@(ind)    uint16_t float16_val;
@(ind)    canardDecodeScalar(transfer, *bit_ofs, @(field.type.value_type.bitlen), @('true' if dronecan_type_is_signed(field.type.value_type) else 'false'), &float16_val);
@(ind)    msg->@(field_get_data(field))[i] = canardConvertFloat16ToNativeFloat(float16_val);
@(ind)}
@[          else]@
@(ind)canardDecodeScalar(transfer, *bit_ofs, @(field.type.value_type.bitlen), @('true' if dronecan_type_is_signed(field.type.value_type) else 'false'), &msg->@(field_get_data(field))[i]);
@[          end if]@
@(ind)*bit_ofs += @(field.type.value_type.bitlen);
@[        elif field.type.value_type.category == field.type.value_type.CATEGORY_COMPOUND]@
@(ind)_@(underscored_name(field.type.value_type))_decode(transfer, bit_ofs, &msg->@(field_get_data(field))[i], @[if field == msg_fields[-1] and field.type.value_type.get_min_bitlen() < 8]tao && i==msg->@(field.name).len@[else]false@[end if]@);
@[        end if]@
@{indent -= 1}@{ind = '    '*indent}@
@(ind)}
@[              if field.type.value_type.category == field.type.value_type.CATEGORY_COMPOUND]@
@{indent -= 1}@{ind = '    '*indent}@
@(ind)}
@[              end if]@
@[      elif field.type.category == field.type.CATEGORY_VOID]@
@(ind)*bit_ofs += @(field.type.bitlen);
@[      end if]@
@[      if msg_union]@
@(ind)break;
@{indent -= 1}@{ind = '    '*indent}@
@(ind)}
@[      end if]@

@[    end for]@
@[  if msg_union]@
@{indent -= 1}@{ind = '    '*indent}@
@(ind)}
@[  end if]@
@{indent -= 1}@{ind = '    '*indent}@
@(ind)}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
@(msg_c_type) sample_@(msg_underscored_name)_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"
#endif
