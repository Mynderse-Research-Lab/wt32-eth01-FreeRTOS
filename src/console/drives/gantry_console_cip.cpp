/**
 * @file gantry_console_cip.cpp
 * @brief Gantry test console: raw CIP attribute access to the HCS01 and Kinetix 5100.
 *
 * One of the gantry_console_*.cpp files that make up the test console;
 * gantry_console_internal.h declares what they share.
 */

#include "gantry_console_internal.h"

namespace GantryConsoleImpl {

bool parseHcsIdnToInstance(const char *idn, uint16_t &out_inst)
{
    if (idn == nullptr)
    {
        return false;
    }
    char kind = '\0';
    unsigned set_num = 0;
    unsigned blk_num = 0;
    if (sscanf(idn, "%c-%u-%u", &kind, &set_num, &blk_num) != 3)
    {
        return false;
    }
    kind = static_cast<char>(toupper(static_cast<unsigned char>(kind)));
    if ((kind != 'S' && kind != 'P') || set_num > 7U || blk_num > 4095U)
    {
        return false;
    }
    out_inst = static_cast<uint16_t>(((kind == 'P') ? 0x8000U : 0U) |
                                     ((set_num & 0x07U) << 12) |
                                     (blk_num & 0x0FFFU));
    return true;
}

bool readCipAttrRaw(eip::EipSession &session, uint16_t class_id, uint16_t inst_id,
                    uint16_t attr_id, eip::Bytes &out_data, uint8_t *out_status)
{
    const eip::Bytes epath = eip::buildEPath(class_id, inst_id, true, attr_id);
    const eip::Bytes req = eip::buildMessageRouterRequest(
        static_cast<uint8_t>(eip::CipService::kGetAttributeSingle), epath);
    eip::Bytes reply;
    if (!session.sendExplicit(req, reply))
    {
        if (out_status != nullptr)
        {
            *out_status = 0xFF;
        }
        return false;
    }
    eip::MessageRouterResponse mr;
    if (!eip::parseMessageRouterResponse(reply, mr))
    {
        if (out_status != nullptr)
        {
            *out_status = 0xFE;
        }
        return false;
    }
    if (out_status != nullptr)
    {
        *out_status = mr.general_status;
    }
    if (!mr.isSuccess())
    {
        return false;
    }
    out_data = mr.data;
    return true;
}

bool writeCipAttrRaw(eip::EipSession &session, uint16_t class_id, uint16_t inst_id,
                     uint16_t attr_id, const eip::Bytes &data, uint8_t *out_status)
{
    const eip::Bytes epath = eip::buildEPath(class_id, inst_id, true, attr_id);
    const eip::Bytes req = eip::buildMessageRouterRequest(
        static_cast<uint8_t>(eip::CipService::kSetAttributeSingle), epath, data);
    eip::Bytes reply;
    if (!session.sendExplicit(req, reply))
    {
        if (out_status != nullptr)
        {
            *out_status = 0xFF;
        }
        return false;
    }
    eip::MessageRouterResponse mr;
    if (!eip::parseMessageRouterResponse(reply, mr))
    {
        if (out_status != nullptr)
        {
            *out_status = 0xFE;
        }
        return false;
    }
    if (out_status != nullptr)
    {
        *out_status = mr.general_status;
    }
    return mr.isSuccess();
}

static void formatHcsValue(const eip::Bytes &data, uint32_t attr3, char *out, size_t max_len)
{
    const uint8_t dec = static_cast<uint8_t>((attr3 >> 24) & 0x0FU);
    const uint8_t fmt = static_cast<uint8_t>((attr3 >> 20) & 0x07U);
    if (fmt == 4 && data.size() >= 2)
    {
        // SERCOS string: 2-byte current len + 2-byte max len + ASCII bytes (or raw ASCII)
        size_t off = 0;
        size_t slen = data.size();
        if (data.size() >= 4)
        {
            const uint16_t clen = static_cast<uint16_t>(data[0] | (data[1] << 8));
            if (clen <= data.size() - 4)
            {
                off = 4;
                slen = clen;
            }
        }
        const size_t n = (slen < max_len - 1) ? slen : (max_len - 1);
        memcpy(out, data.data() + off, n);
        out[n] = '\0';
        return;
    }
    double scale = 1.0;
    for (uint8_t i = 0; i < dec && i < 9; ++i)
    {
        scale *= 10.0;
    }
    if (data.size() == 2)
    {
        const uint16_t u16 = static_cast<uint16_t>(data[0] | (data[1] << 8));
        if (fmt == 0 || fmt == 3)
        {
            snprintf(out, max_len, "0x%04X", static_cast<unsigned>(u16));
        }
        else if (fmt == 2)
        {
            const int16_t s16 = static_cast<int16_t>(u16);
            snprintf(out, max_len, "%.*f (raw=%d)", static_cast<int>(dec),
                     static_cast<double>(s16) / scale, static_cast<int>(s16));
        }
        else
        {
            snprintf(out, max_len, "%.*f (raw=%u)", static_cast<int>(dec),
                     static_cast<double>(u16) / scale, static_cast<unsigned>(u16));
        }
        return;
    }
    if (data.size() == 4)
    {
        const uint32_t u32 = static_cast<uint32_t>(data[0]) |
                             (static_cast<uint32_t>(data[1]) << 8) |
                             (static_cast<uint32_t>(data[2]) << 16) |
                             (static_cast<uint32_t>(data[3]) << 24);
        if (fmt == 0 || fmt == 3)
        {
            snprintf(out, max_len, "0x%08lX", static_cast<unsigned long>(u32));
        }
        else if (fmt == 2)
        {
            const int32_t s32 = static_cast<int32_t>(u32);
            snprintf(out, max_len, "%.*f (raw=%ld)", static_cast<int>(dec),
                     static_cast<double>(s32) / scale, static_cast<long>(s32));
        }
        else
        {
            snprintf(out, max_len, "%.*f (raw=%lu)", static_cast<int>(dec),
                     static_cast<double>(u32) / scale, static_cast<unsigned long>(u32));
        }
        return;
    }
    size_t pos = 0;
    for (size_t i = 0; i < data.size() && pos + 3 < max_len; ++i)
    {
        pos += static_cast<size_t>(snprintf(out + pos, max_len - pos, "%02X ", data[i]));
    }
    if (pos == 0 && max_len > 0)
    {
        out[0] = '\0';
    }
}

bool readHcsIdnCip(eip::EipSession &session, const char *idn, char *out_str, size_t max_len,
                   uint32_t *out_attr3, size_t *out_len)
{
    uint16_t inst = 0;
    if (!parseHcsIdnToInstance(idn, inst))
    {
        snprintf(out_str, max_len, "<bad IDN>");
        return false;
    }
    eip::Bytes attr3_bytes;
    uint32_t attr3 = 0;
    uint8_t st3 = 0;
    if (readCipAttrRaw(session, kHcs01CipParamClass, inst, 3, attr3_bytes, &st3) &&
        attr3_bytes.size() == 4)
    {
        attr3 = static_cast<uint32_t>(attr3_bytes[0]) |
                (static_cast<uint32_t>(attr3_bytes[1]) << 8) |
                (static_cast<uint32_t>(attr3_bytes[2]) << 16) |
                (static_cast<uint32_t>(attr3_bytes[3]) << 24);
    }
    vTaskDelay(pdMS_TO_TICKS(4));
    eip::Bytes val_bytes;
    uint8_t st7 = 0;
    if (!readCipAttrRaw(session, kHcs01CipParamClass, inst, 7, val_bytes, &st7))
    {
        snprintf(out_str, max_len, "<CIP err a3=0x%02X a7=0x%02X>",
                 static_cast<unsigned>(st3), static_cast<unsigned>(st7));
        return false;
    }
    if (out_attr3 != nullptr)
    {
        *out_attr3 = attr3;
    }
    if (out_len != nullptr)
    {
        *out_len = val_bytes.size();
    }
    formatHcsValue(val_bytes, attr3, out_str, max_len);
    return true;
}

bool readK5100Attr(eip::EipSession &session, uint16_t class_id, uint16_t inst_id,
                   uint16_t attr_id, uint8_t size_bytes, uint32_t &out_val)
{
    const eip::Bytes epath = eip::buildEPath(class_id, inst_id, true, attr_id);
    const eip::Bytes req = eip::buildMessageRouterRequest(
        static_cast<uint8_t>(eip::CipService::kGetAttributeSingle), epath);
    eip::Bytes reply;
    if (!session.sendExplicit(req, reply))
    {
        return false;
    }
    return k5100::parseGetParameterResponse(reply, out_val, size_bytes);
}

}  // namespace GantryConsoleImpl
