/**
 * @file Kinetix5100ParamAccess.cpp
 * @brief Implementation of Kinetix 5100 CIP Parameter Access functions.
 */

#include "Kinetix5100ParamAccess.h"
#include "CipMessageRouter.h"

namespace k5100 {

eip::Bytes buildGetParameterRequest(uint16_t param_id)
{
    eip::Bytes epath = eip::buildEPath(kCipClassParameter, param_id, true, kCipAttributeValue);
    return eip::buildMessageRouterRequest(static_cast<uint8_t>(eip::CipService::kGetAttributeSingle), epath);
}

eip::Bytes buildSetParameterRequest(uint16_t param_id, uint32_t value, uint8_t size_bytes)
{
    eip::Bytes epath = eip::buildEPath(kCipClassParameter, param_id, true, kCipAttributeValue);
    
    eip::Bytes request_data;
    eip::ByteWriter writer(request_data);
    
    if (size_bytes == 2)
    {
        writer.u16(static_cast<uint16_t>(value));
    }
    else if (size_bytes == 4)
    {
        writer.u32(value);
    }
    
    return eip::buildMessageRouterRequest(static_cast<uint8_t>(eip::CipService::kSetAttributeSingle), epath, request_data);
}

bool parseGetParameterResponse(const eip::Bytes& mr_response_data, uint32_t& out_value, uint8_t size_bytes)
{
    eip::MessageRouterResponse response;
    if (!eip::parseMessageRouterResponse(mr_response_data, response))
    {
        return false;
    }
    
    if (!response.isSuccess())
    {
        return false;
    }
    
    eip::ByteReader reader(response.data);
    if (size_bytes == 2)
    {
        uint16_t val16 = 0;
        if (!reader.u16(val16))
        {
            return false;
        }
        out_value = val16;
        return true;
    }
    else if (size_bytes == 4)
    {
        return reader.u32(out_value);
    }
    
    return false;
}

} // namespace k5100
