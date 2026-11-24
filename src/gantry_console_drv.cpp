/**
 * @file gantry_console_drv.cpp
 * @brief Gantry test console: the drv command (drive parameter dumps, reads and writes).
 *
 * One of the gantry_console_*.cpp files that make up the test console;
 * gantry_console_internal.h declares what they share.
 */

#include "gantry_console_internal.h"

namespace GantryConsoleImpl {

static void dumpHcs01Params(w5500::W5500Hal &hal)
{
    ESP_LOGI(TAG, "=== HCS01 Theta (%s:44818 CIP Class 0x64) ===", CONFIG_EIP_TARGET_IP_THETA);
    eip::EipSocketW5500Tcp tcp(hal);
    if (!tcp.connect(CONFIG_EIP_TARGET_IP_THETA, eip::EipSession::kDefaultPort))
    {
        ESP_LOGE(TAG, "drv: HCS01 CIP connect failed (%s:44818)", CONFIG_EIP_TARGET_IP_THETA);
        return;
    }
    eip::EipSession session(tcp);
    if (!session.registerSession())
    {
        ESP_LOGE(TAG, "drv: HCS01 RegisterSession failed");
        tcp.close();
        return;
    }
    struct IdnEntry
    {
        const char *idn;
        const char *label;
    };
    static const IdnEntry kIdns[] = {
        {"S-0-0095", "diag"},
        {"S-0-0390", "diag_num"},
        {"S-0-0383", "motor_temp_C"},
        {"S-0-0384", "amp_temp_C"},
        {"S-0-0201", "warn_temp_C"},
        {"S-0-0204", "shut_temp_C"},
        {"P-0-0141", "therm_load_pct"},
        {"S-0-0111", "motor_I0_A"},
        {"S-0-0109", "motor_Imax_A"},
        {"S-0-0110", "amp_Imax_A"},
        {"P-0-4046", "eff_Ipeak_A"},
        {"S-0-0082", "tq_lim_pos_pct"},
        {"S-0-0083", "tq_lim_neg_pct"},
        {"S-0-0092", "tq_lim_bip_pct"},
        {"P-0-0109", "tq_peak_lim_pct"},
        {"P-0-0442", "eff_tq_pos"},
        {"P-0-0443", "eff_tq_neg"},
        {"P-0-0444", "eff_tq_peak"},
        {"P-0-0445", "tq_lim_status"},
        {"P-0-0051", "Kt_Nm_per_A"},
        {"P-0-4048", "R_winding_ohm"},
        {"P-0-4010", "J_load_kgm2"},
        {"S-0-0100", "vel_Kp"},
        {"S-0-0101", "vel_Tn_ms"},
        {"S-0-0104", "pos_Kv"},
        {"S-0-0106", "curr_Kp"},
        {"S-0-0107", "curr_Tn_ms"},
        {"S-0-0348", "accel_FF_Ka"},
        {"P-0-0004", "vel_smooth_us"},
        {"S-0-0057", "pos_win_deg"},
        {"S-0-0091", "vel_lim_bip"},
        {"S-0-0138", "acc_lim_bip"},
        {"P-0-0556", "axis_ctrl_cfg"},
    };
    char val[80];
    for (const auto &e : kIdns)
    {
        val[0] = '\0';
        readHcsIdnCip(session, e.idn, val, sizeof(val));
        ESP_LOGI(TAG, "  %-10s %-16s = %s", e.idn, e.label, val);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    session.unregisterSession();
    tcp.close();
}

static void dumpK5100Params(w5500::W5500Hal &hal, char axis_ch, const char *ip)
{
    ESP_LOGI(TAG, "=== Kinetix 5100 %c (%s:44818 CIP) ===", axis_ch, ip);
    eip::EipSocketW5500Tcp tcp(hal);
    if (!tcp.connect(ip, eip::EipSession::kDefaultPort))
    {
        ESP_LOGE(TAG, "drv: K5100 %c connect failed (%s)", axis_ch, ip);
        return;
    }
    eip::EipSession session(tcp);
    if (!session.registerSession())
    {
        ESP_LOGE(TAG, "drv: K5100 %c RegisterSession failed", axis_ch);
        tcp.close();
        return;
    }

    uint32_t v = 0;
    if (readK5100Attr(session, 0x28, 1, 6, 2, v))
    {
        ESP_LOGI(TAG, "  P004 MotorContCurrent   = %.2f A", static_cast<double>(v) * 0.01);
    }
    if (readK5100Attr(session, 0x28, 1, 0x0D, 4, v))
    {
        ESP_LOGI(TAG, "  P010 MotorKt            = %.3f Nm/A", static_cast<double>(v) * 0.001);
    }
    if (readK5100Attr(session, 0x28, 1, 0x0E, 4, v))
    {
        ESP_LOGI(TAG, "  P011 MotorInertia       = %lu (1e-7 kgm2)", static_cast<unsigned long>(v));
    }

    struct K5100Entry
    {
        uint16_t id;
        uint8_t sz;
        bool is_signed;
        const char *label;
    };
    static const K5100Entry kParams[] = {
        {611, 2, false, "MotorPeakCurrent_0.01A"},
        {647, 2, false, "MotorRatedTq_0.01Nm"},
        {649, 2, true,  "MaxContTqLim_0.01pct"},
        {128, 2, true,  "PresetTqLim1_pct"},
        {129, 2, true,  "PresetTqLim2_pct"},
        {130, 2, true,  "PresetTqLim3_pct"},
        {629, 2, true,  "MotorOverloadLim_pct"},
        {161, 2, true,  "OverloadWarn_pct"},
        {162, 2, true,  "CrashTqThresh_pct"},
        {655, 4, false, "OutputCurrent_0.01A"},
        {217, 2, false, "GainAdjustMode"},
        {216, 2, true,  "SysGainRespLevel"},
        {144, 2, false, "LoadInertiaRatio"},
        {659, 4, true,  "TotalInertia_1e-7"},
        {185, 2, true,  "PosPropGain_rad_s"},
        {187, 2, true,  "VelFFGain_pct"},
        {189, 2, true,  "VelPropGain_rad_s"},
        {191, 2, true,  "VelIntGain_rad_s"},
        {192, 2, true,  "AccelFFGain_pct"},
        {123, 2, true,  "TqCmdLpf_ms"},
        {208, 2, true,  "Notch1Freq_Hz"},
        {209, 2, true,  "Notch1Depth_dB"},
        {210, 2, true,  "ResSuppLpf_0.1ms"},
        {605, 4, true,  "CurrLoopKp_1e-3"},
        {606, 2, true,  "CurrLoopKi"},
    };
    for (const auto &p : kParams)
    {
        v = 0;
        if (readK5100Attr(session, k5100::kCipClassParameter, p.id, k5100::kCipAttributeValue,
                          p.sz, v))
        {
            if (p.is_signed && p.sz == 2)
            {
                ESP_LOGI(TAG, "  P%03u %-22s = %d", static_cast<unsigned>(p.id), p.label,
                         static_cast<int>(static_cast<int16_t>(v)));
            }
            else if (p.is_signed && p.sz == 4)
            {
                ESP_LOGI(TAG, "  P%03u %-22s = %ld", static_cast<unsigned>(p.id), p.label,
                         static_cast<long>(static_cast<int32_t>(v)));
            }
            else
            {
                ESP_LOGI(TAG, "  P%03u %-22s = %lu", static_cast<unsigned>(p.id), p.label,
                         static_cast<unsigned long>(v));
            }
        }
        else
        {
            ESP_LOGW(TAG, "  P%03u %-22s = <read err>", static_cast<unsigned>(p.id), p.label);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    session.unregisterSession();
    tcp.close();
}

void runDrvCommand(const GantryTestConsoleConfig *cfg, const char *cmd)
{
    if (cfg == nullptr || cfg->gantry == nullptr || cfg->w5500_hal == nullptr)
    {
        ESP_LOGE(TAG, "drv: gantry or W5500 HAL not available");
        return;
    }

    char sub[16] = {};
    char a1[32] = {};
    char a2[32] = {};
    char a3[32] = {};
    char a4[32] = {};
    const int n = sscanf(cmd, "%*s %15s %31s %31s %31s %31s", sub, a1, a2, a3, a4);
    if (n < 1)
    {
        ESP_LOGI(TAG, "Usage: drv read [t|x|z|all]");
        ESP_LOGI(TAG, "       drv hcs get <IDN> | drv hcs set <IDN> <val> | drv hcs cmd <IDN> [ms]");
        ESP_LOGI(TAG, "       drv k5100 get <x|z> <id> [2|4] | drv k5100 set <x|z> <id> <val> [2|4]");
        return;
    }
    for (int i = 0; sub[i]; ++i) sub[i] = static_cast<char>(tolower(static_cast<unsigned char>(sub[i])));

    if (strcmp(sub, "read") == 0)
    {
        const char which = (n >= 2) ? static_cast<char>(tolower(static_cast<unsigned char>(a1[0]))) : 'a';
        if (which == 'a' || which == 't')
        {
            dumpHcs01Params(*(cfg->w5500_hal));
        }
        if (which == 'a' || which == 'x')
        {
            dumpK5100Params(*(cfg->w5500_hal), 'X', CONFIG_EIP_TARGET_IP_X);
        }
        if (which == 'a' || which == 'z')
        {
            dumpK5100Params(*(cfg->w5500_hal), 'Z', CONFIG_EIP_TARGET_IP_Z);
        }
        return;
    }

    if (strcmp(sub, "hcs") == 0 && n >= 3)
    {
        for (int i = 0; a1[i]; ++i) a1[i] = static_cast<char>(tolower(static_cast<unsigned char>(a1[i])));
        uint16_t inst = 0;
        if (!parseHcsIdnToInstance(a2, inst))
        {
            ESP_LOGE(TAG, "drv hcs: invalid IDN '%s' (expected S-0-xxxx or P-0-xxxx)", a2);
            return;
        }
        eip::EipSocketW5500Tcp tcp(*(cfg->w5500_hal));
        if (!tcp.connect(CONFIG_EIP_TARGET_IP_THETA, eip::EipSession::kDefaultPort))
        {
            ESP_LOGE(TAG, "drv hcs: CIP connect failed (%s:44818)", CONFIG_EIP_TARGET_IP_THETA);
            return;
        }
        eip::EipSession session(tcp);
        if (!session.registerSession())
        {
            ESP_LOGE(TAG, "drv hcs: RegisterSession failed");
            tcp.close();
            return;
        }
        if (strcmp(a1, "get") == 0)
        {
            char val[96] = {};
            uint32_t attr3 = 0;
            size_t vlen = 0;
            if (readHcsIdnCip(session, a2, val, sizeof(val), &attr3, &vlen))
            {
                ESP_LOGI(TAG, "HCS %s (inst=0x%04X attr3=0x%08lX %uB) = %s",
                         a2, static_cast<unsigned>(inst), static_cast<unsigned long>(attr3),
                         static_cast<unsigned>(vlen), val);
            }
            else
            {
                ESP_LOGE(TAG, "HCS get %s failed: %s", a2, val);
            }
        }
        else if (strcmp(a1, "set") == 0 && n >= 4)
        {
            if (cfg->gantry->isBusy())
            {
                ESP_LOGE(TAG, "drv hcs set: gantry busy — pause/stop first");
                session.unregisterSession();
                tcp.close();
                return;
            }
            if (hcs01::Hcs01ComwsClient::isProtectedIdn(a2))
            {
                ESP_LOGE(TAG, "drv hcs set: refusing protected IP IDN %s", a2);
                session.unregisterSession();
                tcp.close();
                return;
            }
            char before[80] = {};
            uint32_t attr3 = 0;
            size_t vlen = 0;
            if (!readHcsIdnCip(session, a2, before, sizeof(before), &attr3, &vlen) ||
                (vlen != 2 && vlen != 4))
            {
                ESP_LOGE(TAG, "drv hcs set %s: pre-read failed (%s, len=%u)",
                         a2, before, static_cast<unsigned>(vlen));
                session.unregisterSession();
                tcp.close();
                return;
            }
            const uint8_t dec = static_cast<uint8_t>((attr3 >> 24) & 0x0FU);
            int64_t raw_i64 = 0;
            if (strncmp(a3, "0x", 2) == 0 || strncmp(a3, "0X", 2) == 0)
            {
                raw_i64 = static_cast<int64_t>(strtoull(a3, nullptr, 16));
            }
            else
            {
                double scale = 1.0;
                for (uint8_t i = 0; i < dec && i < 9; ++i) scale *= 10.0;
                const double dv = strtod(a3, nullptr) * scale;
                raw_i64 = static_cast<int64_t>(dv >= 0.0 ? (dv + 0.5) : (dv - 0.5));
            }
            eip::Bytes payload;
            eip::ByteWriter w(payload);
            if (vlen == 2)
            {
                w.u16(static_cast<uint16_t>(raw_i64));
            }
            else
            {
                w.u32(static_cast<uint32_t>(raw_i64));
            }
            uint8_t cip_st = 0;
            const bool ok = writeCipAttrRaw(session, kHcs01CipParamClass, inst, 7, payload, &cip_st);
            vTaskDelay(pdMS_TO_TICKS(10));
            char after[80] = {};
            readHcsIdnCip(session, a2, after, sizeof(after));
            ESP_LOGI(TAG, "HCS set %s=%s (raw=%lld %uB) -> %s (cip=0x%02X, before='%s', after='%s')",
                     a2, a3, static_cast<long long>(raw_i64), static_cast<unsigned>(vlen),
                     ok ? "OK" : "FAIL", static_cast<unsigned>(cip_st), before, after);
        }
        else if (strcmp(a1, "cmd") == 0)
        {
            if (cfg->gantry->isBusy())
            {
                ESP_LOGE(TAG, "drv hcs cmd: gantry busy — pause/stop first");
                session.unregisterSession();
                tcp.close();
                return;
            }
            const uint32_t timeout_ms = (n >= 4) ? static_cast<uint32_t>(strtoul(a3, nullptr, 10)) : 20000U;
            eip::Bytes z2 = {0x00, 0x00};
            eip::Bytes go2 = {0x03, 0x00};
            uint8_t st_wr = 0;
            writeCipAttrRaw(session, kHcs01CipParamClass, inst, 7, z2, &st_wr);
            vTaskDelay(pdMS_TO_TICKS(20));
            bool ok = writeCipAttrRaw(session, kHcs01CipParamClass, inst, 7, go2, &st_wr);
            bool done = false;
            bool err = false;
            uint16_t last_sw = 0;
            for (uint32_t elapsed = 0; ok && elapsed < timeout_ms; elapsed += 200U)
            {
                vTaskDelay(pdMS_TO_TICKS(200));
                eip::Bytes sb;
                if (!readCipAttrRaw(session, kHcs01CipParamClass, inst, 8, sb) || sb.size() < 2)
                {
                    if (!readCipAttrRaw(session, kHcs01CipParamClass, inst, 7, sb) || sb.size() < 2)
                    {
                        continue;
                    }
                }
                last_sw = static_cast<uint16_t>(sb[0] | (sb[1] << 8));
                if (last_sw & 0x08U)
                {
                    err = true;
                    break;
                }
                if ((last_sw & 0x03U) == 0x03U && !(last_sw & 0x04U))
                {
                    done = true;
                    break;
                }
            }
            writeCipAttrRaw(session, kHcs01CipParamClass, inst, 7, z2);
            vTaskDelay(pdMS_TO_TICKS(20));
            char diag[96] = {};
            readHcsIdnCip(session, "S-0-0095", diag, sizeof(diag));
            ESP_LOGI(TAG, "HCS cmd %s -> %s (wr=0x%02X sw=0x%04X err=%d diag='%s')",
                     a2, (done && !err) ? "OK" : "FAIL",
                     static_cast<unsigned>(st_wr), static_cast<unsigned>(last_sw),
                     err ? 1 : 0, diag);
        }
        else
        {
            ESP_LOGE(TAG, "Usage: drv hcs <get|set|cmd> <IDN> [val|timeout_ms]");
        }
        session.unregisterSession();
        tcp.close();
        return;
    }

    if (strcmp(sub, "k5100") == 0 && n >= 4)
    {
        for (int i = 0; a1[i]; ++i) a1[i] = static_cast<char>(tolower(static_cast<unsigned char>(a1[i])));
        const char ax = static_cast<char>(tolower(static_cast<unsigned char>(a2[0])));
        const char *ip = (ax == 'x') ? CONFIG_EIP_TARGET_IP_X
                       : (ax == 'z') ? CONFIG_EIP_TARGET_IP_Z
                                     : nullptr;
        if (ip == nullptr)
        {
            ESP_LOGE(TAG, "drv k5100: axis must be x or z");
            return;
        }
        const uint16_t pid = static_cast<uint16_t>(strtoul(a3, nullptr, 10));
        eip::EipSocketW5500Tcp tcp(*(cfg->w5500_hal));
        if (!tcp.connect(ip, eip::EipSession::kDefaultPort))
        {
            ESP_LOGE(TAG, "drv k5100: connect failed (%s)", ip);
            return;
        }
        eip::EipSession session(tcp);
        if (!session.registerSession())
        {
            ESP_LOGE(TAG, "drv k5100: RegisterSession failed");
            tcp.close();
            return;
        }
        if (strcmp(a1, "get") == 0)
        {
            const uint8_t sz = (n >= 5 && atoi(a4) == 4) ? 4 : 2;
            uint32_t val = 0;
            if (readK5100Attr(session, k5100::kCipClassParameter, pid, k5100::kCipAttributeValue, sz, val))
            {
                ESP_LOGI(TAG, "K5100 %c P%u (%uB) = %lu", static_cast<char>(toupper(static_cast<unsigned char>(ax))),
                         static_cast<unsigned>(pid), static_cast<unsigned>(sz), static_cast<unsigned long>(val));
            }
            else
            {
                ESP_LOGE(TAG, "K5100 %c P%u read failed", static_cast<char>(toupper(static_cast<unsigned char>(ax))),
                         static_cast<unsigned>(pid));
            }
        }
        else if (strcmp(a1, "set") == 0 && n >= 5)
        {
            if (cfg->gantry->isBusy())
            {
                ESP_LOGE(TAG, "drv k5100 set: gantry busy — pause/stop first");
                session.unregisterSession();
                tcp.close();
                return;
            }
            // Optional 5th arg for 4-byte size: drv k5100 set x 659 12345 4
            char a5[16] = {};
            const int n5 = sscanf(cmd, "%*s %15s %31s %31s %31s %31s %15s", sub, a1, a2, a3, a4, a5);
            const uint8_t sz = (n5 >= 6 && atoi(a5) == 4) ? 4 : 2;
            const uint32_t new_val = static_cast<uint32_t>(strtol(a4, nullptr, 10));
            uint32_t before = 0;
            uint32_t after = 0;
            readK5100Attr(session, k5100::kCipClassParameter, pid, k5100::kCipAttributeValue, sz, before);
            const eip::Bytes req = k5100::buildSetParameterRequest(pid, new_val, sz);
            eip::Bytes reply;
            bool ok = false;
            if (session.sendExplicit(req, reply))
            {
                eip::MessageRouterResponse mr;
                ok = eip::parseMessageRouterResponse(reply, mr) && mr.isSuccess();
            }
            readK5100Attr(session, k5100::kCipClassParameter, pid, k5100::kCipAttributeValue, sz, after);
            ESP_LOGI(TAG, "K5100 %c set P%u=%lu (%uB) -> %s (before=%lu after=%lu)",
                     static_cast<char>(toupper(static_cast<unsigned char>(ax))),
                     static_cast<unsigned>(pid), static_cast<unsigned long>(new_val),
                     static_cast<unsigned>(sz), ok ? "OK" : "FAIL",
                     static_cast<unsigned long>(before), static_cast<unsigned long>(after));
        }
        else
        {
            ESP_LOGE(TAG, "Usage: drv k5100 <get|set> <x|z> <param_id> [val] [2|4]");
        }
        session.unregisterSession();
        tcp.close();
        return;
    }

    ESP_LOGE(TAG, "Unknown drv subcommand: %s", sub);
}

}  // namespace GantryConsoleImpl
