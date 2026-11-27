/**
 * @file gantry_console_cfg.cpp
 * @brief Gantry test console: the cfg command and parameter save.
 *
 * One of the gantry_console_*.cpp files that make up the test console;
 * gantry_console_internal.h declares what they share.
 */

#include "gantry_console_internal.h"

namespace GantryConsoleImpl {

static constexpr uint32_t kParamSavePauseTimeoutMs = 5000;

bool cmdTokenIs(const char *cmd, const char *word) {
  size_t i = 0;
  for (; word[i] != '\0'; ++i) {
    if (tolower(static_cast<unsigned char>(cmd[i])) !=
        tolower(static_cast<unsigned char>(word[i]))) {
      return false;
    }
  }
  return cmd[i] == '\0' || isspace(static_cast<unsigned char>(cmd[i]));
}

static const char *skipCmdSpace(const char *s) {
  while (s != nullptr && *s != '\0' && isspace(static_cast<unsigned char>(*s))) {
    ++s;
  }
  return s;
}

static void formatSetTime(uint32_t unix_s, char *out, size_t out_len) {
  if (unix_s == 0) {
    snprintf(out, out_len, "no clock");
    return;
  }
  const time_t when = static_cast<time_t>(unix_s);
  struct tm utc;
  gmtime_r(&when, &utc);
  strftime(out, out_len, "%Y-%m-%dT%H:%M:%SZ", &utc);
}

// One 'PARAM key=value' line per visible parameter, replayable as 'cfg key value'.
// The toolbox page turns this into the downloadable parameter file. Passwords
// stay out of it.
static void printConfigExport() {
  auto &config = Config::AppConfig::instance();
  const Config::ConfigSetInfo set = config.activeSet();
  char when[24];
  formatSetTime(set.unix_s, when, sizeof(when));
  ESP_LOGI(TAG, "PARAM # gantry parameters, set #%lu, stored %s, %d change(s) since",
           static_cast<unsigned long>(set.seq), when, config.pendingChanges());
  for (const auto &schema : Config::getParamSchemas()) {
    if (!config.isParamVisible(schema.key) || strcmp(schema.key, "con_pw") == 0 ||
        strcmp(schema.key, "dev_pw") == 0) {
      continue;
    }
    // The display format rounds to two decimals; a file has to reload exactly.
    char value[40];
    float number = 0.0f;
    if (schema.type == Config::ParamType::FLOAT && config.getParamFloat(schema.key, number)) {
      snprintf(value, sizeof(value), "%.7g", static_cast<double>(number));
      ESP_LOGI(TAG, "PARAM %s=%s", schema.key, value);
    } else if (config.formatParamValue(schema.key, value, sizeof(value))) {
      ESP_LOGI(TAG, "PARAM %s=%s", schema.key, value);
    }
  }
  ESP_LOGI(TAG, "OK parameter export done");
}

static void printClock() {
  char when[24];
  const time_t now = time(nullptr);
  formatSetTime(now > 1700000000 ? static_cast<uint32_t>(now) : 0, when, sizeof(when));
  ESP_LOGI(TAG, "Clock %s (NTP %s)", when, stored().ntp_server);
}

// A flash write can stall both CPUs past the Class 1 timeout, so Class 1 closes
// around it and the motors must already be off.
bool saveParameters(const GantryTestConsoleConfig *cfg) {
  auto &config = Config::AppConfig::instance();
  if (cfg != nullptr && cfg->gantry != nullptr &&
      (cfg->gantry->isEnabled() || cfg->gantry->isBusy())) {
    ESP_LOGE(TAG, "ERROR: parameters not saved: run 'stop' first, motors must be off");
    return false;
  }
  if (!config.hasStorage()) {
    ESP_LOGE(TAG, "ERROR: parameters not saved: NVS is unavailable ('cfg factory' erases it)");
    return false;
  }
  if (config.pendingChanges() == 0 && config.activeSet().valid) {
    ESP_LOGI(TAG, "OK nothing to save, set #%lu is current",
             static_cast<unsigned long>(config.activeSet().seq));
    return true;
  }
  if (!eip::pauseClass1(kParamSavePauseTimeoutMs)) {
    ESP_LOGE(TAG, "ERROR: parameters not saved: Class 1 did not close");
    return false;
  }
  const bool saved = config.saveSet();
  eip::resumeClass1();
  if (!saved) {
    ESP_LOGE(TAG, "ERROR: NVS write failed; values stay in RAM only");
    return false;
  }
  ESP_LOGI(TAG, "OK saved set #%lu. Class 1 is reopening; wait for it, then 'enable'",
           static_cast<unsigned long>(config.activeSet().seq));
  return true;
}

// A stored value is the only one in force, so a change reaches the Gantry now.
// Reboot-only keys still wait for the boot that applies them.
static void applyStoredToMotion(const GantryTestConsoleConfig *cfg, const char *key) {
  if (cfg != nullptr && cfg->gantry != nullptr) {
    cfg->gantry->setRuntimeParams(gantryRuntimeParams(stored()));
  }
  static constexpr const char *kMoveProfileKeys[] = {
      "spd_mm_s", "acc_mm_s2", "dec_mm_s2", "th_spd_deg_s", "th_acc_deg_s2", "th_dec_deg_s2"};
  for (const char *profile_key : kMoveProfileKeys) {
    if (key == nullptr || strcmp(key, profile_key) == 0) {
      gantryConsoleLoadStoredProfile();
      return;
    }
  }
}

void runCfgCommand(const GantryTestConsoleConfig *cfg, const char *cmd) {
  const char *args = skipCmdSpace(cmd + 3);
  auto &config = Config::AppConfig::instance();

  if (args == nullptr || *args == '\0') {
    const char *section = "";
    for (const auto &schema : Config::getParamSchemas()) {
      if (!config.isParamVisible(schema.key)) {
        continue;
      }
      if (strcmp(section, schema.section) != 0) {
        section = schema.section;
        ESP_LOGI(TAG, "--- %s ---", section);
      }
      char value[40];
      if (!config.formatParamValue(schema.key, value, sizeof(value))) {
        snprintf(value, sizeof(value), "?");
      }
      ESP_LOGI(TAG, "  %-16s %-16s = %s %s%s", schema.key, schema.label, value,
               schema.units, schema.reboot_required ? "  (reboot)" : "");
    }
    ESP_LOGI(TAG, "cfg <key> [value] | cfg unlock <pw> | cfg lock | "
                  "cfg passwd <old> <new> | cfg save | cfg export | cfg time | "
                  "cfg factory | cfg reboot [discard]");
    return;
  }

  char buf[160];
  strncpy(buf, args, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';

  char *token = buf;
  char *rest = buf;
  while (*rest != '\0' && !isspace(static_cast<unsigned char>(*rest))) {
    ++rest;
  }
  if (*rest != '\0') {
    *rest = '\0';
    rest = const_cast<char *>(skipCmdSpace(rest + 1));
  } else {
    rest = const_cast<char *>(rest);
  }
  for (char *p = token; *p != '\0'; ++p) {
    *p = static_cast<char>(tolower(static_cast<unsigned char>(*p)));
  }

  if (cmdTokenIs(token, "reboot") && (*rest == '\0' || strcmp(rest, "discard") == 0)) {
    if (*rest == '\0' && config.pendingChanges() > 0) {
      ESP_LOGE(TAG, "ERROR: %d unsaved change(s). 'cfg save' first, or 'cfg reboot discard'",
               config.pendingChanges());
      return;
    }
    ESP_LOGI(TAG, "Rebooting");
    // Over TCP the console ends the reply and closes the client first;
    // restarting here sent no prompt and no FIN, and the supervisor's console
    // stayed busy for its 180 s deadline (OPEN_ITEMS S-7).
    if (!gantryNetConsoleDeferRestart()) {
        esp_restart();
    }
    return;
  }
  if (cmdTokenIs(token, "lock") && *rest == '\0') {
    config.lockDeveloperMode();
    ESP_LOGI(TAG, "OK developer mode locked");
    return;
  }
  if (cmdTokenIs(token, "save") && *rest == '\0') {
    saveParameters(cfg);
    return;
  }
  if (cmdTokenIs(token, "export") && *rest == '\0') {
    printConfigExport();
    return;
  }
  if (cmdTokenIs(token, "time") && *rest == '\0') {
    printClock();
    return;
  }
  if (cmdTokenIs(token, "factory") && *rest == '\0') {
    if (!config.hasStorage()) {
      // NVS was unreadable at boot; this is the operator asking to start over.
      if (nvs_flash_erase() != ESP_OK || nvs_flash_init() != ESP_OK || !config.init()) {
        ESP_LOGE(TAG, "ERROR: NVS erase failed");
        return;
      }
      ESP_LOGI(TAG, "OK NVS erased and a new set stored");
      return;
    }
    if (!config.factoryReset()) {
      ESP_LOGE(TAG, "ERROR: factory reset failed");
      return;
    }
    applyStoredToMotion(cfg, nullptr);
    ESP_LOGI(TAG, "OK compile-time defaults in RAM, not saved. 'cfg save' stores them over the "
                  "set; download the parameter file first to keep the old values");
    return;
  }
  if (cmdTokenIs(token, "unlock")) {
    if (*rest == '\0') {
      ESP_LOGE(TAG, "Usage: cfg unlock <password>");
      return;
    }
    if (!config.unlockDeveloperMode(rest)) {
      ESP_LOGE(TAG, "ERROR: developer unlock failed");
      return;
    }
    ESP_LOGI(TAG, "OK developer mode unlocked");
    return;
  }
  if (cmdTokenIs(token, "passwd")) {
    if (!config.isDeveloperModeUnlocked()) {
      ESP_LOGE(TAG, "ERROR: developer mode is locked");
      return;
    }
    char *old_pw = rest;
    char *new_pw = rest;
    while (*new_pw != '\0' && !isspace(static_cast<unsigned char>(*new_pw))) {
      ++new_pw;
    }
    if (*new_pw == '\0') {
      ESP_LOGE(TAG, "Usage: cfg passwd <old> <new>");
      return;
    }
    *new_pw = '\0';
    new_pw = const_cast<char *>(skipCmdSpace(new_pw + 1));
    if (*new_pw == '\0') {
      ESP_LOGE(TAG, "Usage: cfg passwd <old> <new>");
      return;
    }
    if (!config.changeDeveloperPassword(old_pw, new_pw)) {
      ESP_LOGE(TAG, "ERROR: developer password change failed");
      return;
    }
    ESP_LOGI(TAG, "OK developer password changed");
    return;
  }

  // Keys renamed on 2026-11-16 still work under their old name.
  if (const char *renamed = Config::renamedParamKey(token)) {
    ESP_LOGI(TAG, "%s is now %s", token, renamed);
    token = const_cast<char *>(renamed);
  }
  const Config::ParamSchema *schema = Config::findParamSchema(token);
  if (schema == nullptr) {
    ESP_LOGE(TAG, "ERROR: unknown config key '%s'", token);
    return;
  }
  if (!config.isParamVisible(token)) {
    ESP_LOGE(TAG, "ERROR: '%s' requires developer mode ('cfg unlock <password>')", token);
    return;
  }

  if (*rest == '\0') {
    char value[40];
    if (!config.formatParamValue(token, value, sizeof(value))) {
      ESP_LOGE(TAG, "ERROR: cannot read '%s'", token);
      return;
    }
    ESP_LOGI(TAG, "%s (%s) = %s %s%s", token, schema->label, value, schema->units,
             schema->reboot_required ? "  (reboot)" : "");
    if (schema->description != nullptr) {
      ESP_LOGI(TAG, "  %s", schema->description);
    }
    return;
  }

  bool reboot = false;
  if (!config.applyParamText(token, rest, reboot)) {
    ESP_LOGE(TAG, "ERROR: rejected %s = %s", token, rest);
    return;
  }
  applyStoredToMotion(cfg, token);
  char value[40];
  const char *shown = value;
  if (!config.formatParamValue(token, value, sizeof(value))) {
    shown = rest;
  }
  ESP_LOGI(TAG, "OK %s = %s %s%s", token, shown, schema->units,
           reboot ? "  (reboot required; 'cfg save', then 'cfg reboot')" : "  (not saved)");
}

}  // namespace GantryConsoleImpl
