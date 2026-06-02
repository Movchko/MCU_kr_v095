#include "app.h"

extern "C" {
#include "backend.h"
}

#include "device_config.h"
#include "device_relay.hpp"
#include "mku_cfg_flash.h"
#include "stm32h5xx_hal.h"
#include "stm32h5xx_hal_flash.h"
#include "stm32h5xx_hal_flash_ex.h"

#include <string.h>

#include "main.h"

#define FLASH_CFG_SECTOR     (31u)
#define MKU_CFG_HEADER_MAGIC 0x4D4B5543u

static MKUCfg g_cfg;
static MKUCfg g_saved_cfg;
static uint32_t g_ppku_last_seen_ms = 0u;
static const uint32_t PPKU_ONLINE_TIMEOUT_MS = 5000u;

/* 1: Relay1, 2: Relay2 */
static VDeviceRelay g_relay1(1);
static VDeviceRelay g_relay2(2);

static void Relay1_SetOutput(uint8_t state)
{
    HAL_GPIO_WritePin(Relay1_GPIO_Port, Relay1_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void Relay2_SetOutput(uint8_t state)
{
    HAL_GPIO_WritePin(Relay2_GPIO_Port, Relay2_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint8_t Relay1_GetFeedback(void)
{
    GPIO_PinState cod = HAL_GPIO_ReadPin(Relay1_COD_GPIO_Port, Relay1_COD_Pin);
    GPIO_PinState csc = HAL_GPIO_ReadPin(Relay1_CSC_GPIO_Port, Relay1_CSC_Pin);
    if (cod == GPIO_PIN_SET && csc == GPIO_PIN_RESET) return 1u;
    if (cod == GPIO_PIN_RESET && csc == GPIO_PIN_SET) return 0u;
    return (cod == GPIO_PIN_SET) ? 1u : 0u;
}

static uint8_t Relay2_GetFeedback(void)
{
    GPIO_PinState cod = HAL_GPIO_ReadPin(Relay2_COD_GPIO_Port, Relay2_COD_Pin);
    GPIO_PinState csc = HAL_GPIO_ReadPin(Relay2_CSC_GPIO_Port, Relay2_CSC_Pin);
    if (cod == GPIO_PIN_SET && csc == GPIO_PIN_RESET) return 1u;
    if (cod == GPIO_PIN_RESET && csc == GPIO_PIN_SET) return 0u;
    return (cod == GPIO_PIN_SET) ? 1u : 0u;
}

static void VDeviceSetStatus(uint8_t DNum, uint8_t Code, const uint8_t *Parameters)
{
    uint8_t data[7] = {0};
    for (uint8_t i = 0; i < 7; i++) {
        data[i] = Parameters[i];
    }
    SendMessage(DNum, Code, data, 0, BUS_CAN12);
}

static uint8_t IsPpkuOnline(uint32_t now_ms)
{
    return ((now_ms - g_ppku_last_seen_ms) <= PPKU_ONLINE_TIMEOUT_MS) ? 1u : 0u;
}

static void Relay_ApplyTarget(VDeviceRelay &relay, uint8_t state)
{
    uint8_t p[1] = { (state != 0u) ? 1u : 0u };
    relay.CommandCB(10u, p);
}

static void Relay_HandleAutonomousFire(const DeviceRelayConfig *cfg, VDeviceRelay &relay, uint8_t fire_active)
{
    if (cfg == nullptr || cfg->mode != 1u) {
        return;
    }
    if (fire_active != 0u) {
        Relay_ApplyTarget(relay, (cfg->initial_state != 0u) ? 0u : 1u);
    } else {
        Relay_ApplyTarget(relay, (cfg->initial_state != 0u) ? 1u : 0u);
    }
}

void SetHAdr(uint8_t h_adr)
{
    g_cfg.UId.devId.h_adr = h_adr;
    extern uint8_t nDevs;
    extern Device BoardDevicesList[];
    for (uint8_t i = 0; i < nDevs; i++) {
        BoardDevicesList[i].h_adr = g_cfg.UId.devId.h_adr;
    }
    SaveConfig();
}

extern "C" {

void DefaultConfig(void)
{
    uint32_t uid0 = HAL_GetUIDw0();
    uint32_t uid1 = HAL_GetUIDw1();
    uint32_t uid2 = HAL_GetUIDw2();

    memset(&g_cfg, 0, sizeof(g_cfg));

    g_cfg.UId.UId0 = uid0;
    g_cfg.UId.UId1 = uid1;
    g_cfg.UId.UId2 = uid2;
    g_cfg.UId.UId3 = HAL_GetDEVID();
    g_cfg.UId.UId4 = 1;

    g_cfg.UId.devId.zone  = 0;
    g_cfg.UId.devId.l_adr = 0;

    uint8_t hadr = (uint8_t)(uid0 & 0xFFu);
    if (hadr == 0u) {
        hadr = (uint8_t)(uid1 & 0xFFu);
        if (hadr == 0u) {
            hadr = 1u;
        }
    }
    g_cfg.UId.devId.h_adr = hadr;
    g_cfg.UId.devId.d_type = DEVICE_MCU_KR;

    g_cfg.VDtype[0] = DEVICE_RELAY_TYPE;
    g_cfg.VDtype[1] = DEVICE_RELAY_TYPE;

    DeviceRelayConfig *r1 = (DeviceRelayConfig*)g_cfg.Devices[0].reserv;
    DeviceRelayConfig *r2 = (DeviceRelayConfig*)g_cfg.Devices[1].reserv;
    memset(r1, 0, sizeof(DeviceRelayConfig));
    memset(r2, 0, sizeof(DeviceRelayConfig));
    r1->mode = 0u;
    r1->initial_state = 0u;
    r1->persist_state_enabled = 0u;
    r1->feedback_inverted = 0u;
    r1->switch_delay_s = 0u;
    r1->settle_time_ms = 100u;
    r2->mode = 0u;
    r2->initial_state = 0u;
    r2->persist_state_enabled = 0u;
    r2->feedback_inverted = 0u;
    r2->switch_delay_s = 0u;
    r2->settle_time_ms = 100u;
}

uint32_t GetConfigSize(void) { return (uint32_t)sizeof(g_cfg); }

uint32_t GetConfigWord(uint16_t num)
{
    uint32_t byte_index = (uint32_t)num * 4u;
    uint32_t cfg_size = GetConfigSize();
    if (byte_index + 4u > cfg_size) return 0u;
    uint8_t *p = (uint8_t *)&g_cfg;
    uint32_t word = 0u;
    word |= ((uint32_t)p[byte_index + 0] << 24);
    word |= ((uint32_t)p[byte_index + 1] << 16);
    word |= ((uint32_t)p[byte_index + 2] << 8);
    word |= ((uint32_t)p[byte_index + 3] << 0);
    return word;
}

void SetConfigWord(uint16_t num, uint32_t word)
{
    uint32_t byte_index = (uint32_t)num * 4u;
    uint32_t cfg_size = GetConfigSize();
    if (byte_index + 4u > cfg_size) return;
    uint8_t *p = (uint8_t *)&g_cfg;
    p[byte_index + 0] = (uint8_t)((word >> 24) & 0xFFu);
    p[byte_index + 1] = (uint8_t)((word >> 16) & 0xFFu);
    p[byte_index + 2] = (uint8_t)((word >> 8) & 0xFFu);
    p[byte_index + 3] = (uint8_t)((word >> 0) & 0xFFu);
}

#define MKU_CFG_HEADER_SIZE 8u
#define QUADWORD_SIZE       16u

static bool FlashReadConfig(MKUCfg *out)
{
    if (out == nullptr) return false;
    const uint32_t *p = (const uint32_t *)FLASH_CFG_ADDR;
    if (p[0] != MKU_CFG_HEADER_MAGIC) return false;
    uint32_t sz = p[1];
    if (sz != sizeof(MKUCfg) || sz > FLASH_CFG_SIZE - MKU_CFG_HEADER_SIZE) return false;
    memcpy(out, p + 2, sz);
    return true;
}

void FlashWriteData(uint8_t *ConfigPtr, uint32_t ConfigSize)
{
    if (ConfigPtr == nullptr || ConfigSize != sizeof(MKUCfg) ||
        ConfigSize > FLASH_CFG_SIZE - MKU_CFG_HEADER_SIZE) {
        return;
    }

    __attribute__((aligned(16))) uint8_t buf[FLASH_CFG_SIZE_BYTES];
    uint32_t *hdr = (uint32_t *)buf;
    hdr[0] = MKU_CFG_HEADER_MAGIC;
    hdr[1] = ConfigSize;
    memcpy(buf + MKU_CFG_HEADER_SIZE, ConfigPtr, ConfigSize);

    uint32_t total = MKU_CFG_HEADER_SIZE + ConfigSize;
    uint32_t n_quad = (total + QUADWORD_SIZE - 1u) / QUADWORD_SIZE;

    FLASH_EraseInitTypeDef erase;
    uint32_t sector_err = 0u;
#if defined(FLASH_TYPEERASE_SECTORS_NS)
    erase.TypeErase = FLASH_TYPEERASE_SECTORS_NS;
#else
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
#endif
    erase.Banks = FLASH_BANK_2;
    erase.Sector = FLASH_CFG_SECTOR;
    erase.NbSectors = 1;

    HAL_StatusTypeDef st = HAL_FLASH_Unlock();
    if (st != HAL_OK) return;
    st = HAL_FLASHEx_Erase(&erase, &sector_err);
    if (st != HAL_OK) {
        HAL_FLASH_Lock();
        return;
    }
#if defined(FLASH_TYPEPROGRAM_QUADWORD_NS)
    uint32_t prog_type = FLASH_TYPEPROGRAM_QUADWORD_NS;
#else
    uint32_t prog_type = FLASH_TYPEPROGRAM_QUADWORD;
#endif
    for (uint32_t i = 0u; i < n_quad && st == HAL_OK; i++) {
        uint32_t addr = FLASH_CFG_ADDR + i * QUADWORD_SIZE;
        st = HAL_FLASH_Program(prog_type, addr, (uint32_t)(buf + i * QUADWORD_SIZE));
    }
    HAL_FLASH_Lock();
}

void SaveConfig(void)
{
    uint32_t size = GetConfigSize();
    FlashWriteData((uint8_t *)&g_cfg, size);
    g_saved_cfg = g_cfg;
}

void ResetMCU(void) { NVIC_SystemReset(); }

uint32_t GetID(void)
{
    uint32_t id0 = HAL_GetUIDw0();
    uint32_t id1 = HAL_GetUIDw1();
    uint32_t id2 = HAL_GetUIDw2();
    return (id0 ^ id1 ^ id2);
}

void MCU_KRCommandCB(uint8_t Command, uint8_t *Parameters)
{
    if (Command == 20) {
        g_cfg.UId.devId.zone = Parameters[0];
        SaveConfig();
    }
}

void CommandCB(uint8_t Dev, uint8_t Command, uint8_t *Parameters)
{
    switch (Dev) {
    case 0: MCU_KRCommandCB(Command, Parameters); break;
    case 1: g_relay1.CommandCB(Command, Parameters); break;
    case 2: g_relay2.CommandCB(Command, Parameters); break;
    default: break;
    }
}

void ListenerCommandCB(uint32_t MsgID, uint8_t *MsgData)
{
    if (MsgData == nullptr) {
        return;
    }

    can_ext_id_t id;
    id.ID = MsgID;
    uint32_t now = HAL_GetTick();

    if (id.field.dir == 1u && id.field.d_type == DEVICE_PPKY_TYPE) {
        g_ppku_last_seen_ms = now;
    }

    if (IsPpkuOnline(now) != 0u) {
        return;
    }

    if (id.field.dir != 1u) {
        return;
    }

    const uint8_t cmd = MsgData[0];
    const uint8_t own_zone = (uint8_t)(g_cfg.UId.devId.zone & 0x7Fu);
    const uint8_t zone_match = (id.field.zone == own_zone || id.field.zone == 0u) ? 1u : 0u;
    if (zone_match == 0u) {
        return;
    }

    DeviceRelayConfig *r1 = (DeviceRelayConfig*)g_cfg.Devices[0].reserv;
    DeviceRelayConfig *r2 = (DeviceRelayConfig*)g_cfg.Devices[1].reserv;
    if (cmd == ServiceCmd_Fire_SetStatusFire) {
        Relay_HandleAutonomousFire(r1, g_relay1, 1u);
        Relay_HandleAutonomousFire(r2, g_relay2, 1u);
    } else if (cmd == ServiceCmd_Fire_StopExtinguishment) {
        Relay_HandleAutonomousFire(r1, g_relay1, 0u);
        Relay_HandleAutonomousFire(r2, g_relay2, 0u);
    }
}

void App_Init(void)
{
    extern Device BoardDevicesList[];
    extern uint8_t nDevs;

    if (!FlashReadConfig(&g_cfg)) {
        DefaultConfig();
        SaveConfig();
    }
    g_ppku_last_seen_ms = HAL_GetTick() - (PPKU_ONLINE_TIMEOUT_MS + 1u);
    g_saved_cfg = g_cfg;
    SetConfigPtr((uint8_t *)&g_saved_cfg, (uint8_t *)&g_cfg);

    g_relay1.DeviceInit(&g_cfg.Devices[0]);
    g_relay1.VDeviceSetStatus = VDeviceSetStatus;
    g_relay1.VDeviceSaveCfg = SaveConfig;
    g_relay1.Relay_SetOutput = Relay1_SetOutput;
    g_relay1.Relay_GetFeedback = Relay1_GetFeedback;
    g_relay1.Init();

    g_relay2.DeviceInit(&g_cfg.Devices[1]);
    g_relay2.VDeviceSetStatus = VDeviceSetStatus;
    g_relay2.VDeviceSaveCfg = SaveConfig;
    g_relay2.Relay_SetOutput = Relay2_SetOutput;
    g_relay2.Relay_GetFeedback = Relay2_GetFeedback;
    g_relay2.Init();

    nDevs = 1;
    BoardDevicesList[0].zone  = g_cfg.UId.devId.zone;
    BoardDevicesList[0].h_adr = g_cfg.UId.devId.h_adr;
    BoardDevicesList[0].l_adr = g_cfg.UId.devId.l_adr;
    BoardDevicesList[0].d_type = DEVICE_MCU_KR;

    if (nDevs < MAX_DEVS) {
        BoardDevicesList[nDevs].zone = g_cfg.UId.devId.zone;
        BoardDevicesList[nDevs].h_adr = g_cfg.UId.devId.h_adr;
        BoardDevicesList[nDevs].l_adr = 1;
        BoardDevicesList[nDevs].d_type = g_relay1.GetDT();
        nDevs++;
    }
    if (nDevs < MAX_DEVS) {
        BoardDevicesList[nDevs].zone = g_cfg.UId.devId.zone;
        BoardDevicesList[nDevs].h_adr = g_cfg.UId.devId.h_adr;
        BoardDevicesList[nDevs].l_adr = 2;
        BoardDevicesList[nDevs].d_type = g_relay2.GetDT();
        nDevs++;
    }

    extern bool isListener;
    isListener = true;
}

void App_Timer1ms(void)
{
    static uint16_t led_cnt = 0u;
    static uint16_t status_cnt = 0u;
    uint32_t now = HAL_GetTick();

    extern Device BoardDevicesList[];
    BoardDevicesList[1].d_type = g_relay1.GetDT();
    BoardDevicesList[2].d_type = g_relay2.GetDT();

    if (status_cnt < 1000u) {
        status_cnt++;
    } else {
        status_cnt = 0u;
        uint8_t status_data[7] = {0};
        status_data[0] = (uint8_t)(now / 1000u);
        status_data[1] = 0u;
        status_data[2] = 0u;
        status_data[3] = 0u;
        status_data[4] = (uint8_t)(CAN1_Active | (CAN2_Active << 1));

        const uint32_t VREF_MV = 3300u;
        const uint32_t ADC_MAX = 4095u;
        const uint32_t DIV_K   = 11u;
        uint32_t raw_u24 = ADC_GetU24Filtered();
        uint32_t v_adc_mv = (raw_u24 * VREF_MV) / ADC_MAX;
        uint32_t u24_mv = v_adc_mv * DIV_K;
        uint32_t code_1v = u24_mv / 1000u;
        if (code_1v > 255u) code_1v = 255u;
        status_data[5] = (uint8_t)code_1v;
        status_data[6] = App_GetCanStateMask();
        SendMessage(0, 0, status_data, SEND_NOW, BUS_CAN12);

        uint8_t pos_data[7] = {0u, 0u, 0u, 0u, 0u, 0u, 0u};
        SendMessage(0, ServiceCmd_PositionDevice, pos_data, SEND_NOW, BUS_CAN12);
    }

    if (led_cnt < 1000u) {
        led_cnt++;
    } else {
        led_cnt = 0u;
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }

    App_UpdateCanActivity();
    g_relay1.Timer1ms();
    g_relay2.Timer1ms();

    BackendProcess();
}

} /* extern "C" */

