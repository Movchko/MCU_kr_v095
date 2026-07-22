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
#include "mku_cfg_flash.h"

extern DTS_HandleTypeDef hdts;

MKUCfg g_cfg;
MKUCfg g_saved_cfg;
static uint32_t g_ppku_last_seen_ms = 0u;
static const uint32_t PPKU_ONLINE_TIMEOUT_MS = 5000u;

/* 1: Relay1, 2: Relay2 */
static VDeviceRelay g_relay1(1);
static VDeviceRelay g_relay2(2);
static uint8_t g_relay1_fire_latched = 0u;
static uint8_t g_relay2_fire_latched = 0u;

static uint32_t ResetDelayms = 3000;
static uint8_t isReset = 0;

/* Слот 0/1: реле. VDtype[slot]==0 — канал отключён. */
static uint8_t App_IsSlotEnabled(uint8_t slot)
{
	if (slot >= NUM_DEV_IN_MCU) {
		return 0u;
	}
	return (g_cfg.VDtype[slot] != 0u) ? 1u : 0u;
}

static uint8_t App_IsRelaySlotEnabled(uint8_t slot)
{
	if (slot >= 2u) {
		return 0u;
	}
	return (g_cfg.VDtype[slot] == DEVICE_RELAY_TYPE) ? 1u : 0u;
}

static void App_RebuildBoardDevicesList(void)
{
	extern Device BoardDevicesList[];
	extern uint8_t nDevs;

	BoardDevicesList[0].zone   = g_cfg.UId.devId.zone;
	BoardDevicesList[0].h_adr  = g_cfg.UId.devId.h_adr;
	BoardDevicesList[0].l_adr  = g_cfg.UId.devId.l_adr;
	BoardDevicesList[0].d_type = DEVICE_MCU_KR;

	BoardDevicesList[1].zone   = g_cfg.UId.devId.zone;
	BoardDevicesList[1].h_adr  = g_cfg.UId.devId.h_adr;
	BoardDevicesList[1].l_adr  = 1;
	BoardDevicesList[1].d_type = App_IsRelaySlotEnabled(0) ? DEVICE_RELAY_TYPE : 0u;

	BoardDevicesList[2].zone   = g_cfg.UId.devId.zone;
	BoardDevicesList[2].h_adr  = g_cfg.UId.devId.h_adr;
	BoardDevicesList[2].l_adr  = 2;
	BoardDevicesList[2].d_type = App_IsRelaySlotEnabled(1) ? DEVICE_RELAY_TYPE : 0u;

	nDevs = 3;
}

static uint8_t App_IsBoardDevActive(uint8_t dnum)
{
	if (dnum == 1u) {
		return App_IsRelaySlotEnabled(0);
	}
	if (dnum == 2u) {
		return App_IsRelaySlotEnabled(1);
	}
	return 1u;
}

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
    if (!App_IsBoardDevActive(DNum)) {
        return;
    }
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

static uint8_t Relay_IsLatchMode(uint8_t mode)
{
    return (mode == 1u || mode == 4u || mode == 5u || mode == 6u) ? 1u : 0u;
}

static uint8_t Relay_GetLatchTargetState(const DeviceRelayConfig *cfg)
{
    if (cfg == nullptr || Relay_IsLatchMode(cfg->mode) == 0u) {
        return 0xFFu;
    }
    return (cfg->initial_state != 0u) ? 0u : 1u;
}

static uint8_t Relay_ShouldApplyCommand(uint8_t cmd, uint8_t *params,
                                        const DeviceRelayConfig *cfg,
                                        uint8_t *fire_latched)
{
    if (cmd != 10u || cfg == nullptr || params == nullptr) {
        return 1u;
    }

    const uint8_t latch_target = Relay_GetLatchTargetState(cfg);
    if (latch_target == 0xFFu) {
        return 1u;
    }

    uint8_t new_state = 0u;
    if (params[0] == 0u || params[0] == 1u) {
        new_state = params[0];
    } else {
        return 1u;
    }

    if (new_state != latch_target) {
        return 1u;
    }

    if (*fire_latched != 0u) {
        return 0u;
    }

    *fire_latched = 1u;
    return 1u;
}

static void Relay_HandleAutonomousLatch(const DeviceRelayConfig *cfg, VDeviceRelay &relay,
                                        uint8_t *fire_latched)
{
    if (cfg == nullptr || Relay_IsLatchMode(cfg->mode) == 0u) {
        return;
    }
    if (*fire_latched != 0u) {
        return;
    }
    Relay_ApplyTarget(relay, Relay_GetLatchTargetState(cfg));
    *fire_latched = 1u;
}

static void Relay_TryAutonomousFire(const DeviceRelayConfig *cfg, VDeviceRelay &relay,
                                    uint8_t *fire_latched, uint8_t zone_match)
{
    if (cfg == nullptr) {
        return;
    }
    if (cfg->mode == 1u) {
        if (zone_match == 0u) {
            return;
        }
        Relay_HandleAutonomousLatch(cfg, relay, fire_latched);
    } else if (cfg->mode == 4u) {
        /* Пожар в любой зоне — zone_match не требуется. */
        Relay_HandleAutonomousLatch(cfg, relay, fire_latched);
    }
}

static void Relay_TryAutonomousStart(const DeviceRelayConfig *cfg, VDeviceRelay &relay,
                                     uint8_t *fire_latched, uint8_t zone_match)
{
    if (cfg == nullptr) {
        return;
    }
    if (cfg->mode == 5u) {
        if (zone_match == 0u) {
            return;
        }
        Relay_HandleAutonomousLatch(cfg, relay, fire_latched);
    } else if (cfg->mode == 6u) {
        Relay_HandleAutonomousLatch(cfg, relay, fire_latched);
    }
}

void SetHAdr(uint8_t h_adr)
{
    g_cfg.UId.devId.h_adr = h_adr;
    App_RebuildBoardDevicesList();
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

void ResetMCU(void)
{
    isReset = 1;
}

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
    case 1: {
        if (!App_IsRelaySlotEnabled(0)) {
            break;
        }
        DeviceRelayConfig *cfg = (DeviceRelayConfig*)g_cfg.Devices[0].reserv;
        if (Relay_ShouldApplyCommand(Command, Parameters, cfg, &g_relay1_fire_latched) != 0u) {
            g_relay1.CommandCB(Command, Parameters);
        }
    } break;
    case 2: {
        if (!App_IsRelaySlotEnabled(1)) {
            break;
        }
        DeviceRelayConfig *cfg = (DeviceRelayConfig*)g_cfg.Devices[1].reserv;
        if (Relay_ShouldApplyCommand(Command, Parameters, cfg, &g_relay2_fire_latched) != 0u) {
            g_relay2.CommandCB(Command, Parameters);
        }
    } break;
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

    DeviceRelayConfig *r1 = (DeviceRelayConfig*)g_cfg.Devices[0].reserv;
    DeviceRelayConfig *r2 = (DeviceRelayConfig*)g_cfg.Devices[1].reserv;
    if (cmd == ServiceCmd_Fire_SetStatusFire) {
        if (App_IsRelaySlotEnabled(0)) {
            Relay_TryAutonomousFire(r1, g_relay1, &g_relay1_fire_latched, zone_match);
        }
        if (App_IsRelaySlotEnabled(1)) {
            Relay_TryAutonomousFire(r2, g_relay2, &g_relay2_fire_latched, zone_match);
        }
    }
    /* StopExtinguishment / Pause: реле не возвращаем. */
}

extern "C" void RcvStartExtinguishment(uint32_t MsgID, uint8_t *MsgData, uint8_t is_mine)
{
    (void)is_mine;
    if (MsgData == nullptr) {
        return;
    }

    uint32_t now = HAL_GetTick();
    if (IsPpkuOnline(now) != 0u) {
        return;
    }

    can_ext_id_t id;
    id.ID = MsgID & 0x0FFFFFFFu;
    uint8_t zone = (uint8_t)(id.field.zone & 0x7Fu);
    if (zone == 0u) {
        zone = MsgData[1] & 0x7Fu;
    }

    const uint8_t own_zone = (uint8_t)(g_cfg.UId.devId.zone & 0x7Fu);
    const uint8_t zone_match = (zone == 0u || zone == own_zone) ? 1u : 0u;

    DeviceRelayConfig *r1 = (DeviceRelayConfig*)g_cfg.Devices[0].reserv;
    DeviceRelayConfig *r2 = (DeviceRelayConfig*)g_cfg.Devices[1].reserv;
    if (App_IsRelaySlotEnabled(0)) {
        Relay_TryAutonomousStart(r1, g_relay1, &g_relay1_fire_latched, zone_match);
    }
    if (App_IsRelaySlotEnabled(1)) {
        Relay_TryAutonomousStart(r2, g_relay2, &g_relay2_fire_latched, zone_match);
    }
}

void App_Init(void)
{
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

    App_RebuildBoardDevicesList();

    extern bool isListener;
    isListener = true;
}

void App_Timer1ms(void)
{
    static uint16_t led_cnt = 0u;
    static uint16_t status_cnt = 0u;
    uint32_t now = HAL_GetTick();

    if (status_cnt < 1000u) {
        status_cnt++;
    } else {
        status_cnt = 0u;

        int32_t temperature;
        if(HAL_DTS_GetTemperature(&hdts, &temperature)!= HAL_OK)
        {
            /* DTS GetTemperature Error */
        }

        if(temperature > 128) temperature = 128;
        if(temperature < -128) temperature = -128;

        uint8_t temp = (uint8_t)temperature;

        uint8_t status_data[7] = {0};
        status_data[0] = (uint8_t)(now / 1000u);
        status_data[1] = temp;
        status_data[2] = 0u;
        status_data[3] = 0u;
        status_data[4] = (uint8_t)(CAN1_Active | (CAN2_Active << 1));

        const uint32_t VREF_MV = 3300u;
        const uint32_t ADC_MAX = 4095u;
        uint32_t raw_u24 = ADC_GetU24Filtered();
        uint32_t v_adc_mv = (raw_u24 * VREF_MV) / ADC_MAX;
        uint32_t u24_mv = v_adc_mv;
        uint32_t code_1v = (u24_mv + 500) / 1000u;
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

    // задержка софт-рестарта. нужно чтобы усройство успело широковещательную переслать команду дальше
    if (isReset) {
        ResetDelayms--;
        if (ResetDelayms == 0u) {
            NVIC_SystemReset();
        }
    }

    if (App_IsRelaySlotEnabled(0)) {
        g_relay1.Timer1ms();
    }
    if (App_IsRelaySlotEnabled(1)) {
        g_relay2.Timer1ms();
    }

    BackendProcess();
}

void AplyConfig(void)
{
    App_RebuildBoardDevicesList();
}

} /* extern "C" */

