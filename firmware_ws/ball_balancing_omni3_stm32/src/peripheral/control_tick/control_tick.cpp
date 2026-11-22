#include "peripheral/control_tick/control_tick.hpp"

#include <soc.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_tim.h>
#include <zephyr/devicetree.h>
#include <zephyr/irq.h>

namespace bb {

namespace {

// ハードウェアレジスタへの不変アドレス。指先の const 化は
// レジスタ書込みを妨げるため NOLINT。
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TIM_TypeDef* const TIMER =
    reinterpret_cast<TIM_TypeDef*>(DT_REG_ADDR(DT_NODELABEL(timers6)));

// ティック供給用セマフォ。K_SEM_DEFINE で定数初期化する
// （実行時コンストラクタは走らない前提のため）。
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
K_SEM_DEFINE(tick_sem, 0, 1);

}  // namespace

bool control_tick::init() {
  // timers6 は Zephyr ドライバに紐付かないため、ペリフェラルクロックは
  // ここで明示的に供給する（供給なしのレジスタ書込みは破棄される）。
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);
  // 1 MHz / (ARR+1) = 1 kHz
  LL_TIM_SetPrescaler(TIMER, DT_PROP(DT_NODELABEL(timers6), st_prescaler));
  LL_TIM_SetAutoReload(TIMER, 999U);
  LL_TIM_GenerateEvent_UPDATE(TIMER);
  LL_TIM_ClearFlag_UPDATE(TIMER);

  irq_connect_dynamic(TIM6_DAC_IRQn, 2, tick_isr_cb, nullptr, 0);
  irq_enable(TIM6_DAC_IRQn);
  return true;
}

void control_tick::start() {
  k_sem_reset(&tick_sem);
  LL_TIM_EnableIT_UPDATE(TIMER);
  LL_TIM_EnableCounter(TIMER);
}

void control_tick::stop() {
  LL_TIM_DisableIT_UPDATE(TIMER);
  LL_TIM_DisableCounter(TIMER);
}

void control_tick::wait() {
  k_sem_take(&tick_sem, K_FOREVER);
}

struct k_sem* control_tick::sem() {
  return &tick_sem;
}

void control_tick::tick_isr_cb(const void* arg) {
  ARG_UNUSED(arg);
  LL_TIM_ClearFlag_UPDATE(TIMER);
  k_sem_give(&tick_sem);
}

}  // namespace bb
