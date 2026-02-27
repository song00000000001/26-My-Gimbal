#include "ws2812_ctrl_driver.h"

//30 + Num * 3 * 8 + 30
#define WS2312_LED_NUM 20   // R标灯的 WS2812 LED 数量

#define PWM_DATA_LEN (WS2312_LED_NUM * 24) // 每个WS2812需要24个码元
#define WS2812_RESET_LEN 40 // 定义重置周期数（800KHz 下，1.25us/bit，40个0约 50us）
#define dma_data_len (PWM_DATA_LEN + WS2812_RESET_LEN) // DMA数据总长度

#define WS2312_0bit 38
#define WS2312_1bit 66

static uint16_t tim_pwm_dma_buff[dma_data_len] = {0};//PWM DMA数据缓存

void Buff_translate(uint8_t* color_buff,uint16_t* dma_row_ptr) //颜色数组转换为码元数组
{   
    uint32_t dat_idx = 0;
	for(uint32_t i = 0;i < (WS2312_LED_NUM*3);i++)
	{
        for(int8_t k = 7; k >= 0; k--) // MSB First: 高位先发
        {
            if ((color_buff[i] >> k) & 0x01) {
                dma_row_ptr[dat_idx++] = WS2312_1bit; // 50
            } else {
                dma_row_ptr[dat_idx++] = WS2312_0bit; // 29
            }
        }
	}
}


// DMA 完成回调函数
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    // 判定是哪个定时器触发的
    if(htim==arm_tim1) {
        // 传输完成后立即停止 DMA
        // 停止顺序：先停通道，如果有必要可以手动把 CCR 清零
        HAL_TIM_PWM_Stop_DMA(htim, arm_channel_1);
        // 强制清零 CCR，防止停止瞬间引脚保持高电平
        __HAL_TIM_SET_COMPARE(htim, arm_channel_1, 0);
    }
}         

// 灯臂全部填充指定颜色
void ws2312_show(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t temp_pixels[WS2312_LED_NUM * 3] = {0};

    for (int i = 0; i < WS2312_LED_NUM; i++) 
    {
        temp_pixels[i * 3]     = g; // WS2812 典型为 GRB 顺序
        temp_pixels[i * 3 + 1] = r;
        temp_pixels[i * 3 + 2] = b;
    }
    // 将 RGB 数据转换为 PWM 码元
    Buff_translate(temp_pixels, tim_pwm_dma_buff);
    // 4. 非阻塞启动 4 路 DMA 传输
    HAL_TIM_PWM_Start_DMA(arm_tim1, arm_channel_1, (uint32_t *)tim_pwm_dma_buff, dma_data_len);//主灯臂outside
}


