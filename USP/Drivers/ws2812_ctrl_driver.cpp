#include "ws2812_ctrl_driver.h"

//30 + Num * 3 * 8 + 30
#define WS2312_LED_NUM 50   // R标灯的 WS2812 LED 数量

#define PWM_DATA_LEN (WS2312_LED_NUM * 24) // 每个WS2812需要24个码元
#define WS2812_RESET_LEN 40 // 定义重置周期数（800KHz 下，1.25us/bit，40个0约 50us）
#define dma_data_len (PWM_DATA_LEN + WS2812_RESET_LEN) // DMA数据总长度

#define WS2312_0bit 38
#define WS2312_1bit 66

static uint32_t tim_pwm_dma_buff[dma_data_len] = {0};//PWM DMA数据缓存

void Buff_translate(uint8_t* color_buff,uint32_t* dma_row_ptr) //颜色数组转换为码元数组
{   
    uint32_t dat_idx = 0;
	for(uint32_t i = 0;i < (WS2312_LED_NUM*3);i++)
	{
        for(int8_t k = 7; k >= 0; k--) // MSB First: 高位先发
        {
            if ((color_buff[i] >> k) & 0x01) {
                dma_row_ptr[dat_idx++] = WS2312_1bit; 
            } else {
                dma_row_ptr[dat_idx++] = WS2312_0bit; 
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

// 灯臂填充指定颜色
/*
5*10,R标灯的 WS2812 灯珠排列如下（共50颗）：
9  10 29 30 49
8  11 28 31 48
7  12 27 32 47
6  13 26 33 46 
5  14 25 34 45
4  15 24 35 44
3  16 23 36 43
2  17 22 37 42
1  18 21 38 41
0  19 20 39 40
*  *  *    
*        *     
*        *    
*        *
*     *
*  *
*  *  
*     *
*        *   
*           *
*/
#define ws2312_map_len 22
uint8_t ws2312_map[ws2312_map_len] = {0,1,2,3,4,5,6,7,8,9,10,15,16,29,25,22,33,31,32,33,38,40};
void ws2312_show(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t temp_pixels[WS2312_LED_NUM * 3] = {0};


    for (int i = 0; i < ws2312_map_len; i++) 
    {
        temp_pixels[ws2312_map[i] * 3]     = g; // WS2812 典型为 GRB 顺序
        temp_pixels[ws2312_map[i] * 3 + 1] = r;
        temp_pixels[ws2312_map[i] * 3 + 2] = b;
    }
    // 将 RGB 数据转换为 PWM 码元
    Buff_translate(temp_pixels, tim_pwm_dma_buff);
    // 4. 非阻塞启动 4 路 DMA 传输
    HAL_TIM_PWM_Start_DMA(arm_tim1, TIM_CHANNEL_3, (uint32_t *)tim_pwm_dma_buff, dma_data_len);//主灯臂outside
    // __HAL_TIM_SET_COMPARE(arm_tim1,TIM_CHANNEL_4,50);//测试用
    // HAL_TIM_PWM_Start(arm_tim1,TIM_CHANNEL_4);
}

