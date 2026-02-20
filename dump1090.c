#incude <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <signal.h>
#include <unistd.h>
#include <iio.h>

#define CENTER_FREQ 315020000
#define SAMPLE_RATE 1000000
#define CHUNK_SIZE 200000  // 每次读取 0.2 秒
#define SNAPSHOT_CHUNKS 5  // 快照总长度: 1个前置 + 1个触发 + 3个后置 = 1秒

int stop = 0;

void sigint_handler(int sig) {
    printf("\n[!] 收到退出信号，正在安全关闭硬件...\n");
    stop = 1;
}

// 脉冲结构体
typedef struct {
    char state;
    int duration;
} Pulse;

// 简单的二进制转十六进制打印
void print_hex(const char *bits, int len) {
    printf(" 🔑 滚动码 (Hex) : ");
    int byte_val = 0;
    int bit_count = 0;
    for (int i = 0; i < len; i++) {
        byte_val = (byte_val << 1) | (bits[i] - '0');
        bit_count++;
        if (bit_count == 8) {
            printf("%02X ", byte_val);
            byte_val = 0;
            bit_count = 0;
        }
    }
    // 处理末尾不足 8 位的数据
    if (bit_count > 0) {
        byte_val = byte_val << (8 - bit_count);
        printf("%02X ", byte_val);
    }
    printf("\n");
}

// 核心离线解码函数
void analyze_packet(int16_t *i_data, int16_t *q_data, int total_samples, double i_leak, double q_leak) {
    int decimation = 5;
    int dec_len = total_samples / decimation;
    double *mag = (double *)malloc(dec_len * sizeof(double));
    
    // 1. 抽取、去直流并计算幅度 (包络)
    for (int i = 0; i < dec_len; i++) {
        int idx = i * decimation;
        double di = (double)i_data[idx] - i_leak;
        double dq = (double)q_data[idx] - q_leak;
        mag[i] = sqrt(di * di + dq * dq);
    }

    // 2. 指数滑动平均滤波 (平滑毛刺)
    double alpha = 0.2; 
    double smoothed = mag[0];
    double peak_val = 0.0;
    
    for (int i = 0; i < dec_len; i++) {
        smoothed = alpha * mag[i] + (1.0 - alpha) * smoothed;
        mag[i] = smoothed; 
        if (smoothed > peak_val) {
            peak_val = smoothed;
        }
    }

    // 3. 动态二值化切割
    double threshold = peak_val * 0.4;
    int *binary = (int *)malloc(dec_len * sizeof(int));
    for (int i = 0; i < dec_len; i++) {
        binary[i] = (mag[i] > threshold) ? 1 : 0;
    }

    // 4. 提取 H/L 脉冲序列
    Pulse *pulses = (Pulse *)malloc(dec_len * sizeof(Pulse));
    int pulse_cnt = 0;
    int last_bit = binary[0];
    int duration = 1;

    for (int i = 1; i < dec_len; i++) {
        if (binary[i] == last_bit) {
            duration++;
        } else {
            pulses[pulse_cnt].state = (last_bit == 1) ? 'H' : 'L';
            pulses[pulse_cnt].duration = duration;
            pulse_cnt++;
            last_bit = binary[i];
            duration = 1;
        }
    }
    pulses[pulse_cnt].state = (last_bit == 1) ? 'H' : 'L';
    pulses[pulse_cnt].duration = duration;
    pulse_cnt++;

    // 剥除头尾的低电平静默期
    int start_idx = 0;
    while (start_idx < pulse_cnt && pulses[start_idx].state == 'L') start_idx++;
    int end_idx = pulse_cnt - 1;
    while (end_idx >= 0 && pulses[end_idx].state == 'L') end_idx--;

    // 过滤掉极短的环境噪音毛刺 (小于 20 个采样点)
    Pulse *clean_pulses = (Pulse *)malloc((end_idx - start_idx + 1) * sizeof(Pulse));
    int clean_cnt = 0;
    for (int i = start_idx; i <= end_idx; i++) {
        if (pulses[i].duration > 20) {
            clean_pulses[clean_cnt++] = pulses[i];
        }
    }

    if (clean_cnt < 30) {
        free(clean_pulses);
        free(mag);
        free(binary);
        free(pulses);
        return; 
    }

    printf("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    printf("📸 [快照成功] 截获射频包，正在进行分析...\n");
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");

    // ========================================================
    // 新增：强制输出所有接收到的底层脉宽元数据
    // ========================================================
    printf(" 📊 【底层脉宽元数据 (Raw Data)】:\n   ");
    for (int i = 0; i < clean_cnt; i++) {
        printf("%c%d ", clean_pulses[i].state, clean_pulses[i].duration);
        // 每 8 个脉冲换行，保持队形整齐
        if ((i + 1) % 8 == 0) {
            printf("\n   ");
        }
    }
    printf("\n----------------------------------------------------------\n");

    // ========================================================
    // 自动指纹锚定逻辑 (寻找 H93 L150)
    // ========================================================
    int sync_idx = -1;
    int streak = 0;
    
    for (int i = 0; i < clean_cnt - 1; i++) {
        char state1 = clean_pulses[i].state;
        int d1 = clean_pulses[i].duration;
        char state2 = clean_pulses[i+1].state;
        int d2 = clean_pulses[i+1].duration;
        
        if (state1 == 'H' && state2 == 'L') {
            // 设定容差范围：H 在 70~110 之间，L 在 130~175 之间
            if (d1 >= 70 && d1 <= 110 && d2 >= 130 && d2 <= 175) {
                streak++;
            } else {
                // 匹配中断！检查是不是遇到了我们要找的同步停顿 (> 200)
                if (streak >= 4 && d2 > 200) {
                    sync_idx = i + 1;
                    break;
                }
                // 否则重新计数
                streak = 0;
            }
        }
    }

    if (sync_idx != -1 && sync_idx + 1 < clean_cnt) {
        printf(" 🎯 特征匹配成功！在连续 %d 次前导握手后，锁定同步间隙: L%d\n", streak, clean_pulses[sync_idx].duration);
        
        char bits[2048];
        int bit_idx = 0;
        
        // 解析 PWM 比特流
        for (int i = sync_idx + 1; i < clean_cnt; i++) {
            if (clean_pulses[i].state == 'H') {
                if (clean_pulses[i].duration > 75) {
                    bits[bit_idx++] = '1';
                } else if (clean_pulses[i].duration > 30) {
                    bits[bit_idx++] = '0';
                }
            }
        }
        bits[bit_idx] = '\0';
        
        if (bit_idx > 0) {
            printf(" 💾 有效 Payload 长度 : %d Bits\n", bit_idx);
            printf(" 🔢 二进制流 : ");
            for(int i=0; i<bit_idx; i++) {
                putchar(bits[i]);
                if((i+1)%8 == 0) putchar(' ');
            }
            printf("\n");
            print_hex(bits, bit_idx);
        }
    } else {
        printf("\n ⚠️ 警告: 自动解码失败！未能从上方元数据中匹配到标准前导码或同步间隙。\n");
    }
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");

    free(clean_pulses);
    free(mag);
    free(binary);
    free(pulses);
}

int main(void) {
    signal(SIGINT, sigint_handler);

    printf("📡 正在初始化 PlutoSDR (本地 AXI 总线模式)...\n");
    
    struct iio_context *ctx = iio_create_local_context();
    if (!ctx) {
        fprintf(stderr, "无法创建本地 IIO 上下文！请确保程序在 PlutoSDR 内部运行。\n");
        return 1;
    }

    struct iio_device *phy = iio_context_find_device(ctx, "ad9361-phy");
    struct iio_device *dev = iio_context_find_device(ctx, "cf-ad9361-lpc");

    struct iio_channel *rx_lo = iio_device_find_channel(phy, "altvoltage0", true);
    iio_channel_attr_write_longlong(rx_lo, "frequency", CENTER_FREQ);

    struct iio_channel *rx0_i = iio_device_find_channel(dev, "voltage0", false);
    struct iio_channel *rx0_q = iio_device_find_channel(dev, "voltage1", false);
    iio_channel_enable(rx0_i);
    iio_channel_enable(rx0_q);

    iio_channel_attr_write_longlong(iio_device_find_channel(phy, "voltage0", false), "sampling_frequency", SAMPLE_RATE);
    iio_channel_attr_write_longlong(iio_device_find_channel(phy, "voltage0", false), "rf_bandwidth", SAMPLE_RATE);
    iio_channel_attr_write(iio_device_find_channel(phy, "voltage0", false), "gain_control_mode", "manual");
    iio_channel_attr_write_longlong(iio_device_find_channel(phy, "voltage0", false), "hardwaregain", 30);

    struct iio_buffer *rxbuf = iio_device_create_buffer(dev, CHUNK_SIZE, false);
    if (!rxbuf) {
        perror("创建 RX Buffer 失败");
        return 1;
    }

    printf("⏳ 正在校准环境底噪 (DC Offset)...\n");
    double i_sum = 0, q_sum = 0;
    int calib_samples = CHUNK_SIZE * 3;
    
    for (int k = 0; k < 3; k++) {
        iio_buffer_refill(rxbuf);
        void *p_dat = iio_buffer_first(rxbuf, rx0_i);
        void *p_end = iio_buffer_end(rxbuf);
        ptrdiff_t p_inc = iio_buffer_step(rxbuf);
        for (; p_dat < p_end; p_dat += p_inc) {
            i_sum += ((int16_t*)p_dat)[0];
            q_sum += ((int16_t*)p_dat)[1];
        }
    }
    double i_leak = i_sum / calib_samples;
    double q_leak = q_sum / calib_samples;

    double max_noise = 0;
    iio_buffer_refill(rxbuf);
    void *p_dat = iio_buffer_first(rxbuf, rx0_i);
    void *p_end = iio_buffer_end(rxbuf);
    ptrdiff_t p_inc = iio_buffer_step(rxbuf);
    for (; p_dat < p_end; p_dat += p_inc) {
        double di = ((int16_t*)p_dat)[0] - i_leak;
        double dq = ((int16_t*)p_dat)[1] - q_leak;
        double m = sqrt(di*di + dq*dq);
        if (m > max_noise) max_noise = m;
    }
    double trigger_level = max_noise * 3.5;

    printf("✅ 校准完成! 泄漏向量 I:%.1f Q:%.1f | 触发门限: %.1f\n", i_leak, q_leak, trigger_level);
    printf(">>> 🚀 触发式快照雷达已启动！请随时按下车钥匙... (按 Ctrl+C 退出) <<<\n");

    int total_snap_samples = CHUNK_SIZE * SNAPSHOT_CHUNKS;
    int16_t *snap_i = (int16_t *)malloc(total_snap_samples * sizeof(int16_t));
    int16_t *snap_q = (int16_t *)malloc(total_snap_samples * sizeof(int16_t));

    while (!stop) {
        iio_buffer_refill(rxbuf);
        
        int triggered = 0;
        int check_cnt = 0;
        p_dat = iio_buffer_first(rxbuf, rx0_i);
        for (; p_dat < p_end && check_cnt < 5000; p_dat += p_inc, check_cnt++) {
            double di = ((int16_t*)p_dat)[0] - i_leak;
            double dq = ((int16_t*)p_dat)[1] - q_leak;
            if (sqrt(di*di + dq*dq) > trigger_level) {
                triggered = 1;
                break;
            }
        }

        if (triggered) {
            printf("\n⚡ 检测到射频爆发！正在锁定快门...\n");
            int offset = 0;
            
            p_dat = iio_buffer_first(rxbuf, rx0_i);
            for (; p_dat < p_end; p_dat += p_inc) {
                snap_i[offset] = ((int16_t*)p_dat)[0];
                snap_q[offset] = ((int16_t*)p_dat)[1];
                offset++;
            }

            for (int chunk = 1; chunk < SNAPSHOT_CHUNKS; chunk++) {
                iio_buffer_refill(rxbuf);
                p_dat = iio_buffer_first(rxbuf, rx0_i);
                for (; p_dat < p_end; p_dat += p_inc) {
                    snap_i[offset] = ((int16_t*)p_dat)[0];
                    snap_q[offset] = ((int16_t*)p_dat)[1];
                    offset++;
                }
            }

            analyze_packet(snap_i, snap_q, total_snap_samples, i_leak, q_leak);
            
            // 清理堆积在底层的硬件缓存，防止连续触发
            for (int k=0; k<3; k++) iio_buffer_refill(rxbuf);
            printf(">>> 继续监听... <<<\n");
        }
    }

    free(snap_i);
    free(snap_q);
    iio_buffer_destroy(rxbuf);
    iio_channel_disable(rx0_i);
    iio_channel_disable(rx0_q);
    iio_context_destroy(ctx);
    printf("已安全退出。\n");
    return 0;
}
