############################################################################
#
#   gyrofft_bridge —— 把板载 gyro_fft 的峰值结果桥接到 debug_array
#
#   背景：本板没有 SD 卡也没有 USB，唯一的观测通道是 MAVLink 串口。
#   而 sensor_gyro_fft 没有对应的 MAVLink 流，数据出不来。
#
#   直接传原始陀螺数据行不通：1997.95 Hz × 74 B/帧 ≈ 148 KB/s，
#   而 921600 波特率实测只能到 12 KB/s，且 PX4 的 mavlink 主循环
#   （px4_usleep(_main_loop_delay)）把单流频率硬限在约 154 Hz，
#   Nyquist 只有 77 Hz —— 看不到 167 Hz 的电机基频。
#
#   所以让板子本地算 FFT（CMSIS-DSP，采样率 1997.95 Hz，Nyquist 999 Hz），
#   这里只把「峰值结论 + 当时的油门 + 姿态」打包发出去。
#
#   打包内容见 gyrofft_bridge.cpp 顶部的字段表。
#
############################################################################

px4_add_module(
	MODULE modules__gyrofft_bridge
	MAIN gyrofft_bridge
	STACK_MAIN 2048
	SRCS
		gyrofft_bridge.cpp
	DEPENDS
		px4_work_queue
	)
