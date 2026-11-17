############################################################################
#
#   Copyright (c) 2026 RM2027 Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

# 软件 I²C（GPIO bit-bang）主机。
#
# 为什么放在 lib 而不是某个驱动目录：
#   rm2027/fmu-h743 没有把任何可做硬件 I²C 的引脚引到插座上，
#   所有"外挂在接插件上的 I²C 器件"都只能靠 GPIO 造时序。
#   目前有两个使用者，将来可能更多：
#     · src/drivers/distance_sensor/vl53l0x_softi2c/
#     · src/drivers/magnetometer/qmc5883l_softi2c/
#   ⇒ 抽成公共库，避免两份代码各自演化后时序不一致。
#
# 本库**不依赖任何 PX4 头文件**（见 SoftI2C.cpp 文件头），
# 所以主机侧单元测试可以直接编译它（见 vl53l0x_softi2c/test_softi2c.cpp）。

add_library(softi2c INTERFACE)
target_sources(softi2c INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/SoftI2C.cpp)
target_include_directories(softi2c INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
