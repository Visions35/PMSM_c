#include <stdio.h>
#include <math.h>

#define SAMPLE_RATE 44100  // 采样率
#define CUTOFF_FREQ 2000   // 截止频率
#define PI 3.14159265359

typedef struct {
    double b0, b1, b2;
    double a1, a2;
    double x1, x2;
    double y1, y2;
} FilterState;

void initializeFilter(FilterState *filter) {
    // 计算滤波器系数
    double omega_c = 2.0 * PI * CUTOFF_FREQ / SAMPLE_RATE;
    double alpha = sin(omega_c) / (2.0 * 0.7071);
    double a0_inv = 1.0 / (1.0 + alpha);

    filter->b0 = (1.0 - cos(omega_c)) * 0.5 * a0_inv;
    filter->b1 = 1.0 - cos(omega_c) * a0_inv;
    filter->b2 = (1.0 - cos(omega_c)) * 0.5 * a0_inv;
    filter->a1 = -2.0 * cos(omega_c) * a0_inv;
    filter->a2 = (1.0 - alpha) * a0_inv;

    filter->x1 = 0.0;
    filter->x2 = 0.0;
    filter->y1 = 0.0;
    filter->y2 = 0.0;
}

double filterSample(FilterState *filter, double input) {
    double output = filter->b0 * input + filter->b1 * filter->x1 + filter->b2 * filter->x2
                  - filter->a1 * filter->y1 - filter->a2 * filter->y2;

    filter->x2 = filter->x1;
    filter->x1 = input;
    filter->y2 = filter->y1;
    filter->y1 = output;

    return output;
}

int main() {
    FilterState filter;
    initializeFilter(&filter);

    // 生成一个简单的输入信号，例如正弦波
    double frequency = 1000.0;  // 输入信号频率
    double amplitude = 0.5;     // 输入信号幅度
    double time = 0.0;
    double timeStep = 1.0 / SAMPLE_RATE;

    for (int i = 0; i < SAMPLE_RATE; ++i) {
        double input = amplitude * sin(2.0 * PI * frequency * time);
        double output = filterSample(&filter, input);
        printf("%lf\n", output);

        time += timeStep;
    }

    return 0;
}