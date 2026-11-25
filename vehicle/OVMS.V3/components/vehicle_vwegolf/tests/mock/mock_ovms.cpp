#include "mock_ovms.hpp"

MetricStore         g_metrics;
uint32_t            g_tick_count = 0;
std::function<void(TickType_t)> g_delay_hook;
StandardMetricsType StandardMetrics;
OvmsMetrics         MyMetrics;
OvmsConfig          MyConfig;
OvmsCommandApp      MyCommandApp;
OvmsVehicleFactory  MyVehicleFactory;
OvmsNotify          MyNotify;
