#include <iostream>
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "CommonMini.hpp"
#include "esminiLib.hpp"
#include "RoadManager.hpp"
#include <vector>
#include <stdexcept>
#include <fstream>
#include <benchmark/benchmark.h>

#define _USE_MATH_DEFINES
#include <math.h>


static void BM_CutIn(benchmark::State& state)
{
    std::string scenario_file = "../../../resources/xosc/cut-in.xosc";

    for (auto _ : state)
    {
        printf("Benchmarking cut-in.xosc\n");
        SE_Init(scenario_file.c_str(), 0, 0, 0, 0);
        while (SE_GetQuitFlag() == 0)
        {
            SE_StepDT(0.1f);
        }
        SE_Close();
    }
}

BENCHMARK(BM_CutIn);


BENCHMARK_MAIN();
