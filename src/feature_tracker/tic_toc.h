#pragma once


#include <chrono>


class TicToc
{
public:
    TicToc()
    {
        tic();
    }


    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    double toc()
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start; // double后面还有一个参数，默认s
        return elapsed_seconds.count()*1000; //ms
    }


private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};