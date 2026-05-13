#ifndef RAND_H
#define RAND_H

#include <math.h>
#include <stdlib.h>

inline double RandDouble()
{
    double r = static_cast<double>(rand());
    return r / RAND_MAX;
}

inline double RandNormal()
{
    double x1, x2, w;
    do{
        x1 = 2.0 * RandDouble() - 1.0;
        x2 = 2.0 * RandDouble() - 1.0;
        w = x1 * x1 + x2 * x2;
    }while( w >= 1.0 || w == 0.0);

    w = sqrt((-2.0 * log(w))/w);
    return x1 * w;
}

/*
这是 Box-Muller 变换的极坐标形式，数学原理：

如果有两个独立均匀分布的点 (x1, x2) 在单位圆内，则：

Z0 = x1 * sqrt(-2 * ln(w) / w) 服从 N(0,1)

Z1 = x2 * sqrt(-2 * ln(w) / w) 也服从 N(0,1)（相互独立）

这里只返回了 Z0，丢弃了 Z1（也可以保存起来下次用）。
 */

#endif // random.h