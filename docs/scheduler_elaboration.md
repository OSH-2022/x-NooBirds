# 调度器(scheduler.cpp)代码详细阐述

调度部分的主要功能是作为一个中间层与小车识别、定位、分析模块以及接近物理实体的硬件控制模块相连，控制调度整个智能交通系统，实现交通系统运行的安全性、公平性、舒适性和优先权控制等智能的功能。调度模块采用集中化调度，统一协调控制智能交通系统中的所有可以被调度器控制的车辆。调度模块的输入是每一辆小车的位置和速度，输出给小车控制模块的是调度模块建议的小车的速度。

在整个项目的系统构架搭建好之后，各个功能模块是同步开发的，而且直接上实体小车进行测试较为复杂，调试信息不够充分，所以在编写调度模块的过程中，直接接上小车的定位分析模块和实体小车的控制模块进行功能测试分析是不现实而且也不高效的。为了解决调度模块开发过程中调试方面的问题，需要编写模拟模块。模拟模块需要模仿整个系统的主函数的运行方式和小车的运行状态以实现产生小车的位置和速度发送给调度模块、定时调用调度模块、接受调度器的建议修改小车的速度等功能。此外，为了从整体观察小车的状态，提高调试的效率，将小车的状态可视化是非常关键的。

综上，小车的模拟与调度部分总共有三个大的模块需要实现，即小车的调度模块、模拟模块和可视化模块。

最后，开发完成的调度模块需要整理、修改并封装到一个类中，提供接口供主程序调用执行。

调度模块的源代码在x-NooBirds/srcs/scheduler/scheduler.cpp中。

调度模块的执行效果可以参考x-NooBirds/srcs/scheduler/videos中的视频。视频的名称中，数字+形状（rect, cir, eightshape)表示有几辆小车走什么形状，初始位置可以从视频中看出来。

## 调度模块

首先，整个scheduler.cpp开发的一个前提是对外界关键信息的了解只有以下这些：
1. 每一台车辆的位置向量和速度向量
2. 每一台车辆的碰撞半径
``` cpp
// collision radius for each car
long double obj_radius[OBJ_NUM] = {200.0, 200.0, 200.0};

// real data for kinetic information for each car
long double obj_x     [OBJ_NUM];
long double obj_y     [OBJ_NUM];
long double obj_vx    [OBJ_NUM];
long double obj_vy    [OBJ_NUM];
```

也就是说，这个调度器无法获取车辆的具体轨迹，能够获取的只是在当前时刻小车的速度和位置以及碰撞半径等基础信息。能够获取轨迹信息的话对整个开发会有较大的帮助，因为有了轨迹信息，整个交通系统的自由度就少很多了，不确定的情况也就更少了。例如，通过分析轨迹信息和碰撞半径，可能发生碰撞的坐标（例如交叉点）和范围（通过碰撞半径分析）都可以确定，而且借助这些信息，小车的追尾的判定也更加容易了。此外，在危险情况下如何避险也会有较容易实现的方案，例如对于追尾的情况，如果两辆车是在同一、个轨迹上，那可以让两辆车按照相同的速度运行即可。但是，如果调度算法是按照已知车辆的轨迹的基础来设计的话，通用性、可扩展性会受到较大限制。这样设计的调度算法无法不加大量修改就应用在不同的轨迹上，比如增加一种三角形轨迹，调度算法就蒙圈了。此外，这样设计的调度算法还无法处理有障碍物或者是有不受控制的车辆闯入的情况。因此，虽然在最后实地实验的时候小车是按照固定轨迹运行的，但是这里所说的调度算法是在**未知小车的具体轨迹**的情况下工作的。

调度器实现了两个版本，一个是沙盘推演型调度器，对应于scheduler_1()；另一个是相对计算法调度器，对应于scheduler_2()。经过比较，scheduler_2()的运行效率和调度效果要较scheduler_1()更好。因此，技术细节中主要介绍一下scheduler_2()。但是scheduler_1()和scheduler_2()中有很多共通的设计方法和理念，而且scheduler_1()比scheduler_2()先写出来，因此，先简要介绍一下scheduler_1()的技术细节。

### 沙盘推演型调度器

在调度的过程中，我们要做的一件很重要的事情是利用现在以及过去的信息预测未来。具体到调度过程中，我们要根据诸如车辆的位置、速度（当前的信息）以及车辆以前是否被减速过（过去的信息）等等信息来推测未来小车之间容不容易发生碰撞。如果可能发生碰撞的话，要利用这些信息生成出一个行之有效的建议速度发送给小车控制模块，让小车发生碰撞的可能性尽量降低。这一个过程就类似于LRU算法，从现在以及过去推测未来并做出决定。

考虑到调度算法每隔几十毫秒就会被调用一次，这个调用的时间间隔非常短，因此，这里可以取一个物理上合理的近似：假设小车在当前调度器被调用到下一次调度器被调用的时间间隔内，小车的速度保持不变。此外，调度器还作了一个假设，将小车的碰撞范围抽象为圆柱体，而不是

在这个假设下，沙盘推演型调度器的主要思路是，根据当前的速度往前一步一步推演，每一步都对小车之间的距离进行计算，并判断小车之间的距离是否太近了。考虑到小车识别、数据处理、进程通信、下发指令后控制小车等等过程中的延迟以及小车位置和速度测量等的误差以及建模过程所带来的系统性误差，判断小车之间的距离是否太近了的标准应该要稍作加紧，不能仅仅只是判断两车之间的距离是否小于两车的碰撞半径之和，而是应该将后者乘上一个安全系数safety_coe再与前者进行比较，这样做可以让调度算法的调度安全性参数化，同时也能更好保障安全，实现一定的“冗余”效果。

具体来说，在初始化之后，scheduler_1()会往前推演SCHE_INTERVAL个ms。在这段时间内，一旦发现有两辆车处于不安全状态（也就是两车的距离小于两车的碰撞半径之和乘上safety_coe），那么立刻停止循环并标记这两辆车。之后尝试慢慢降低其中一辆车的速度，再检测两辆车是否仍然处于不安全状态。这个试探性过程的流程如下：首先初始化adj_coe（用来表示小车的速度降低多少倍，小于1）。然后选择一辆车减速，并探测减速之后这两辆车是否仍然处于不安全状态，如果是，那么以步长adj_step_size降低adj_coe,再次检测；如果不是，那么找到了一个解决方案。如果让这一辆选定的车减速无法解决，那么尝试让另一辆车减速以跳出不安全状态，这个过程和之前类似。

``` cpp
    int success = 0;
    for (i = 0; i < SCHE_INTERVAL; i++) {
        for (j = 0; j < OBJ_NUM; j++) {
            obj_x_tmp[j] += obj_vx_tmp[j];
            obj_y_tmp[j] += obj_vy_tmp[j];
        }
        for (j = 0; j < OBJ_NUM - 1; j++) {
            for (k = j + 1; k < OBJ_NUM; k++) {
                dist_sq = pow((obj_x_tmp[j] - obj_x_tmp[k]), 2) + pow((obj_y_tmp[j] - obj_y_tmp[k]), 2);
                if (dist_sq < pow((safety_coe * (obj_radius[j] + obj_radius[k])), 2)) {
                    long double velocity_j;
                    long double velocity_k;
                    velocity_j = pow(obj_vx_tmp[j], 2) + pow(obj_vy_tmp[j], 2);
                    velocity_k = pow(obj_vx_tmp[k], 2) + pow(obj_vy_tmp[k], 2);
                    is_safe = 0;
                    adj_coe = 1.0 - adj_step_size;
                    if (velocity_j > velocity_k) {
                        for (int l = 0; l < adj_times; l++) {
                            long double j_x = obj_x_initial[j];
                            long double j_y = obj_y_initial[j];
                            long double k_x = obj_x_initial[k];
                            long double k_y = obj_y_initial[k];
                            if (l == adj_times - 1) {
                                adj_coe = 0.0;
                            }
                            long double j_vx = adj_coe * obj_vx_initial[j];
                            long double j_vy = adj_coe * obj_vy_initial[j];
                            long double k_vx = obj_vx_initial[k];
                            long double k_vy = obj_vy_initial[k];
                            success = 1;
                            int dist_sq;
                            for (int m = 0; m < SCHE_INTERVAL; m++) {
                                dist_sq = pow((j_x - k_x), 2) + pow((j_y - k_y), 2);
                                if (dist_sq < pow(((safety_coe + 1.0) * (obj_radius[j] + obj_radius[k])), 2)) {
                                    success = 0;
                                    break;
                                }
                                j_x += j_vx;
                                j_y += j_vy;
                                k_x += k_vx;
                                k_y += k_vy;
                            }
                            if (success == 1) {
                                break;
                            }

                            adj_coe = adj_coe - adj_step_size;
                        }
                        if (success == 1) {
                            is_adjust[j] = 1;
                            adjusted_car = j;
                            break;
                        }
                    }
                    else {
                        for (int l = 0; l < adj_times; l++) {
                            long double j_x = obj_x_initial[j];
                            long double j_y = obj_y_initial[j];
                            long double k_x = obj_x_initial[k];
                            long double k_y = obj_y_initial[k];

                            long double j_vx = obj_vx_initial[j];
                            long double j_vy = obj_vy_initial[j];
                            long double k_vx = adj_coe * obj_vx_initial[k];
                            long double k_vy = adj_coe * obj_vy_initial[k];
                            success = 1;
                            int dist_sq;
                            for (int m = 0; m < SCHE_INTERVAL; m++) {
                                dist_sq = pow((j_x - k_x), 2) + pow((j_y - k_y), 2);
                                if (dist_sq < pow(((safety_coe + 1.0) * (obj_radius[j] + obj_radius[k])), 2)) {
                                    success = 0;
                                    break;
                                }
                                j_x += j_vx;
                                j_y += j_vy;
                                k_x += k_vx;
                                k_y += k_vy;
                            }
                            if (success == 1) {
                                break;
                            }
                            adj_coe = adj_coe - adj_step_size;
                        }
                        if (success == 1) {
                            is_adjust[k] = 1;
                            adjusted_car = k;
                            break;
                        }
                    }
                }
            }
            if (success == 1) {
                break;
            }
        }
        if (success == 1) {
            break;
        }
    }
```

值得一提的是，在一开始设计这个算法的时候，考虑到调度器很快就会再次调用，为了降低算法的复杂度，每次执行scheduler_1()只会检测一对处于不安全状态的车辆并对其中的一辆车减速。这个考虑有一定问题，在scheduler_2()中更改为了循环反复检测处于不安全状态的车辆，详见下一部分。

接着有一个很重要的问题，小车在减速之后怎么样恢复速度。如果小车不恢复速度的话，最后很有可能的情况就是所有小车的速度会越来越慢，直到最后大家都以非常慢的速度运行（实际上物理小车无法实现这么精细化的小车的速度），这显然不是智能交通系统的最优解，而且是一个非常差的解。因此，当已经被减速的小车较为安全时，需要将小车的速度大小恢复为原始速度的大小。实现这个的思路和前面的减速较为类似，可以使用推演法，往前一步一步推演SCHE_INTERVAL个ms，如果小车之间处于更安全状态（safety_coe要比之前检测不安全状态的时候要更大一些），那么将被减速的小车恢复为初始速度（调度器可以建议恢复成初始速度，物理小车会以一定的加速度加速回初始速度）。

为了实现速度恢复部分，需要设置标志is_adjust[]来表示某一辆小车的速度被调整过了。此外，恢复初始速度的方式也需要考虑一下。由于调度器只知道小车的初始最大速度，当小车处于轨迹上某一点的时候，想要恢复其速度还需要得到小车的速度的方向。获取速度的方向可以通过分析小车当前的速度的x, y分量来得到，但是有一种特殊情况需要考虑，就是小车的速度为0的时候（可能是因为之前该小车与另外一辆小车之间的距离过近了，导致调度器建议小车停下或者是以很小的速度运行（对应于实际物理小车其实已经停下来了）），无法通过速度的x, y分量得到角度。考虑到模拟器了解的信息较多，可以在scheduler.cpp中实现一个return_velocity_angle()函数，这个函数可以根据当前小车所在的轨迹、轨迹段和运动方向（顺时针还是逆时针）得出小车当前的速度的方向角（也就是与轨迹的切线方向平行）。通过调用这个函数，调度器就可以获取到速度角，这样调度器就可以恢复小车的x, y方向的速度了，使得小车的总速度大小等于初始速度大小。

接着需要考虑什么时候尝试去恢复小车的速度以及在什么条件下可以恢复小车的速度。为了避免小车刚被减速就被加速（这样会导致调度器根本就没有工作），scheduler_1()中采用了两种方法：
1. 如果这一次调用有小车的速度被减速了，那么这一轮就不去尝试恢复小车的速度。
2. 将safety_coe变大一些，只有正在被考虑的小车$i$（is_adjust[i]=1)恢复为初始速度后，经过推演，其与其它小车之前的距离要比碰撞半径之和的更新后的safety_coe倍要大才允许恢复其速度。

最后，核心代码如下。
``` cpp
    int is_update = 0;
    int is_still_safe = 1;
    int updated_car = 0;
    if (is_safe == 1) {
        for (int n = 0; n < OBJ_NUM; n++) {
            if (is_adjust[n] == 0) {    
                continue;
            }
            else {
                is_still_safe = 1;
                for (i = 0; i < OBJ_NUM; i++) {
                    if (i == n) {
                        obj_x_tmp[i] = obj_x[i];
                        obj_y_tmp[i] = obj_y[i];
                        long double vel_angle;
                        vel_angle = return_velocity_angle(i);
                        obj_vx_tmp[i] = initial_vel[i] * cos(vel_angle);
                        obj_vy_tmp[i] = initial_vel[i] * sin(vel_angle);
                    }
                    else {
                        obj_x_tmp[i] = obj_x[i];
                        obj_y_tmp[i] = obj_y[i];
                        obj_vx_tmp[i] = obj_vx[i];
                        obj_vy_tmp[i] = obj_vy[i];
                    }
                }
                for (i = 0; i < SCHE_INTERVAL; i++) {
                    for (j = 0; j < OBJ_NUM; j++) {
                        obj_x_tmp[j] += obj_vx_tmp[j];
                        obj_y_tmp[j] += obj_vy_tmp[j];
                    }
                    for (j = 0; j < OBJ_NUM - 1; j++) {
                        for (k = j + 1; k < OBJ_NUM; k++) {
                            dist_sq = pow((obj_x_tmp[j] - obj_x_tmp[k]), 2) + pow((obj_y_tmp[j] - obj_y_tmp[k]), 2);
                            if (dist_sq < pow(((safety_coe + 1.0) * (obj_radius[j] + obj_radius[k])), 2)) {
                                is_still_safe = 0;
                            }
                        }
                    }
                }
                if (is_still_safe == 1) {
                    is_update = 1;
                    updated_car = n;
                    is_adjust[n] = 0;
                    // std::cout << "Car " << updated_car << " is updated at time " << sim_time << "." << std::endl;
                    // print_location_verbose();
                    break;
                }
            }
        }
    }
```

和减速部分类似，考虑到调度器很快会被再次调用以及算法复杂性，一次对scheduler_1()的调用最多只会恢复一辆小车的速度，如果当前系统发现处于不安全状态（有小车的速度需要减速的话），scheduler_1()不会恢复任何一辆小车的速度。

### 相对计算法调度器

相对计算法调度器的整体架构和沙盘推演型调度器类似。相对计算法调度器的核心思想是使用相对速度和相对距离进行安全状态检测。此外，scheduler_2()还对scheduler_1()中的一些算法细节作了优化，解决了一些在可视化调试的过程中发现的问题和缺陷。

接着，我以对比的视角介绍相对计算法调度器。

首先，相对计算法调度器并没有采用推演的方式进行安全距离检测，而是采用了相对速度来判断小车是否处于不安全的状态。为了复用代码，我编写了unsafe_state_detector()函数，这个函数的作用是判断两个小车在SCHE_INTERVAL这个时间窗口内是否会处于对应于safety_coe这个安全参数的不安全状态。函数的输入是小车的位置和速度，返回值是1表明当前两辆小车处于不安全状态。
``` cpp
int unsafe_state_detector(int j, int k, long double jx, long double jy, long double jvx, long double jvy, long double kx, long double ky, long double kvx, long double kvy, long double coe) {
    long double vx_rel = jvx - kvx;
    long double vy_rel = jvy - kvy;
    long double x_rel = jx - kx;
    long double y_rel = jy - ky;
    long double x_rel_fin = (jx + jvx * SCHE_INTERVAL) - (kx + kvx * SCHE_INTERVAL);
    long double y_rel_fin = (jy + jvy * SCHE_INTERVAL) - (ky + kvy * SCHE_INTERVAL);
    long double dist_now = pow(x_rel, 2) + pow(y_rel, 2);
    long double dist_fin = pow(x_rel_fin, 2) + pow(y_rel_fin, 2);
    long double a = 0.0;
    long double b = 0.0;
    long double c = 0.0;
    long double min_dist = 0.0;
    long double minimum_dist = 0.0;
    long double dist_now_fin_min = (dist_now > dist_fin) ? dist_fin : dist_now;
    if ((fabs(vx_rel) < 1e-6)) {
        min_dist = fabs(x_rel);
        if ((y_rel > 0 && y_rel_fin < 0) || (y_rel < 0 && y_rel_fin > 0)) {
            minimum_dist = min_dist;
        }
        else {
            minimum_dist = dist_now_fin_min;
        }
    }
    else if (fabs(vy_rel) < 1e-6) {
        min_dist = fabs(y_rel);
        if ((x_rel > 0 && x_rel_fin < 0) || (x_rel < 0 && x_rel_fin > 0)) {
            minimum_dist = min_dist;
        }
        else {
            minimum_dist = dist_now_fin_min;
        }
    }
    else {
        a = -vy_rel / vx_rel;
        b = 1.0;
        c = -a * x_rel - b * y_rel;
        min_dist = pow(c, 2) / (pow(a, 2) + 1.0);
        long double y_per = (-c) / (pow(a, 2) + 1.0);
        if ((y_per > y_rel && y_per < y_rel_fin) || (y_per < y_rel && y_per > y_rel_fin)) {
            minimum_dist = min_dist;
        }
        else {
            minimum_dist = dist_now_fin_min;
        }
    }

    if (minimum_dist < pow((coe * (obj_radius[j] + obj_radius[k])), 2)) {
        return 1;
    }
    else {
        return 0;
    }
}
```

这个函数的思想是寻找两辆车之间最近的距离。首先，根据输入计算出车辆$j$相对于车辆$k$的相对位置和速度，然后以车辆$k$为原点，建立平面直角坐标系。之后计算出初始时刻车辆$j$和$k$之间的距离、经过了预测窗口时间SCHE_INTERVALms之后车辆$j$和车辆$k$之间的距离、车辆$k$（原点）到车辆$j$的轨迹的最短距离。为了计算点到直线的最短距离并判断这三个距离中哪一个是真正的两车之间的最短距离，我将车辆之间的相对运动情况进行了分类：
1. 车辆$j$相对车辆$k$的运动轨迹平行于$y$轴
2. 车辆$j$相对车辆$k$的运动轨迹平行于$x$轴
3. 除了1, 2之外的其它较为一般的情况

根据每一种类别，分别计算出垂足是否在车辆$j$相对于车辆$k$的轨迹线段上，如果在，那么车辆之间的最短距离就是车辆$k$到车辆$j$的轨迹的最短距离，否则，两车之间的最短距离就是初始时刻车辆$j$和$k$之间的距离与经过了预测窗口时间SCHE_INTERVALms之后车辆$j$和车辆$k$之间的距离中的较小值。根据两车之间的最短距离，可以较为方便地判断两车是否处于安全状态。

计算垂足是否在轨迹上的方法借用的是平面几何的一些知识。设出垂足的坐标$(x,y)$,利用两个方程$ax+by+c=0$和$bx-ax=0$(垂直条件)就可以得到$(x,y)$。接着只需要判断$x$或$y$是否处在线段的两个端点之间就行了。

采用这样的相对距离+相对速度进行计算的一个好处是算法的复杂度降低了，scheduler_1()中的SCHE_INTERVAL次（一般是$10^3$这个数量级）for循环就不必要了。这里的复杂度降低了，就为这个算法实现更多更复杂也更好的功能带来了时间上的“冗余”。

其二，scheduler_2()增加了循环调整。在沙盘推演型调度器中，每次调用scheduler_1()一次最多只会调整一辆小车的速度。在相对计算法调度器中，判断是否有小车处于不安全状态以及尝试对处于不安全状态的小车进行调整是循环执行的，一直执行到没有处于不安全状态的小车为止。为了实现这种增量式的调度，并再一次在沙盘推演型调度器的基础上进行代码优化，本调度器中舍弃了scheduler_1()中最后一起更新并输出小车的建议速度vel_adjust的方法，改为先在调度器的临时变量vx_tmp和vy_tmp中修改小车的速度。这样做符合循环的特点，避免重复劳动，提高了代码的运行效率。

其三，相对计算法调度器对速度调整进行了更细粒度的优化。在相对计算法调度器中，速度调整分为如下几步尝试：
1. 为了尽可能提高交通系统的利用率，首先调度器将尝试将其中一辆车加速。如果加速之后处于不安全状态的两辆车处于安全状态，那么问题得到解决，置标记success为1。直接转入速度恢复模块。
    ```cpp
    while (amplify_factor * vel_total_j < initial_vel[j]) {
        ret = unsafe_state_detector(j, k, x_tmp[j], y_tmp[j], amplify_factor * vx_tmp[j], amplify_factor * vy_tmp[j], x_tmp[k], y_tmp[k], vx_tmp[k], vy_tmp[k], safety_coe);
        if (ret == 0) {
            success = 1;
            adjusted_car = j;
            speed_up_times[j]++;
            is_adjust[j] = 1;
            vx_tmp[j] *= amplify_factor;
            vy_tmp[j] *= amplify_factor;
            break;
        }
        amplify_factor *= 2.0;
    }
    ```
2. 如果加速无法解决问题，那么就要尝试减速。接着调度器将对首先尝试对其中一辆车减速，如果问题解决，那么做好相应的标记之后转入速度恢复模块。如果问题没有解决，那么尝试对另一辆车进行减速，过程类似，不再赘述。
3. 如果减速仍然无法解决问题，那么现在系统处于一种临界的状态，也就是说两辆小车在调度器被调用的时候已经处于按照safety_coe的标准下不安全的状态了。为什么系统会进入临界的状态？这是因为调度器并不是时时刻刻都在运行，两次调用之间是有一定的时间间隔的，而且调度器所知道的信息是有限的、局部的，调度器并不知道小车的轨迹、小车的目的地或者小车在将来速度会发生怎么样的变化等等信息。这种情况下，为了确保整个智能交通系统的安全性，调度器采取的方式是停止其中一辆车的运动。这样做可以让两辆车在非常危险的情况下保持安全，这个做法的有效性也在模拟视频和实地测试上得到了检验。此外，还有一个实现上的细节需要注意，因为进入临界状态意味着两辆小车之间的初始距离在safety_coe的标准下已经不安全了，这样再调用unsafe_state_detector()的话返回值一定是1（表明函数判定处于不安全状态），这样调度器便无法确定让哪一辆车停下来，因此，在这里就需要放松safety_coe标准（减小safety_coe)，避免调度器无法判断让哪一辆小车停止后系统能处于安全状态。
    ``` cpp
    adj_coe = 0.001;
    long double velocity_j = pow(obj_vx[j], 2) + pow(obj_vy[j], 2);
    long double velocity_k = pow(obj_vx[k], 2) + pow(obj_vy[k], 2);
    corner_case[j][k] = 1;
    long double coe;
    coe = 1.0;
    ret = unsafe_state_detector(j, k, x_tmp[j], y_tmp[j], vx_tmp[j], vy_tmp[j], x_tmp[k], y_tmp[k], adj_coe * vx_tmp[k], adj_coe * vy_tmp[k], coe);
    if (ret == 0) {
        adjusted_car = k;
        slow_down_times[k]++;
        is_adjust[k] = 1;
        slow_car[j][k] = j;
    }
    else {
        ret = unsafe_state_detector(j, k, x_tmp[j], y_tmp[j], adj_coe * vx_tmp[j], adj_coe * vy_tmp[j], x_tmp[k], y_tmp[k], vx_tmp[k], vy_tmp[k], coe);
        if (ret == 0) {
            adjusted_car = j;
            slow_down_times[j]++;
            is_adjust[j] = 1;
            slow_car[j][k] = k;
        }
        else { 
        }
    }
    ```

这三个层次的尝试中，沙盘推演型调度器只采用了其中的第二种方式，这说明了沙盘推演型调度器的不足之处。但是，正因为有了沙盘推演型调度器，我才能更好地理解这个问题，并发现调度器存在的问题，从而开发出第二款相对计算法调度器。这三个层次的设计综合考虑了整个系统的交通效率、安全性、调度的可行性、调度的高效性等指标，与操作系统的理解运用、分析评价和依据一些评价指标的创新设计有异曲同工之处。

其四，相对计算法调度器对速度恢复模块的调用策略有一些改变，速度恢复的判定依据和恢复方法与沙盘推演型调度器相一致。在沙盘推演型调度器中，如果scheduler_1()在执行前面的代码的时候发现系统处于不安全的状态，那么其不会尝试去恢复任何一辆小车的速度。这样做有一个问题：如果有几辆小车反复处于不安全状态（原因类似小车进入临界状态），那么沙盘推演型调度器就无法恢复除了这几辆小车之外的小车的速度。这样整个交通系统的效率就会受到较大的影响，而且这也体现出了这个沙盘推演型调度器的可扩展性不够好。为了解决这个问题，需要做出一点让步和权衡，将恢复模块的执行条件调整为每调用一次scheduler_2()就执行一次，而且仅仅执行一次。这样交通系统的整体效率可以得到保证，同时也省去了反复执行速度恢复模块的开销。
``` cpp
long double vx_save;
long double vy_save;
for (l = 0; l < OBJ_NUM; l++) {
    if (is_adjust[l] == 0) {
        continue;
    }
    else {
        for (i = 0; i < OBJ_NUM; i++) {
            if (i == l) {
                // x_tmp[i] = obj_x[i];
                // y_tmp[i] = obj_y[i];
                long double vel_angle;
                vel_angle = return_velocity_angle(i);
                vx_save = vx_tmp[i];
                vy_save = vy_tmp[i];
                vx_tmp[i] = initial_vel[i] * cos(vel_angle);
                vy_tmp[i] = initial_vel[i] * sin(vel_angle);
            }
            else {
                // x_tmp[i] = obj_x[i];
                // y_tmp[i] = obj_y[i];
                // vx_tmp[i] = obj_vx[i];
                // vy_tmp[i] = obj_vy[i];
            }
        }

        ret = 0;
        for (j = 0; j < OBJ_NUM - 1; j++) {
            for (k = j + 1; k < OBJ_NUM; k++) {
                long double coe;
                if (l == slow_car[j][k] && corner_case[j][k] == 1) {
                    coe = 1.0;
                }
                else {
                    coe = safety_coe * 2.0;
                }
                ret = unsafe_state_detector(j, k, x_tmp[j], y_tmp[j], vx_tmp[j], vy_tmp[j], x_tmp[k], y_tmp[k], vx_tmp[k], vy_tmp[k], coe);
                if (ret == 1) {
                    break;
                }
            }
            if (ret == 1) {
                break;
            }
        }

        if (ret == 0) {
            // need_update = 1;
            is_adjust[l] = 0;
            updated_car = l;
            break;
        }
        else {
            vx_tmp[l] = vx_save;
            vy_tmp[l] = vy_save;
        }
    }
}
```

其五，相对计算调度器考虑了优先级、公平性和舒适性。借用操作系统中调度算法的优先级和公平性的概念，相对计算调度器实现了带有优先级和公平性的调度。首先，可以在调度器中通过priority[]数组规定每一辆小车的优先级，数值越大，小车的优先级就越高。优先级调度能够让优先级更高的车辆被优先考虑加速且尽量先不减速，保证优先级更高的车辆能够有更短的交通时间。公平性原则在两辆车优先级相同的时候起作用，这个原则会尽量先让两辆车中加速次数较少的一辆车加速且先让两辆车中减速次数较少的一辆车减速（如果可以的话）。例如，如果车辆$j$的优先级比车辆$k$的要大，那么车辆$j$将被优先考虑加速，车辆$k$将被优先考虑减速。最后，无论是相对计算调度器还是沙盘推演调度器，都考虑到了舒适性，在调度调整速度的过程中，尽量不要让速度变化幅度太大。舒适性可以通过从当前速度出发进行上下调整探测来实现。

第六点，相对计算调度器考虑了死锁的情况。真实的智能交通系统是一个复杂的整体，车辆的数量和它们的行为之间的制约关系都是复杂且多变的，此外，这里的调度器服务的对象是数量任意、轨迹任意的车辆，再叠加上调度器固有的问题和局限性，出现死锁也就在所难免了。借用操作系统中分析、处理死锁的相关思路，在这个调度器的设计过程中，我发现了两种情况的死锁并设置了相应的解决方案：
1. 局部性死锁。指两辆小车由于种种原因互相速度都变得特别小，从可视化界面上基本看不出来他们发生了运动。针对这种情况，我在相对计算调度器的最后加入了局部死锁检测和恢复。这种情况的死锁可以通过让一辆车加速，另一辆车停止的方法破除（相当于让两辆小车的距离变远）。
    ``` cpp
    // *
    // find the deadlocked state of the system and recover from the detected deadlock
    // If two cars are running at very slow speed, then the deadlock may occur. Try to speed up the car.
    // *
    long double vel_total[OBJ_NUM] = {0};
    for (i = 0; i < OBJ_NUM; i++) {
        vel_total[i] = sqrt(pow(vx_tmp[i], 2) + pow(vy_tmp[i], 2));
    }
    long double angle_j;
    long double angle_k;
    for (j = 0; j < OBJ_NUM - 1; j++) {
        for (k = j + 1; k < OBJ_NUM; k++) {
            if (fabs(vel_total[j]) < 0.01 && fabs(vel_total[k] < 0.01)) {
                angle_j = return_velocity_angle(j);
                angle_k = return_velocity_angle(k);
                ret = unsafe_state_detector(j, k, x_tmp[j], y_tmp[j], 0.0, 0.0, x_tmp[k], y_tmp[k], initial_vel[k] * cos(angle_k), initial_vel[k] * sin(angle_k), safety_coe - 1.0);
                if (ret == 0) {
                    vx_tmp[j] = 0.0;
                    vy_tmp[j] = 0.0;
                    vx_tmp[k] = initial_vel[k] * cos(angle_k);
                    vy_tmp[k] = initial_vel[k] * sin(angle_k);
                }
                else {
                    ret = unsafe_state_detector(j, k, x_tmp[j], y_tmp[j], initial_vel[j] * cos(angle_j), initial_vel[j] * sin(angle_j), x_tmp[k], y_tmp[k], 0.0, 0.0, safety_coe - 1.0);
                    if (ret == 0) {
                        vx_tmp[j] = initial_vel[j] * cos(angle_j);
                        vy_tmp[j] = initial_vel[j] * sin(angle_j);
                        vx_tmp[k] = 0.0;
                        vy_tmp[k] = 0.0;
                    }
                    else {
                    }
                }
            }
        }
    }
    ```
2. 全局性死锁。这种情况一般出现在车辆数量非常大的时候，许多车辆都发生了相互制约，导致调度器没有办法找到一个满意的速度调整方案从而死循环。这种情况下，由于调度器不知道小车的轨迹，不方便进行回退处理，所以最经济实用的方式就是报错，并让整个系统停止运行。具体的实现是，用cnt记录循环次数，如果cnt超过阈值，那么很有理由判断系统可能处于死锁，这种情况下，error=1。最后让所有输出的建议速度都为$0$。

## 模拟模块

模拟模块分成以下几个技术步骤来实现：

1. 小车的初始化。这一步骤需要设置小车的数量、每一辆小车的碰撞半径、与小车运行的轨迹相关的参数、小车的初始位置和小车的初始速度。
2. 模拟器的整体架构。小车初始化完成之后，模拟器开始模拟。模拟的采用时间片循环的方式，每一次while循环相当于模拟了1ms的时间（选择ms这个量级的原因是整个智能交通系统完成摄像头拍摄与数据处理、调用调度模块以及控制小车运行这一循环过程的延时是ms量级，选用ms量级作为模拟的时间参数可以更加贴近实际的情况），循环的时间上界是simulation_time。在1ms的模拟时间片中，调度器完成对小车的运动模拟、调用调度模块、根据调度模块的信息修正小车的速度并进行碰撞检测等功能。
3. 小车的轨迹模拟。在对小车进行运动模拟之前首先需要解决的一个问题是小车是否有预定的轨道。在与小组成员的讨论中，我们确定了小车需要完成按固定轨迹运行的技术路线，这一技术路线可以模拟智能交通系统中的宏观调度问题。因此，运动模拟中小车是有预定轨道的。为了更准确地模拟现实街区中的交通问题同时探索调度算法在更多场景下的应用，在本模拟器中，实现了正方形、圆形、八字形轨迹。在实现轨迹的过程中，为了更加贴近物理小车的转弯方面的限制，模拟器将正方形轨迹中的四个尖锐的直角替换成了四个圆，半径用corner_radius来刻画，实现了对小车转弯半径的模拟。此外，轨迹的模拟是高度参数化的，在轨迹的外形固定了之后，轨迹的中心点坐标、特征边长（比如对于正方形轨迹特征边长就是带圆角的正方形从中轴线到圆角圆心连线的距离，对八字形和圆形轨迹来说就是半径）、转弯半径等都是可以更改的。这样做能让调度器更加灵活，提供更丰富的场景模拟。一些可以定制的参数如下：
    ``` cpp
    // parameters for route rendering and simulation
    long double corner_radius = 200.0;

    long double rect_center[2] = {0.0, 0.0};
    long double rect_len = 1000.0;

    long double cir_center[2] = {0.0, 0.0};
    long double cir_radius = 1200.0;

    long double tri_center[2];
    long double tri_len;

    long double eight_center[2] = {0.0};
    long double eight_len = 1400.0;
    long double eight_radius = 1400.0;

    long double angle[OBJ_NUM];

    int direction[OBJ_NUM];
    ```
4. 小车的运动模拟。小车在每一个1ms的模拟时间片内要运动一次。为了刻画小车所在的轨迹和小车所在轨迹上的哪一段，每一辆小车都有obj_traj和obj_traj_seg属性。其中，obj_traj用来指示当前小车的轨迹是什么形状的，例如trajectory可以是枚举traj类型中的rect(正方形)、eight(八字形)等等,obj_traj_seg用来指示小车当前在obj_traj轨迹上的哪一段，例如对于正方形轨迹来说，小车可能会处于UPPER_LEFT(正方形的左上方圆角转弯处)、UPPER_EDGE(正方形上边)等等。此外，为了更好刻画并模拟涉及到转弯的路线，需要给小车一个angle信息来描述转弯的角度（这一角度采用极坐标刻画，极轴指向x轴正方向）。最后，模拟器是高度可定制的，小车的运动方向是可以通过direction规定的，当direction为CLOCKWISE(0)时，小车顺时针运动，当direction为COUNTER_CLOCKWISE(1)时，小车逆时针运动。有了小车的轨迹辅助信息，每个时间片只需要对每一个小车进行单独分析，根据其当前所在的轨迹和位置，判断这个时间片后小车的位置并修改obj_x, obj_y, (小车在二维平面上的坐标) obj_vx, obj_vy（小车的速度在二维平面的x, y分量）以及其它轨迹相关的参数（如angle, obj_traj_seg)。对于正方形和八字形，情况复杂一些，需要判断小车经过1ms的时间片后是否会转移到整个轨迹的另一段上（例如小车从八字形轨迹中的圆形转移到了直线轨道），这个只需要将1ms内小车走过的路程和小车离切换需要走过的路程进行比较即可，根据比较的结果，更新obj_traj_seg, angle等参数就能完成轨迹段的切换。接着展示一下正方形左上角圆弧段的运动模拟代码：
    ``` cpp
    for (int i = 0; i < OBJ_NUM; i++) {
        switch (obj_traj[i]) {
        case rect:
            switch (obj_traj_seg[i]) {
            case UPPER_LEFT:
                if (direction[i] == CLOCKWISE) {
                    corner_left = corner_radius * (angle[i] - pi / 2);
                    move_dist = sqrt(pow(obj_vx[i], 2) + pow(obj_vy[i], 2));
                    velocity_total = sqrt(pow(obj_vx[i], 2) + pow(obj_vy[i], 2));
                    if (move_dist > corner_left) {
                        obj_traj_seg[i] = UPPER_EDGE;
                        obj_x[i] = rect_center[0] - rect_len + move_dist - corner_left;
                        obj_y[i] = rect_center[1] + rect_len + corner_radius;
                        obj_vx[i] = velocity_total;
                        obj_vy[i] = 0.0;
                    }
                    else {
                        angle[i] = angle[i] - (move_dist) / corner_radius;
                        angle_adjust = angle[i] - pi / 2;
                        obj_x[i] = rect_center[0] - rect_len - corner_radius * sin(angle_adjust);
                        obj_y[i] = rect_center[1] + rect_len + corner_radius * cos(angle_adjust);
                        obj_vx[i] = velocity_total * cos(angle_adjust);
                        obj_vy[i] = velocity_total * sin(angle_adjust);
                    }
                }
                else {
                }
                break;
    ```
5. 调用小车的调度模块。这一部分只需要执行对应的调度器就可以了，比如scheduler_1()或者是scheduler_2()，只要调度器的接口是一致的，都可以放在这一步调用。接口统一方便开发多个版本，并对不同版本的功能进行比较和检验。根据整个项目的设计框架，调度模块需要每隔一段时间接收小车的位置和速度信息（实际实现中主函数是用类的成员函数往类中输入当前小车的位置和速度信息），然后再将建议的速度放在成员数据newData中等待主函数取用。与之相对应，模拟模块用简单易行的全局变量共享方式实现数据的输入和输出，将小车当前时刻的输入数据准备在obj_x[] 等全局数组中，然后调用调度模块。调度模块执行完成之后将建议的小车速度直接放在全局数组vel_adjust[][] 中
6. 输出调试或可视化数据并修正小车的速度。从全局数组vel_adjust[][] 中获取建议的速度，并修正小车的速度。之后进行文字调试信息(命令行中使用-v输出文字调试信息，包含小车的id、位置和速度)或可视化信息（通过print_location_concise()函数输出，之后可以交由draw.py进行可视化）。
7. 碰撞检测。调用collision_detection()函数检测是否有小车发生了碰撞，如果有碰撞，返回值为1，且向stdout输出碰撞信息，包含时间、碰撞的车辆。

最后，为了方便生成车辆的初始信息（轨迹、轨迹段、位置、角度、速度等信息），scheduler.cpp中实现了初始化方法initialize_car(),这个方法可以根据提供的信息自动往全局变量赋值，提高了代码的复用性，让初始化的过程更加简便，也就方便调试或更换模拟的场景。

## 可视化模块

可视化模块由许元元完成，我只对其做了细微的修改。模拟器中只需要每隔PRINT_INTERVALms调用输出函数，按照坐标对的形式输出小车的坐标并输出模拟器当前的时间即可。本项目中可视化的步骤如下：
1. 运行模拟器，将输出重定向到out.txt中
2. 执行
``` console
python .\draw.py
```
即可生成动图。

为了留出全屏画面的时间，在draw.py中加入了短暂的停顿。这样做方便录制展示的视频。

## 封装

为了让能调用写好的调度器，需要按照他设计好的格式封装自己的代码，实现好给定的接口。大致经过如下几个步骤封装自己的代码：
1. 定义类的名称。我认为我开发调度器的过程非常具有启发式的特点，所以我把类命名为HeuristicScheduler。
2. 将需要保留的全局变量放到类的成员中。
3. 定义并实现类的成员函数schedule()。本项目中我将scheduler_2()移植到了schedule()中。移植的过程需要注意修改首尾数据的输入和输出，其它代码不需要怎么修改。

## 总结

1. 通力合作开发效率更高。我在开发调度器的过程中，功能验证、问题发现等等都要依赖于可视化。没有可视化的帮助，调度算法不可能开发出来。
2. 模拟器是福也是祸。没有模拟器，可视化也就无从谈起，开发也就难以推进。正是因为借助模拟器验证好了调度器的功能的正确性，我们上车实地测试的时候修改了几个参数很快就让小车按照调度安全有序地运行了起来。但是，模拟器也让我走了很多弯路。由于模拟器实现的时候，小车的位置和速度并不是强关联的，这意味着小车的位置有可能是对的，但是实际上小车的速度方向等等却是错的（这是由于我在模拟器中改变小车的位置的时候使用的是速度的大小，而大小是基本上保持不变的，所以小车的方向的正确与否与小车的位置没有关系）。一旦小车的速度方向错了，那就意味着调度算法接受了错误的输入，导致最后调度器的输出非常异常，可视化的结果也非常异常。由于可视化只接收小车的位置，我很长时间都没有发现调度器中速度更新的错误（因为可视化的结果看上去非常正确，小车的位置按照预期变化），导致我浪费了很多的时间去调整自己的调度算法。所以，写代码需要深入本质，不要被表象所迷惑。
3. 高度参数化能让代码的适用性更强。高度可定制的参数能让模拟器模拟更多的情况，更好地检验调度器。此外，高度可定制的参数可以让调度器更加灵活，可扩展性更好，适用性更强。比如，由于实地测试中存在多方面的延迟和误差，调度器的safety_coe就应该调大一点，这样可以对冲延迟和误差带来的影响，让实地测试的表现符合预期。


