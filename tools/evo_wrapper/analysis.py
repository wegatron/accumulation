import os
import re
import pandas as pd
import matplotlib.pyplot as plt


def run_evo(system_call, param_name, param_values, debug=True):
    os.system('rm log.txt')
    for pv in param_values:
        if debug:
            print(system_call+str(pv)+'.txt >> ./data/log.txt')
        os.system(system_call+str(pv)+'.txt')


def log_statics():
    info_pd = pd.DataFrame()
    ind = 0
    with open('./data/log.txt', 'rt') as f:
        for line in f:
            fields = re.split(r"\s+", line.strip())
            if len(fields) != 2:
                continue
            if fields[0] == 'max':
                max_val = float(fields[1])
            elif fields[0] == 'mean':
                mean_val = float(fields[1])
            elif fields[0] == 'median':
                median_val = float(fields[1])
            elif fields[0] == 'min':
                min_val = float(fields[1])
            elif fields[0] == 'rmse':
                rmse_val = float(fields[1])
            elif fields[0] == 'sse':
                sse_val = float(fields[1])
            elif fields[0] == 'std':
                std_val = float(fields[1])
                data = pd.Series([max_val, mean_val, median_val, min_val, rmse_val, sse_val, std_val])
                data.name = "{0:.2f}".format(0.4*ind)
                info_pd = info_pd.append(data)
                ind = ind + 1
    info_pd.columns = ['max', 'mean', 'median', 'min', 'rmse', 'sse', 'std']
    f.close()
    return info_pd


def compare_plot(alg_names, statics, xlabel):
    attrs = statics[0].columns
    attr_n = len(attrs)
    alg_n = len(alg_names)
    for attr_i in range(0, attr_n):
        attr_cp_pd = pd.DataFrame()
        for alg_i in range(0, alg_n):
            tmp_s = statics[alg_i].iloc[:, attr_i]
            tmp_s.name = alg_names[alg_i]
            # merge column
            attr_cp_pd = pd.concat([attr_cp_pd, tmp_s], axis=1)
        attr_cp_pd.to_csv('./data/'+attrs[attr_i]+'.csv')
        attr_cp_pd.plot(linewidth=0.7)
        plt.title(attrs[attr_i])
        plt.ylabel('error')
        plt.xlabel(xlabel)
        plt.grid(True)
        plt.savefig('./data/'+attrs[attr_i]+'.png', quality=100, dpi=300)
        plt.close()


if __name__ == '__main__':
    noise_sigma = []

    for i in range(0, 21):
        noise_sigma.append(i * 0.4)

    # run_evo('evo_rpe tum -as ./data/frame_traj_gt.txt ./data/F/frame_traj_est_F_', 'noise', noise_sigma)

    F_rt_statics = log_statics()

    # run_evo('evo_rpe tum -as ./data/frame_traj_gt.txt ./data/frame_traj_est_E_', 'noise', noise_sigma)

    E_rt_statics = log_statics()

    # compare
    compare_plot(['rt_by_F', 'rt_by_E'], [F_rt_statics, E_rt_statics], 'noise(pixel_sigma)')




