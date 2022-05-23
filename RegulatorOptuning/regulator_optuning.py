import os
import psutil
import optuna
from optuna.samplers import RandomSampler
import json
import time
import pandas as pd
import numpy as np
"""
Requires psutils and optuna

Optuned parameters:
    
"""

Q_MATRIX_PARAMS = [f"Q{i}" for i in range(12)]
R_MATRIX_PARAMS = [f"R{i}" for i in range(6)]
Q_MATRIX_PARAMS_RANGES = {
    "Q0": (-10000, 10000),
    "Q1": (-10000, 10000),
    "Q2": (-10000, 10000),
    "Q3": (-10000, 10000),
    "Q4": (-10000, 10000),
    "Q5": (-10000, 10000),
    "Q6": (-10000, 10000),
    "Q7": (-10000, 10000),
    "Q8": (-10000, 10000),
    "Q9": (-10000, 10000),
    "Q10": (-10000, 10000),
    "Q11": (-10000, 10000)
}
R_MATRIX_PARAMS_RANGES = {
    "R0": (-10000, 10000),
    "R1": (-10000, 10000),
    "R2": (-10000, 10000),
    "R3": (-10000, 10000),
    "R4": (-10000, 10000),
    "R5": (-10000, 10000),
}


def kill_thruster_regulator():
    for p in psutil.process_iter():
        if p.name() == 'ThrusterRegulator':
            p.terminate()
            p.wait()
    print("Killed thruster regulator")


def wait_for_log():
    mypath = os.getcwd()
    while True:
        print("waiting...")
        files = [f for f in os.listdir(mypath) if os.path.isfile(os.path.join(mypath, f))]
        for file in files:
            if file == "observation_log.csv":
                print("Found observation log")
                dataframe = pd.read_csv(file)
                os.remove(file)
                return dataframe
        time.sleep(1)


def write_regulator_params_to_file(q_params, r_params):
    auv_system_path = os.path.join("/", *os.getcwd().split('/')[0:-1])
    auv_config_path = os.path.join(auv_system_path, "auvConfig", "auvConfig.json")
    with open(auv_config_path, "r") as file:
        data = json.load(file)
    data["regulator"]["QMatrix"] = q_params
    data["regulator"]["RMatrix"] = r_params
    with open(auv_config_path, "w") as file:
        json.dump(data, file, indent=4)
    print("Wrote data to auv config")


def create_request():
    file = open(os.path.join(os.getcwd(), "observation_request.txt"), "w")
    file.close()


def objective(trial: optuna.Trial):
    q_params = [trial.suggest_uniform(q_matrix_param, Q_MATRIX_PARAMS_RANGES[q_matrix_param][0], Q_MATRIX_PARAMS_RANGES[q_matrix_param][1]) for q_matrix_param in Q_MATRIX_PARAMS]
    r_params = [trial.suggest_uniform(r_matrix_param, R_MATRIX_PARAMS_RANGES[r_matrix_param][0], R_MATRIX_PARAMS_RANGES[r_matrix_param][1]) for r_matrix_param in R_MATRIX_PARAMS]
    write_regulator_params_to_file(q_params, r_params)
    kill_thruster_regulator()

    time.sleep(2)
    create_request()

    observation_log = wait_for_log()[1:]

    score = 0
    if len(observation_log) >= 2:
        timestart = observation_log["timestamp"].iat[0]
        timestop = observation_log["timestamp"].iat[-1]
        timespan = timestop - timestart
        print(f"start {timestart}")
        print(f"stop {timestop}")
        print(f"timespan {timespan}")
        penalty = (1 - (timespan / 60)) * 100000
        if timespan >= 60:
            penalty = 0
        print(f"penalty {penalty}")
        score += penalty
    else:
        return 100000

    observation_log.drop("timestamp", axis=1, inplace=True)
    measured = observation_log.iloc[:, list(range(0, 6))]
    predicted = observation_log.iloc[:, list(range(6, 12))]
    difference = pd.DataFrame(measured.values - predicted.values)

    score += difference.apply(lambda x: (x ** 2).mean() ** .5).sum()

    if score < 0:
        raise ValueError("Score can't be < 0")
    return score


if __name__ == '__main__':
    study_name = "Regulator Optuning"
    storage_name = "sqlite:///example.db"
    study = optuna.create_study(study_name=study_name, storage=storage_name, load_if_exists=True) #sampler=RandomSampler()
    study.optimize(objective)
