import optuna
import os
import sys
import plotly
from regulator_optuning import write_regulator_params_to_file


def visualise(study):
    fig = optuna.visualization.plot_slice(study)
    fig.update_yaxes(type="log")
    fig.show()


def load_best(study: optuna.study.Study):
    best_params = study.best_params
    q_matrix = [best_params[f"Q{i}"] for i in range(12)]
    r_matrix = [best_params[f"R{i}"] for i in range(6)]
    write_regulator_params_to_file(q_matrix, r_matrix)


ARG_TO_FUNCT_MAPPER = {
    "-v": visualise,
    "-l": load_best
}


if __name__ == "__main__":
    study = optuna.load_study("Regulator Optuning Phase 2", storage="sqlite:///example.db")
    for arg in sys.argv:
        if arg in ARG_TO_FUNCT_MAPPER:
            ARG_TO_FUNCT_MAPPER[arg](study)
