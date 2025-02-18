import numpy as np
import crocoddyl
import matplotlib.pyplot as plt
import seaborn as sns


class ReportBench:
    metrics_names = [
        "ShootingProblem::calc",
        "ShootingProblem::calcDiff",
        "SolverDDP::Qu",
        "SolverDDP::Quu",
        "SolverDDP::Quu_inv",
        "SolverDDP::Quu_inv_Qux",
        "SolverDDP::Qx",
        "SolverDDP::Qxu",
        "SolverDDP::Qxx",
        "SolverDDP::Vx",
        "SolverDDP::Vxx",
        "SolverDDP::backwardPass",
        "SolverDDP::calcDiff",
        "SolverDDP::computeDirection",
        "SolverDDP::computeGains",
        "SolverDDP::tryStep",
        "SolverFDDP::forwardPass",
        "SolverFDDP::solve",
    ]

    def __init__(self):
        self.metrics = {
            name: [
                crocoddyl.stop_watch_get_average_time(name),
                crocoddyl.stop_watch_get_min_time(name),
                crocoddyl.stop_watch_get_max_time(name),
                crocoddyl.stop_watch_get_total_time(name),
            ]
            for name in self.metrics_names
        }
        for name in self.metrics_names:
            self.metrics[name].append(
                int(np.floor(self.metrics[name][3] / self.metrics[name][0]))
            )  # Add number of calls


def plot_bench(report):
    # Generate data as x,y where x and y are the metrics index and value
    data_x = []
    data_y = []
    colors = []
    for i, name in enumerate(report.metrics_names[:-1]):
        for j in range(3):  # Take average, min and max
            data_x.append(i)
            data_y.append(report.metrics[name][j])
            colors.append([0, 0.5, 1][j])
    n = len(report.metrics_names[:-1])

    fig, ax1 = plt.subplots(figsize=(12, 6))
    # fig.canvas.manager.set_window_title("Benchmarks")
    # fig.subplots_adjust(left=0.1, right=0.9, top=0.9, bottom=0.25)
    ax1.yaxis.grid(True, linestyle="-", which="both", color="lightgrey", alpha=0.5)
    ax1.xaxis.grid(True, linestyle="-", which="major", color="lightgrey", alpha=0.8)
    scatter = ax1.scatter(data_x, data_y, c=colors, s=100)
    ax1.set_yscale("log")
    legend = ax1.legend(
        scatter.legend_elements()[0],
        ["Average", "Min", "Max"],
        title="Classes",
    )
    ax1.add_artist(legend)
    ax1.set_xticks(np.arange(n), report.metrics_names[:n], rotation=60, fontsize=8)

    # At the top of each column, add the number of calls and the total_time spent
    pos = np.arange(n)
    total_calls = [report.metrics[name][4] for name in report.metrics_names[:n]]
    percent_time = [
        100 * report.metrics[name][3] / report.metrics["SolverFDDP::solve"][3]
        for name in report.metrics_names[:n]
    ]
    upper_labels = [
        f"{calls}\n{time:.1f}%" for calls, time in zip(total_calls, percent_time)
    ]
    for tick, label in zip(range(len(pos)), ax1.get_xticklabels()):
        ax1.text(
            pos[tick],
            1.05,
            upper_labels[tick],
            transform=ax1.get_xaxis_transform(),
            horizontalalignment="center",
            size="small",
            weight="bold",
        )

    plt.tight_layout()
    plt.show()
