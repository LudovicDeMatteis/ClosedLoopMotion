from tkinter_interface import create_button_grid, ask_for_parameters

if __name__ == "__main__":
    motions = ["Walk", "Jump", "Side walk", "Walk stairs"]
    motion_id = create_button_grid(motions, button_width=15, button_height=3, font_size=10)

    match motion_id:
        case 0:
            print("Walk motion selected")
            robots = ["BattoBot Closed", "BattoBot Open", "Digit Closed"]
            robot_id = create_button_grid(robots, button_width=15, button_height=3, font_size=10)
        case 1:
            print("Jump motion selected")
            robots = ["BattoBot Closed", "BattoBot Open", "Digit Closed"]
            robot_id = create_button_grid(robots, button_width=15, button_height=3, font_size=10)
        case 2:
            print("Side walk motion selected")
            robots = ["BattoBot Closed", "BattoBot Open", "Digit Closed"]
            robot_id = create_button_grid(robots, button_width=15, button_height=3, font_size=10)
        case 3:
            print("Walk stairs motion selected")
            robots = ["BattoBot Closed", "BattoBot Open", "Digit Closed"]
            robot_id = create_button_grid(robots, button_width=15, button_height=3, font_size=10)
        case _:
            print("Invalid motion selected")

    parameter_choices = {
        "BattoBot":{
            "Walk": {
                "param_names": ["Center Of Mass Target Velocity (m/s)", "Number of steps", "Single Support Duration (s)", "Double Support Duration (s)", "Center of Mass Height cost weight"],
                "param_ranges": ["0-1.5", "3-8", "0.3-0.6", "0.01-0.1", "0-10000"],
                "params_default": [0.7, 4, 0.4, 0.01, 0],
            },
            "Jump": {
                "param_names": ["Jump Duration (s)"],
                "param_ranges": ["0.3-0.5"],
                "params_default": [0.4],
            },
            "Side walk": {
                "param_names": ["Center Of Mass Target Velocity (m/s)", "Number of steps", "Single Support Duration (s)", "Double Support Duration (s)", "Center of Mass Height cost weight"],
                "param_ranges": ["0-1.0", "3-8", "0.3-0.6", "0.01-0.1", "0-10000"],
                "params_default": [0.4, 4, 0.4, 0.01, 0],
            },
            "Walk stairs": {
                "param_names": ["Center Of Mass Target Velocity (m/s)", "Number of steps", "Single Support Duration (s)", "Double Support Duration (s)", "Stairs Height"],
                "param_ranges": ["0-1.0", "3-8", "0.3-0.6", "0.05-0.2", "0-0.5"],
                "params_default": [0.3, 4, 0.5, 0.1, 0.1],
            },
        },
        "Digit":{
            "Walk": {
                "param_names": ["Center Of Mass Target Velocity (m/s)", "Number of steps", "Single Support Duration (s)", "Double Support Duration (s)", "Center of Mass Height cost weight"],
                "param_ranges": ["0-1.5", "3-8", "0.3-0.6", "0.01-0.1", "0-10000"],
                "params_default": [0.7, 4, 0.4, 0.01, 0],
            },
            "Jump": {
                "param_names": ["Jump Duration (s)"],
                "param_ranges": ["0.3-0.5"],
                "params_default": [0.4],
            },
            "Side walk": {
                "param_names": ["Center Of Mass Target Velocity (m/s)", "Number of steps", "Single Support Duration (s)", "Double Support Duration (s)", "Center of Mass Height cost weight"],
                "param_ranges": ["0-1.0", "3-8", "0.3-0.6", "0.01-0.1", "0-10000"],
                "params_default": [0.4, 4, 0.4, 0.01, 0],
            },
            "Walk stairs": {
                "param_names": ["Center Of Mass Target Velocity (m/s)", "Number of steps", "Single Support Duration (s)", "Double Support Duration (s)", "Stairs Height"],
                "param_ranges": ["0-1.0", "3-8", "0.3-0.6", "0.05-0.2", "0-0.5"],
                "params_default": [0.3, 4, 0.5, 0.1, 0.1],
            },
        },
    }

    if robots[robot_id] in ["BattoBot Closed", "BattoBot Open"]:
        robot = "BattoBot"
    elif robots[robot_id] in ["Digit Closed"]:
        robot = "Digit"
    else:
        print("Invalid robot selected")

    param_names = parameter_choices[robot][motions[motion_id]]["param_names"]
    param_ranges = parameter_choices[robot][motions[motion_id]]["param_ranges"]
    param_default = parameter_choices[robot][motions[motion_id]]["params_default"]

    params_user = ask_for_parameters(param_names, param_ranges, param_default)
    print("Parameters chosen:", params_user)
