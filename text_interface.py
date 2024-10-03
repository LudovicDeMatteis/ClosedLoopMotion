import numpy as np

def create_button_grid(button_names, button_width=10, button_height=2, font_size=12):
    print("\n Select one of the following?")
    for i, name in enumerate(button_names):
        print(f"{i + 1}: {name}")

    while True:
        try:
            choice = int(input("Enter the number of your choice: ")) - 1
            if 0 <= choice < len(button_names):
                selected_button_id = choice
                break
            else:
                print("Invalid choice. Please try again.")
        except ValueError:
            print("Invalid input. Please enter a number.")

    return selected_button_id

def ask_for_parameters(param_names, param_ranges, default_values):
    parameter_choices = []

    for i, (name, range_, default) in enumerate(zip(param_names, param_ranges, default_values)):
        while True:
            user_input = input(f"Enter value for {name} (Recommended: {range_}, Default: {default}): ")
            if user_input == "":
                user_input = default
            try:
                value = float(user_input)
                parameter_choices.append(value)
                break
            except ValueError:
                print("Invalid input. Please enter a number.")
    
    return parameter_choices

def result_interface(robot, ddp, sol, params):
    from motions.utils import plot_solution, create_viewer
    viz = create_viewer(robot, open=True)
    qs = np.array(ddp.xs)[:, :robot.model.nq]

    while True:
        print("\n\n %%%%%%%%%%%% RESULTS %%%%%%%%%%%%")
        print("1: Show plots")
        print("2: Replay trajectory")
        print("3: Quit")

        try:
            choice = int(input("Enter the number of your choice: "))
            if choice == 1:
                plot_solution(robot, ddp, sol, params)
            elif choice == 2:
                viz.play(qs, params.DT)
            elif choice == 3:
                print("Exiting...")
                break
            else:
                print("Invalid choice. Please try again.")
        except ValueError:
            print("Invalid input. Please enter a number.")
