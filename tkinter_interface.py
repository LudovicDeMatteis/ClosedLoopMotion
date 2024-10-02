import tkinter as tk
import numpy as np

def create_button_grid(button_names, button_width=10, button_height=2, font_size=12):
    def on_button_click(button_id):
        nonlocal selected_button_id
        selected_button_id = button_id
        root.destroy()

    # Create the main window
    root = tk.Tk()
    root.title("Closed Loop Motion Selection")

    # Create a frame to hold the buttons
    frame = tk.Frame(root)
    frame.pack(padx=10, pady=10)

    # Add a label above the buttons
    label = tk.Label(frame, text="What motion would you like to play?", font=("Arial", font_size))
    label.grid(row=0, column=0, columnspan=2, pady=(0, 10))

    # Create buttons and add them to the frame in a 2x2 grid
    buttons = [
        tk.Button(frame, text=name, command=lambda i=i: on_button_click(i), width=button_width, height=button_height, font=("Arial", font_size))
        for i, name in enumerate(button_names)
    ]

    # Arrange buttons in a 2x2 grid
    for i, button in enumerate(buttons):
        row = i // 2 + 1
        col = i % 2
        button.grid(row=row, column=col, padx=5, pady=5)

    # Initialize the selected button id
    selected_button_id = None

    # Start the Tkinter event loop
    root.mainloop()

    return selected_button_id

def ask_for_parameters(param_names, param_ranges, default_values):
    parameter_choices = []
    
    def submit_parameters():
        for i, entry in enumerate(entries):
            value = entry.get()
            if value == "":
                value = default_values[i]
            parameter_choices.append(value)
        param_window.destroy()

    param_window = tk.Tk()
    param_window.title("Enter Parameters")

    entries = []
    for i, (name, range_, default) in enumerate(zip(param_names, param_ranges, default_values)):
        tk.Label(param_window, text=f"{name} (Recommended: {range_}):").grid(row=i, column=0, padx=10, pady=5)
        entry = tk.Entry(param_window)
        entry.insert(0, default)  # Show the default value in the input field
        entry.grid(row=i, column=1, padx=10, pady=5)
        entries.append(entry)

    submit_button = tk.Button(param_window, text="Submit", command=submit_parameters)
    submit_button.grid(row=len(param_names), column=0, columnspan=2, pady=10)

    param_window.mainloop()

    parameter_choices = [float(p) for p in parameter_choices]
    
    return parameter_choices

def result_interface(robot, ddp, sol, params):
    from motions.utils import plot_solution, create_viewer
    viz = create_viewer(robot, open=True)
    qs = np.array(ddp.xs)[:, :robot.model.nq]

    def on_plots_button_click():
        plot_solution(robot, ddp, sol, params)
    
    def on_replay_button_click():
        viz.play(qs, params.DT)

    def on_quit_button_click():
        root.quit()

    # Create the main window
    root = tk.Tk()
    root.title("Results Interface")

    # Create a frame to hold the buttons
    main_frame = tk.Frame(root)
    main_frame.pack(padx=10, pady=10)

    # Create main buttons
    plot_button = tk.Button(main_frame, text="Show plots", command=on_plots_button_click, width=15, height=3, font=("Arial", 12))
    traj_button = tk.Button(main_frame, text="Replay trajectory", command=on_replay_button_click, width=15, height=3, font=("Arial", 12))

    # Create small button
    quit_button = tk.Button(main_frame, text="Quit", command=on_quit_button_click, width=10, height=2, font=("Arial", 10))

    # Arrange buttons in the frame
    plot_button.grid(row=0, column=0, padx=5, pady=5)
    traj_button.grid(row=0, column=1, padx=5, pady=5)
    quit_button.grid(row=1, column=0, columnspan=2, pady=(10, 0))

    # Start the Tkinter event loop
    root.mainloop()
