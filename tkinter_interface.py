import tkinter as tk

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
    
    return parameter_choices

