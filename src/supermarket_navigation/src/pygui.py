import tkinter as tk

# Function to read the text file and search for items
def check_item_count():
    item_name = entry.get()  # Get the user's input from the input box
    result_text.delete(1.0, tk.END)  # Clear the previous search results
    
    # Open the file and search line by line
    with open("inventory_list.txt", "r") as f:
        found = False
        items = f.readlines()
        for line in items:
            if item_name.lower() in line.lower():
                result_text.insert(tk.END, line.strip() + "\n")
                found = True
        if not found:
            result_text.insert(tk.END, f"Item not found: {item_name}\n")

# Create the main window
root = tk.Tk()
root.title("Product Search System")

# Label and input box
label = tk.Label(root, text="Enter the product name:")
label.pack()

entry = tk.Entry(root)
entry.pack()

# Search button
button = tk.Button(root, text="Search", command=check_item_count)
button.pack()

# Text box to display search results
result_text = tk.Text(root, height=10, width=40)
result_text.pack()

# Run the GUI
root.mainloop()