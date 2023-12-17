import tkinter as tk
from tkinter import ttk

class App:
    def __init__(self, root):
        self.root = root
        self.root.title("Node Status")
        
        # Create the first table
        self.tree1 = ttk.Treeview(root, columns=("Node", "Status"), show="headings")
        self.tree1.heading("Node", text="Node")
        self.tree1.heading("Status", text="Status")
        
        # Add some sample data to the first table
        self.tree1.insert("", "end", values=("Node 1", "True"))
        self.tree1.insert("", "end", values=("Node 2", "False"))
        self.tree1.insert("", "end", values=("Node 3", "True"))
        
        self.tree1.pack(side=tk.LEFT, expand=True, fill=tk.BOTH)
        
        # Create the second table
        self.tree2 = ttk.Treeview(root, columns=("Node", "Status"), show="headings")
        self.tree2.heading("Node", text="Node")
        self.tree2.heading("Status", text="Status")
        
        # Add some sample data to the second table
        self.tree2.insert("", "end", values=("Node 1", "True"))
        self.tree2.insert("", "end", values=("Node 2", "False"))
        self.tree2.insert("", "end", values=("Node 3", "True"))
        
        self.tree2.pack(side=tk.RIGHT, expand=True, fill=tk.BOTH)
        
        # Set up the update loop for the first table
        self.update_status(self.tree1)

        # Set up the update loop for the second table
        self.update_status(self.tree2)

    def update_status(self, tree):
        # Update the status every 1000 milliseconds (1 second) for the specified table
        self.update_tree(tree)
        self.root.after(1000, lambda: self.update_status(tree))

    def update_tree(self, tree):
        # Get the first row's ID
        first_row_id = tree.get_children()[0]
        
        # Add your logic to update the status based on the flag
        # For demonstration purposes, we'll randomly update the status
        import random
        new_status = random.choice(["True", "False"])
        
        # Update the first row's status
        tree.item(first_row_id, values=("Node 1", new_status))

        # Change the status color based on the flag
        if new_status == "True":
            tree.tag_configure("green", background="green", foreground="white")
            tree.item(first_row_id, tags=("green",))
        else:
            tree.tag_configure("red", background="red", foreground="white")
            tree.item(first_row_id, tags=("red",))

import rosservice
if __name__ == "__main__":
    print(rosservice.get_service_list())
    root = tk.Tk()
    app = App(root)
    root.mainloop()