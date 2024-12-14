import matplotlib.pyplot as plt
import matplotlib
import numpy as np
matplotlib.use('qt5Agg')

class MultiColumnData:
    def __init__(self, num_columns, labels):
        labels = np.array(labels)
        
        # Validate input
        if len(labels) != num_columns:
            raise ValueError(f"Number of labels ({len(labels)}) must match number of columns ({num_columns})")
        
        self.num_columns = num_columns
        self.labels = labels
        self.data = np.empty((0, num_columns), dtype=object)
    
    def add_row(self, row_data):
        row_data = np.array(row_data)
        
        if len(row_data) != self.num_columns:
            raise ValueError(f"Number of data points ({len(row_data)}) must match number of columns ({self.num_columns})")
        
        self.data = np.vstack([self.data, row_data])
    
    def get_column(self, column_name_or_index):
        if isinstance(column_name_or_index, str):
            try:
                column_index = np.where(self.labels == column_name_or_index)[0][0]
            except IndexError:
                raise ValueError(f"Column label '{column_name_or_index}' not found")
        else:
            column_index = column_name_or_index
        
        if column_index < 0 or column_index >= self.num_columns:
            raise IndexError(f"Column index {column_index} out of range")
        
        return self.data[:, column_index]
    
    def plot_column(self, column_name_or_index, **kwargs):
        column_data = self.get_column(column_name_or_index)
        
        # Determine the label for the plot
        if isinstance(column_name_or_index, str):
            label = column_name_or_index
        else:
            label = self.labels[column_name_or_index]
        
        plt.figure(figsize=kwargs.pop('figsize', (10, 6)))
        plt.plot(column_data, label=label, **kwargs)
        plt.title(f'Plot of {label}')
        plt.xlabel('Index')
        plt.ylabel(label)
        plt.legend()
        plt.grid(True)
        
        return plt.gca()
    
    def __str__(self):
        output = "Columns: " + ", ".join(self.labels) + "\n"
        
        # Convert data to string representation
        for row in self.data:
            output += str(row.tolist()) + "\n"
        
        return output
    
    def plot_all_columns(self, **kwargs):
        plt.figure(figsize=kwargs.pop('figsize', (12, 8)))
        
        # Plot each column with a different color and label
        for i in range(self.num_columns):
            plt.plot(self.data[:, i], 
                     label=self.labels[i], 
                     **kwargs)
        
        plt.title('Plot of All Columns')
        plt.xlabel('Index')
        plt.ylabel('Values')
        plt.legend()
        plt.grid(True)
        
        return plt.gca()

# Example usage
if __name__ == "__main__":
    # Create a MultiColumnData object
    labels = ['Temperature', 'Humidity', 'Pressure']
    data_obj = MultiColumnData(3, labels)
    
    # Add some sample data
    data_obj.add_row([25.5, 30, 27])
    data_obj.add_row([26.0, 33, 38])
    data_obj.add_row([24.8, 22, 23])
    
    # Print the data
    print(data_obj)
    
    # Plot a column
    data_obj.plot_all_columns()
    plt.show()