# Ecodesafio
Data analysis repository in which an on-premise neural network is run on an Arduino to predict an electric's car remaining battery life for a race.

# Files 📁

- main_project.ipynb: File where the main project is developed (data analysis and model training)
- Battery_data.pdf: Ground truth file of the battery's behaviour
- train.csv: Piece of data unseen by the models
- test.csv: Piece of data where the model was trained
- weights.txt: Literal weights export of the Neural Networks' weights for ease of readability and use in Arduino
- eco_model.keras: Best model checkpoint
- data.csv: Complete dataset, used only for analysis and not training
- arduino_control: Folder where the main Arduino control code is stored, this is the code that ran on the car's Arduino
