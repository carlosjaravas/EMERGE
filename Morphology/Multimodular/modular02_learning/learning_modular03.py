import os, sys, math, pickle, time
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

from random import randint as ri
from random import uniform as ru

from keras.models import Sequential, load_model
from keras.layers import Dense
from keras.losses import Huber
from keras.optimizers import Adam

from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler, RobustScaler, MinMaxScaler
from sklearn.metrics import mean_squared_error

#Function to split the training data into X,y datasets
def load_dataset(scene_in = "modular02a", data_path = "\\training_data\\2023_14_11"):
    # Defining usefull variables
    path = os.getcwd() + data_path
    file_list = os.listdir(path)
    scene_files_list = [item for item in file_list if scene_in in item and "pkl" in item]


    file = open(path + "\\" + scene_files_list[0], "rb")
    training_data = pickle.load(file)
    training_df = pd.DataFrame(training_data)

    #List to store the name for every joint data column
    increments_columns = []
    prev_j_positions_columns = []
    post_j_positions_columns = []

    #Creates the name for every column
    num_joints = len(training_data[-1]["increments"])
    for joint_n in range(num_joints):
        #List to split inputs per joint
        joint_inc_col_name = "increments_" + str(joint_n)
        increments_columns.append(joint_inc_col_name)

        prev_joint_pos_col_name = "prev_j_positions_" + str(joint_n)
        prev_j_positions_columns.append(prev_joint_pos_col_name)

        #List to split outputs per joint
        post_joint_pos_col_name = "post_j_positions_" + str(joint_n)
        post_j_positions_columns.append(post_joint_pos_col_name)


    #Input columns per joint
    increments_df = pd.DataFrame(training_df['increments'].to_list())
    increments_df.columns = increments_columns


    prev_j_positions_df = pd.DataFrame(training_df['prev_j_positions'].to_list())
    prev_j_positions_df.columns = prev_j_positions_columns


    #Builds the X dataframe
    X_df = pd.concat([increments_df, prev_j_positions_df, 
                    training_df["prev_pos_x"], training_df["prev_pos_y"], training_df["prev_pos_z"]], 
                    axis="columns")


    #Output columns per joint
    post_j_positions_df = pd.DataFrame(training_df['post_j_positions'].to_list())
    post_j_positions_df.columns = post_j_positions_columns


    #Builds the y dataframe
    y_df = pd.concat([post_j_positions_df, 
                    training_df["post_pos_x"], training_df["post_pos_y"], training_df["post_pos_z"]], 
                    axis="columns")

    return(X_df,y_df)

def adapt_X_y(X_a,y_a):
    X_a_cols = ['current_state_x', 'current_state_y', 'current_state_z', 'joint0_current_state_rad', 'joint1_current_state_rad', 'joint2_current_state_rad'
                , 'joint0_actions_rad', 'joint1_actions_rad', 'joint2_actions_rad']

    X_a = pd.concat([X_a["prev_pos_x"], X_a["prev_pos_y"] , X_a["prev_pos_z"],
                X_a["prev_j_positions_0"], X_a["prev_j_positions_1"], X_a["prev_j_positions_2"],
                X_a["increments_0"], X_a["increments_1"], X_a["increments_2"]], 
                        axis="columns")

    X_a.columns = X_a_cols

    y_a_cols = ['future_state_x', 'future_state_y', 'future_state_z', 'joint0_future_state_rad', 'joint1_future_state_rad', 'joint2_future_state_rad']

    y_a = pd.concat([y_a["post_pos_x"], y_a["post_pos_y"] , y_a["post_pos_z"],
                y_a["post_j_positions_0"], y_a["post_j_positions_1"], y_a["post_j_positions_2"]], 
                        axis="columns")
    y_a.columns = y_a_cols
    return X_a, y_a

def plot_history(history, loss):
  hist = pd.DataFrame(history.history)
  hist['epoch'] = history.epoch

  plt.figure()
  plt.xlabel('Epoch')
  plt.ylabel(f'{loss} Loss')
  plt.plot(hist['epoch'], hist['loss'],
           label='Train Error')
  plt.plot(hist['epoch'], hist['val_loss'],
           label = 'Val Error')
  #plt.ylim([0,0.1])
  plt.legend()
  plt.show()

def plot_pred_vs_test(test, pred):
    columns = test.columns.tolist()
    max_list = []
    min_list = []
    mse_list = []
    perc_mse_list = []
    for column_num in range(len(columns)):
        plt.figure()
        plt.scatter(test.iloc[:,column_num], pred.iloc[:,column_num], s=1)

        #Getting important values
        minim = min(test.iloc[:,column_num])
        min_list.append(minim)
        maxim = max(test.iloc[:,column_num])
        max_list.append(maxim)
        mse = mean_squared_error(test.iloc[:,column_num], pred.iloc[:,column_num])
        mse_list.append(round(mse,6))
        rang = (maxim-minim)
        perc = mse/rang *100
        perc_mse_list.append(round(perc,6))
        title = columns[column_num] + " - mse: " + str(round(mse,6))
        plt.title(title)
        plt.xlabel('True Values')
        plt.ylabel('Predictions')
        # Para tener una linea recta con la cual comparar los valores y que no altere
        # los limites de la figura se grafican los valores reales con ellos mismos
        plt.plot(test.iloc[:,column_num],test.iloc[:,column_num])
        plt.grid()
    data = {"Perception": columns, "Max.": max_list, "Min.": min_list, "Mse": mse_list, "Mse perc.": perc_mse_list}
    data_df = pd.DataFrame(data)
    return(data_df)

def export2path(scene, model, scaler):
    timestr = time.strftime("_%Y_%d_%m")
    models_path = "models\\" + timestr[1:]
    if not os.path.exists(models_path):
        os.mkdir(models_path)
        print(f"Directorio '{models_path}' ha sido creado.")
    else:
        print(f"El directorio '{models_path}' ya existe.")
    model.save(models_path + "\\model_" + scene + timestr + ".keras")
    with open(models_path + "\\scaler_" + scene + timestr + '.pkl', 'wb') as file:
        pickle.dump(scaler, file)

def export(scene, model, scaler, test):
    timestr = time.strftime("_%Y_%d_%m")
    model.save("model_" + scene + timestr + ".keras")
    with open("scaler_" + scene + timestr + '.pkl', 'wb') as file:
        pickle.dump(scaler, file)
    with open("model_" + scene + timestr + '.pkl', 'wb') as file:
        pickle.dump(model, file)
    with open("testset_" + scene + timestr + '.pkl', 'wb') as file:
        pickle.dump(test, file)

def main():
    scene = "modular03" #Para modular 02 hay que añadir un _ al final para diferenciarlo del 02b al cargar los archivos
    data_path = ""
    X, y =load_dataset(scene, data_path)
    X, y = adapt_X_y(X,y)
    
    # Se separan los datos de entrenmiento (80%) y los de prueba (20%)
    X_train, X_test, y_train, y_test = train_test_split(
    X, y,
    test_size=0.2, random_state=42)

    # Scaling de los datos 
    scaler = MinMaxScaler()
    X_train_scaled = scaler.fit_transform(X_train)
    X_test_scaled = scaler.transform(X_test)
    test_set = (X_test_scaled, y_test)

    # Creacion y entrenamiento del modelo
    model = Sequential()

    # Capa de entrada con 22 unidades (correspondiente al número de entradas)
    model.add(Dense(units=9, input_dim=9, activation='relu'))

    # Capas ocultas con 64 unidades cada una
    model.add(Dense(units=27, activation='relu'))

    # Capa de salida con 14 unidades (correspondiente al número de salidas) y activación lineal
    model.add(Dense(units=6, activation='linear'))

    # Compilar el modelo con una función de pérdida adecuada para regresión
    model.compile(optimizer=Adam(learning_rate=0.001), loss=Huber(delta=1.0))

    # Supongamos que X_train y y_train son tus datos de entrenamiento
    history = model.fit(X_train_scaled, y_train, epochs=50, batch_size=20, validation_split=0.2)

    export(scene, model, scaler, test_set)




if __name__ == "__main__":
    main()