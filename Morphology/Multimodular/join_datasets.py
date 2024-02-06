import os, pickle, time

# Joins every data set file into one for an specific set of models/scenes
def join_datasets(raw_data_path, scene, modules):
    path = raw_data_path + scene
    file_list = os.listdir(path)
    # List of training data files
    #scene_files_list = [item for item in file_list if scene in item and "pkl" in item]
    # Changed to work for an specific moment
    scene_files_list = [item for item in file_list if scene[:10] in item and "pkl" in item]
    print("---- Joining", len(scene_files_list), scene, "datasets ----")

    # Loads initial data
    file = open(path + "\\" + scene_files_list[0], "rb")
    training_data = pickle.load(file)

    # Appends all data to the initial data
    for dataset_file_name in scene_files_list[1:]:
        dataset_file = open(path + "\\" + dataset_file_name, "rb")
        dataset = pickle.load(dataset_file)
        for element in dataset:
            training_data.append(element)
            
    # Dumps the data in a new file
    timestr = time.strftime("_%Y_%d_%m_%H%M")
    new_file_path = "C:\\Users\\carlo\\OneDrive\\Imágenes\\Documentos\\GitHub\\EMERGE\\Morphology\\" + modules + "\\training\\training_data\\" + timestr[1:-5]
    if not os.path.exists(new_file_path):
        os.mkdir(new_file_path)
    new_file_name = "joined_training_dataset_" + scene + timestr + '.pkl'
    new_file = open(new_file_path + "\\" + new_file_name, 'wb')
    pickle.dump(training_data, new_file)
    new_file.close()

def main():
    # Path were all the files are stored just before scene name
    raw_data_path = "C:\\Users\\carlo\\OneDrive - Estudiantes ITCR\\U\\TFG España\\Codigo\\Datos de entrenamiento\\2024_01_23_"
    modules = "Multimodular"
    if "Multimodular" in modules:
        #scene_list = ["modular02a","modular02b","modular02c","modular03"]
        #scene_list = ["modular02b","modular02","modular03"]
        scene_list = ["modular03a","modular03b","modular03c","modular03d","modular03e"]
    elif "Quad" in modules:
        scene_list = ["modular03"]
    else:
        scene_list = ["modular03"]

    for scene in scene_list:
        join_datasets(raw_data_path, scene, modules)

if __name__ == "__main__":
    main()