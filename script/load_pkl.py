import pickle
import pdb

def load(file_path):
    with open(file_path, 'rb') as file:
        data = pickle.load(file)
    pdb.set_trace()
    print(data)

if __name__ == "__main__":
    path = input()
    load(path)