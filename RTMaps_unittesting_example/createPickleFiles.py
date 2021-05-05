import pickle

inputs = [1,2,3]
outputs = [1,2,3]

with open("input.pickle", 'wb') as handle:
    pickle.dump(inputs,handle)
with open("output.pickle", 'wb') as handle:
    pickle.dump(outputs,handle)