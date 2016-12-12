import numpy as np

# take list of lists and names dictionary
# with each input list being from a single timestamp
# output is array of each signal increasing over time
def makeDictOfArraysFromLists(lists, names):
    rows = len(lists)
    row0 = lists[0]
    listsDict = {}
    for name in names.keys():
        name_index = names[name]
        try:
            float(row0[name_index])
            listsDict[name] = np.zeros(rows)
        except:
            listsDict[name] = []
    for i,row in enumerate(lists):
        for name in names.keys():
            name_index = names[name]
            try:
                listsDict[name][i] = float(row[name_index])
            except:
                listsDict[name].append(row[name_index])
    return listsDict

