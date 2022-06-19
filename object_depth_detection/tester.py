import pickle

with open ('object_list.txt', 'rb') as fp:
    itemlist = pickle.load(fp)

print(itemlist)