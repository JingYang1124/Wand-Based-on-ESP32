# -*- coding: utf-8 -*-
"""
Created on Tue Oct 27 10:50:48 2020

@author: yangj
"""
import pandas as pd
from sklearn.preprocessing import LabelEncoder
from sklearn.model_selection import StratifiedShuffleSplit
from keras.models import Sequential
from keras.layers import Dense

from keras.utils import np_utils



train = pd.read_csv('E:/Pro/python_proj/python_proj/WAND_Release/1D_CNN/dataset/100train.csv')#导入训练集
test = pd.read_csv('E:/Pro/python_proj/python_proj/WAND_Release/1D_CNN/dataset/100test.csv')#导入测试集


#这里是代码的精髓之处，编辑数据集中的标签，直接使用即可
def encode(train, test):
    label_encoder = LabelEncoder().fit(train.species)
    labels = label_encoder.transform(train.species)# From the species to natural number: 0, 1, 2 ...
    #print(labels): [0 0 1 0 1 0 0 1 1 0 0 ......
    classes = list(label_encoder.classes_)# Original classes that corresponds to the labels(numbers) : ['bad','good']

    train = train.drop(['species', 'id'], axis=1) #.drop() delete the columns
    test = test.drop('id', axis=1)

    return train, labels, test, classes

train, labels, test, classes = encode(train, test)

scaled_train=train.values #Only obtain the pure values in "train"
# print(scaled_train) :[[0.36971345 0.20764128 1.01101039 ... 0.86127609 0.66460005 0.56369004]
#                       [0.89455038 0.60264666 0.58669376 ... 0.61239623 0.85502414 0.70027092] ....

# SSS将每个数据集中的30%用作测试
sss = StratifiedShuffleSplit(test_size=0.2, random_state=23) #分层抽样功能，确保每个标签对应的样本的比例

for train_index, valid_index in sss.split(scaled_train, labels):
    X_train, X_valid = scaled_train[train_index], scaled_train[valid_index]
    y_train, y_valid = labels[train_index], labels[valid_index]

nb_features = 70    
nb_class = len(classes)
num_pixels = nb_features*3
num_classes = nb_class
y_train = np_utils.to_categorical(y_train, nb_class)
y_valid = np_utils.to_categorical(y_valid, nb_class)

model = Sequential()
model.add(Dense(num_pixels, input_dim=num_pixels, kernel_initializer='normal', activation='relu'))
# model.add(Dense(100, activation='tanh'))
model.add(Dense(num_classes, kernel_initializer='normal', activation='softmax'))
# Compile model
model.compile(loss='categorical_crossentropy', optimizer='adam', metrics=['accuracy'])

model.fit(X_train, y_train, validation_data=(X_valid, y_valid), epochs=20, batch_size=50, verbose=2)
model.save('Spell_model.h5')
model.summary()

