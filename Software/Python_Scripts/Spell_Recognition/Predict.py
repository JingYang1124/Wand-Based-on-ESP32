# -*- coding: utf-8 -*-
"""
Created on Tue Oct 27 11:54:11 2020

@author: yangj
"""
import time
import pandas as pd
import numpy as np
from sklearn.preprocessing import LabelEncoder
from keras.models import load_model

nb_features = 70 
c = pd.read_csv('E:/Pro/PythonPro/1D_CNN/dataset/10000.csv')#导入测试集
train = pd.read_csv('E:/Pro/PythonPro/1D_CNN/dataset/100train.csv')#导入训练集

def encode(train):
    label_encoder = LabelEncoder().fit(train.species)
    classes = list(label_encoder.classes_)# Original classes that corresponds to the labels(numbers) : ['bad','good']
    return classes

classes = encode(train)

a=np.array(c)
print(a.shape)
model = load_model("Spell_model.h5")
# layerNumber = 0
# for layer in model.layers:
#     weights = layer.get_weights()
#     print(layerNumber)
#     layerNumber += 1
#     print(weights[0])
oldtime=time.process_time()

predict=model.predict_classes(a)
   
newtime=time.process_time()
print("Predict result is:")
for i in range(0,a.shape[0]):    
    print(classes[predict[i]])
print (u'相差：%s s'%(newtime-oldtime))



