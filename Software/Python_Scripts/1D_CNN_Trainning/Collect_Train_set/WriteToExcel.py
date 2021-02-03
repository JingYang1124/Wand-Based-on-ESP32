# -*- coding: utf-8 -*-
"""
Created on Tue Dec 15 17:26:47 2020

@author: yangj
"""

import win32com.client    
import os
class easyExcel:    
      """A utility to make it easier to get at Excel.    Remembering  
      to save the data is your problem, as is    error handling.  
      Operates on one workbook at a time."""    
      def __init__(self, filename=None):  #打开文件或者新建文件（如果不存在的话）  
          self.xlApp = win32com.client.Dispatch('Excel.Application')    
          if filename:    
              self.filename = filename    
              self.xlBook = self.xlApp.Workbooks.Open(filename)    
          else:    
              self.xlBook = self.xlApp.Workbooks.Add()    
              self.filename = ''  
        
      def save(self, newfilename=None):  #保存文件  
          if newfilename:    
              self.filename = newfilename    
              self.xlBook.SaveAs(newfilename)    
          else:    
              self.xlBook.Save()        
      def close(self):  #关闭文件  
          self.xlBook.Close(SaveChanges=0)    
          del self.xlApp    
      def getCell(self, sheet, row, col):  #获取单元格的数据  
          "Get value of one cell"    
          sht = self.xlBook.Worksheets(sheet)    
          return sht.Cells(row, col).Value    
      def setCell(self, sheet, row, col, value):  #设置单元格的数据  
          "set value of one cell"    
          sht = self.xlBook.Worksheets(sheet)    
          sht.Cells(row, col).Value = value         
      def deleteRow(self, sheet, row):  
          sht = self.xlBook.Worksheets(sheet)  
          sht.Rows(row).Delete()#删除行  
          sht.Columns(row).Delete()#删除列
      def getRange(self, sheet, row1, col1, row2, col2):  #获得一块区域的数据，返回为一个二维元组  
          "return a 2d array (i.e. tuple of tuples)"    
          sht = self.xlBook.Worksheets(sheet)  
          return sht.Range(sht.Cells(row1, col1), sht.Cells(row2, col2)).Value    
      def addPicture(self, sheet, pictureName, Left, Top, Width, Height):  #插入图片  
          "Insert a picture in sheet"    
          sht = self.xlBook.Worksheets(sheet)    
          sht.Shapes.AddPicture(pictureName, 1, 1, Left, Top, Width, Height)        
      def cpSheet(self, before):  #复制工作表  
          "copy sheet"    
          shts = self.xlBook.Worksheets    
          shts(1).Copy(None,shts(1))
      def inserRow(self,sheet,row):
          sht = self.xlBook.Worksheets(sheet)
          sht.Rows(row).Insert(1)
      def getCellColor(self, sheet, row, col):
          sht = self.xlBook.Worksheets(sheet)
          return(sht.Cells(row, col).Interior.ColorIndex)

def  WriteDataToTrainSet(spell, dataX,dataY,dataZ):
    cwd = os.getcwd()
    Location = cwd
    FileName = "\Train.xlsx"
    xls = easyExcel(Location + FileName)    
    InitialRow = 2
    InitialCol = 3
    while(xls.getCell('sheet1',InitialRow,2)):
        InitialRow += 1
    xls.setCell('sheet1',InitialRow,2,spell)
    print("This is the " + str(InitialRow) + "th Row")
    for index in range (0,len(dataX)):
        xls.setCell('sheet1', InitialRow, index + InitialCol, dataX[index])
    for index in range (0 + len(dataX),len(dataX) + len(dataY)):
        xls.setCell('sheet1', InitialRow, index + InitialCol, dataY[index - len(dataX)])
    for index in range (0 + len(dataX) + len(dataY),len(dataX) + len(dataY) + len(dataZ)):
        xls.setCell('sheet1', InitialRow, index + InitialCol, dataZ[index - len(dataX) - len(dataY)])
    xls.save()    
    xls.close()
    

"""
This is the example
"""
"""
SpellName = "Avada"
 x = [0.0, 0.0, 0.0, -0.0, -0.001, -0.002, -0.003, -0.004, -0.006, -0.007, -0.008, -0.011, -0.014, -0.018, -0.021, -0.022, -0.025, -0.029, -0.033, -0.036, -0.039, -0.043, -0.047, -0.052, -0.058, -0.066, -0.074, -0.083, -0.092, -0.1, -0.108, -0.115, -0.123, -0.13, -0.138, -0.146, -0.153, -0.161, -0.17, -0.18, -0.19, -0.198, -0.202, -0.204, -0.205, -0.208, -0.21, -0.211, -0.212, -0.213, -0.214, -0.214, -0.215, -0.216, -0.216, -0.214, -0.21, -0.206, -0.199, -0.191, -0.183, -0.175, -0.167, -0.16, -0.153, -0.145, -0.135, -0.128, -0.12, -0.107, -0.091, -0.078, -0.072, -0.072, -0.077, -0.082, -0.092, -0.107, -0.12, -0.129, -0.136, -0.143, -0.15, -0.154, -0.155, -0.155, -0.154, -0.152, -0.152, -0.153, -0.156, -0.16, -0.164, -0.168, -0.172, -0.175, -0.179, -0.184, -0.186, -0.185]
 WriteDataToTrainSet(SpellName,x)
"""
      

