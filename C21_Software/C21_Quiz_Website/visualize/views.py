from django.shortcuts import render

# Create your views here.

# from django.http import JsonResponse 
   
from django.shortcuts import render 
from django.views.generic import View 
   
from rest_framework.views import APIView 
from rest_framework.response import Response 
import sqlite3

conn = sqlite3.connect('db.sqlite3')

class HomeView(View): 
    def get(self, request, *args, **kwargs): 
        return render(request, 'visualize/index.html') 
   
   
#################################################### 
   
## if you don't want to user rest_framework 
   
# def get_data(request, *args, **kwargs): 
# 
# data ={ 
#             "sales" : 100, 
#             "person": 10000, 
#     } 
# 
# return JsonResponse(data) # http response 
   
   
####################################################### 
   
## using rest_framework classes 
   
class ChartData(APIView): 
    authentication_classes = [] 
    permission_classes = [] 
    
    def get(self, request, format = None): 

        # cur = conn.cursor()
        # cur.execute("SELECT user FROM auth_user")
        # labels = cur.fetchall()
        
        # cur.execute("SELECT user FROM auth_user")
        # labels = cur.fetchall()
        labels = [ 
            'January', 
            'February',  
            'March',  
            'April',  
            'May',  
            'June',  
            'July'
            ] 
        chartLabel = "my data"
        chartdata = [0, 10, 5, 2, 20, 30, 45] 
        data ={ 
                     "labels":labels, 
                     "chartLabel":chartLabel, 
                     "chartdata":chartdata, 
             } 
        return Response(data) 
