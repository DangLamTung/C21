from django.shortcuts import render

# Create your views here.

# from django.http import JsonResponse 
   
from django.shortcuts import render 
from django.views.generic import View 
   
from rest_framework.views import APIView 
from rest_framework.response import Response 
import sqlite3



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
        conn = sqlite3.connect('db.sqlite3')
        cur = conn.cursor()
        cur.execute("SELECT username FROM auth_user")
        labels = cur.fetchall()

        cur.execute("SELECT id FROM auth_user")
        user_id_bug = cur.fetchall()

        cur.execute("SELECT user_id FROM quiz_progress")
        user_id = cur.fetchall()

        cur.execute("SELECT current_score FROM quiz_sitting")
        user_score = cur.fetchall()

        cur.execute("SELECT user_id FROM quiz_sitting")
        user_id_score = cur.fetchall()
        real_score = []
        
        for i in user_id:
            max = -1
            for (count,j) in enumerate(user_score):
                if (i == user_id_score[count]):
                    
                    if(max < j[0]):
                        max = j[0]
            real_score.append(max)
        real_labels = []
        for i in user_id:
            real_labels.append(labels[i[0] - 1])
        print(user_id_bug)
        print(labels)
        print(user_id)
        print(user_score)
        print(user_id_score)
        print(real_score)
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
                     "labels":real_labels, 
                     "chartLabel":chartLabel, 
                     "chartdata":real_score, 
             } 
        return Response(data) 
