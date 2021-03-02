import numpy as np 
import math

def array_function(n,mav_sys_id,x_sep,y_sep):

    flag = False
    mav_sys_id = mav_sys_id
    n = n
    x_sep = x_sep
    y_sep = y_sep
    prime_list= [] 
    if n != 1:
        for k in range(2,n+1):
            if n%k ==0:
                prime_list.append(k)
            if len(prime_list) == 1 and prime_list[0] == n:
                print("Its Prime")
                flag = True

    if flag == True:
        apperent_n = n-1
        n = apperent_n
        print(n)
    else:
        pass

    def Divisors(n) :
        divisor_list = []  
        i = 1
        while i <= n : 
            if (n % i==0) : 
                divisor_list.append(i) 
            i = i + 1
        return divisor_list

    divisor_list = Divisors(n) 
    print("All the factors :",divisor_list)
    
    def row_column(n):
        ilist = []
        jlist = []                     
        for i in range(1,n+1):
            for j in range(1,n+1):
                if i*j == n and i not in jlist and j not in ilist:
                    ilist.append(i)
                    jlist.append(j)
        return ilist, jlist

    ilist , jlist = row_column(n)
    mid = int((len(divisor_list)+1)/2)
    mid_number = divisor_list[mid]
    mid_number_index = divisor_list.index(mid_number)
    #print(mid,mid_number,mid_number_index,ilist,jlist)

    if mid_number in ilist:
        ilist_number_index = ilist.index(mid_number)
        index1 = ilist_number_index
    else:
        jlist_number_index = jlist.index(mid_number)
        index1 = jlist_number_index
    ilist_number = ilist[index1]
    jlist_number = jlist[index1] 

    print("ilist_number",ilist_number,"jlist_number",jlist_number)
    total_node_list = np.arange(1,n+1).reshape(ilist_number,jlist_number)
    
    if flag == True:
        A = [0 for i in range(len(total_node_list[0]))]
        A[0] = n+1
        total_node_list_1 = np.r_[total_node_list,[A]]
        total_node_list = total_node_list_1

    print("Matrix :" , total_node_list)
    col = 0
    row = 0
    if mav_sys_id in total_node_list:    
        for i in range(1,len(total_node_list)+1):
            for j in range(1,len(total_node_list[0])+1):
                if total_node_list[i-1][j-1] == mav_sys_id:
                    print("Found mav_sys_id :", total_node_list[i-1][j-1])
                    col = j
                    row = i
                    break        
    print("Row :",row,"Col :",col)
    print("Local_frame x :",(col-1)*10 ,"y :", -(row-1)*10)


    print((col-1)*x_sep , -(row-1)*y_sep)

    return (col-1)*x_sep , -(row-1)*y_sep

def nearest_square_method(n,mav_sys_id,x_sep,y_sep):

    n = n 
    mav_sys_id = mav_sys_id
    x_sep = x_sep
    y_sep = y_sep
    for i in range(1,n+1):
        if i*i >= n:
            val = i 
            break

    total_array = np.arange(1,(val*val)+1).reshape(val,val)
    
    for i in range(len(total_array)):
        for j in range(len(total_array[0])):
            if total_array[i][j] > n:
                total_array[i][j] = 0   

    print(total_array)
    
    for i in range(1,len(total_array)+1):
        for j in range(1,len(total_array[0])+1):
            if total_array[i-1][j-1] == mav_sys_id:
                print("Found mav_sys_id :", total_array[i-1][j-1])
                col = j
                row = i
                break

    print("Row :", row, "Col :", col)
    print("Local_frame_x_coordinate :",(col-1)*x_sep ,"Local_frame_y_coordinate :", -(row-1)*y_sep)

    
        
    return (col-1)*x_sep , -(row-1)*y_sep

if __name__ == "__main__":
    #array_function(18,12)
    nearest_square_method(26,5,10,10)