import numpy as np
import random


def get_priot_list(badness, size=100):
    badness_index = np.argsort(badness)
    badness = np.sort(badness)[::-1]   
    
    product = np.product(badness)

    A = product/badness
    maxx = np.max(A)
    A_new = A/maxx
        
    semi_final = []
    for i in range(size):
        r = random.uniform(0.0, 1.0)
    
        for index in range(len(A_new)):
            if(r < A_new[index]):
                semi_final.append(index)
                break
    
    final = badness_index[semi_final]
    
    return final

badness = np.array([10, 5, 45, 23, 7, 8, 3, 1, 2, 5])       

final =  get_priot_list(badness)

print('10', final.tolist().count(0))
print('5', final.tolist().count(1))
print('45', final.tolist().count(2))
print('23', final.tolist().count(3))
print('7', final.tolist().count(4))
print('8', final.tolist().count(5))
print('3', final.tolist().count(6))
print('1', final.tolist().count(7))
print('2', final.tolist().count(8))
print('5', final.tolist().count(9))




        