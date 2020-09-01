import os

with open('namelist.txt', 'r') as n:
    datasets = [x.strip() for x in n.readlines()]

for s in range(len(datasets)):
    for i in range(len(datasets)):
        path = './data/'+datasets[i]+'/lidar/'
        print(path)
        pcd_list = os.listdir(path+'1/')
        pcd_list.sort()
        with open(path+'pcd_list.txt', 'w') as f:
            for n in pcd_list:
                print(n)
                f.write(n+'\n')

for s in range(len(datasets)):
    for i in range(1, 7):
        with open('./data/'+datasets[s]+'/image/'+'{}.txt'.format(i), 'w') as f:
            path = './data/'+datasets[s]+'/image/'+'./{}/'.format(i)
            name_list = os.listdir(path)
            name_list.sort()
            for n in name_list:
                f.write(n+'\n')
