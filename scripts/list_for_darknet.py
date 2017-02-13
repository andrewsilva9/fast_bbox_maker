import os


text_out = open('/home/asilva/ws/src/fast_bbox_maker/all_files.txt', 'w')
wd = os.getcwd()
for my_dir in os.listdir('../JPEGImages/'):
    print my_dir
    if os.path.isdir('../JPEGImages/'+my_dir):
        for my_file in os.listdir('../JPEGImages/'+my_dir):
            if my_file.startswith('.'):
                continue
            print my_file
            text_out.write('/home/asilva/darknet/dataset/VOCdevkit/new_data/JPEGImages/' + my_file +'\n')