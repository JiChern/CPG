import pickle


class DataSaver(object):
    def __init__(self, data_folder, exp_id):
        self.f_c1_rf = open(data_folder+'/c1_lf.txt', 'w')

        spline_names = ['c1_lf', 'c1_lm', 'c1_lr', 'c1_rf', 'c1_rm', 'c1_rr',
                        'thigh_lf', 'thigh_lm', 'thigh_lr', 'thigh_rf', 'thigh_rm', 'thigh_rr',
                        'tibia_lf', 'tibia_lm', 'tibia_lr', 'tibia_rf', 'tibia_rm', 'tibia_rr']
        
        for index, data in enumerate(spline_names):
            spline_names[index] = data + '_' + str(exp_id)

        spline_names = ['time'] + spline_names

        print(spline_names)
    
        self.f_c1_rf.write(' '.join(str(i) for i in spline_names))
        self.f_c1_rf.write('\n') 

        # self.f_c1_rf

    def dump_data(self, data):
        self.f_c1_rf.write(' '.join(str(i) for i in data))
        self.f_c1_rf.write('\n')



if __name__ == '__main__':
    data_folder = '/home/jichen/paper11_ws/src/hexapod_control/scripts/motor_data'
    # file_name = data_folder + '/c1_lf.txt'
    # print(file_name)
    ds = DataSaver(data_folder, 1)

    data = [0.01, 0.5]
    # ds.dump_data(data)