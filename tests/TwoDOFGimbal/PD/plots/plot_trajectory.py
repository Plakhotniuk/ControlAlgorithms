from tests.TwoDOFGimbal.plot_template import plot_trajectories


input_filename = '../data/TwoAxisGimbalPD.txt'
controller_name = 'ПД регулятор.'
output_filename = 'TwoAxisGimbalPDtrajstep.png'

plot_trajectories(input_filename, controller_name, output_filename)