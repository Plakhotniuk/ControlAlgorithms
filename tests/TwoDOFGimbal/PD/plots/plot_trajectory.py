from tests.TwoDOFGimbal.plot_template import plot_trajectories


input_filename = '../data/TwoAxisGimbalPDdynamic.txt'
controller_name = 'ПД регулятор.'
output_filename = 'TwoAxisGimbalPDtrajDynamic.png'

plot_trajectories(input_filename, 0, output_filename)
