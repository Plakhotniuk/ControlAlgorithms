from tests.TwoDOFGimbal.plot_template import plot_trajectories


input_filename = '../data/TwoAxisGimbalCascadePIdynamic.txt'
controller_name = 'Каскадный ПИ регулятор.'
output_filename = 'TwoAxisGimbalCascadePItrajDynamic.png'

plot_trajectories(input_filename, 0, output_filename)
