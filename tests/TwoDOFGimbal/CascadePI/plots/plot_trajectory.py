from tests.TwoDOFGimbal.plot_template import plot_trajectories


input_filename = '../data/TwoAxisGimbalCascadePI.txt'
controller_name = 'Каскадный ПИ регулятор.'
output_filename = 'TwoAxisGimbalCascadePItrajstep.png'

plot_trajectories(input_filename, controller_name, output_filename)
