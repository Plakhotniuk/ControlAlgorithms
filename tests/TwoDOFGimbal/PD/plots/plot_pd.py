from tests.TwoDOFGimbal.plot_template import plot_error

input_filename = '../data/TwoAxisGimbalPD.txt'
controller_name = 'ПД регулятор.'
output_filename = 'TwoAxisGimbalPDerrstep.png'

plot_error(input_filename, controller_name, output_filename)
