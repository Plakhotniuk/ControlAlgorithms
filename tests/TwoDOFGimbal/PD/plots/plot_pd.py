from tests.TwoDOFGimbal.plot_template import plot_error

input_filename = '../data/TwoAxisGimbalPDdynamic.txt'
controller_name = 'ПД регулятор.'
output_filename = 'TwoAxisGimbalPDerrDynamic.png'

plot_error(input_filename, 0, output_filename)
