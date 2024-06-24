from tests.TwoDOFGimbal.plot_template import plot_error

input_filename = '../data/TwoAxisGimbalCascadePIdynamic.txt'
controller_name = 'Каскадный ПИ регулятор.'
output_filename = 'TwoAxisGimbalCascadePIerrDynamic.png'

plot_error(input_filename, 0, output_filename)
