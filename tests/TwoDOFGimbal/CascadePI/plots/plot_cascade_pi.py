from tests.TwoDOFGimbal.plot_template import plot_error

input_filename = '../data/TwoAxisGimbalCascadePI.txt'
controller_name = 'Каскадный ПИ регулятор.'
output_filename = 'TwoAxisGimbalCascadePIerrstep.png'

plot_error(input_filename, controller_name)
