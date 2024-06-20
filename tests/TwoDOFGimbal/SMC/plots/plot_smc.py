from tests.TwoDOFGimbal.plot_template import plot_error

input_filename = '../data/TwoAxisGimbalSMC.txt'
controller_name = 'Скользяший режим управления.'
output_filename = 'TwoAxisGimbalSMCerrorstep.png'

plot_error(input_filename, controller_name, output_filename)

