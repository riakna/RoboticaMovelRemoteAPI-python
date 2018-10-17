import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt 
'''
sensor_range = np.arange(0, 100, 1)

speed_range = np.arange(0, 10, .1)

lateral_sensor_vclose = fuzz.trimf(sensor_range, [0, 0, 10])
lateral_sensor_close = fuzz.trimf(sensor_range, [9, 21, 30])
lateral_sensor_medium = fuzz.trimf(sensor_range, [26, 38, 50])
lateral_sensor_far = fuzz.trapmf(sensor_range, [46, 60, 100, 100])


lateral_sensor_vclose.view()
'''

lateral_sensor = ctrl.Antecedent(np.arange(0, 100, 1), 'lateral_sensor')
diagonal_sensor = ctrl.Antecedent(np.arange(0, 100, 1), 'diagonal_sensor')
front_sensor = ctrl.Antecedent(np.arange(0, 100, 1), 'front_sensor')

left_speed = ctrl.Consequent(np.arange(0, 10, .1), 'left_speed', defuzzify_method='centroid')
right_speed = ctrl.Consequent(np.arange(0, 10, .1), 'right_speed', defuzzify_method='centroid')

lateral_sensor['very_close'] = fuzz.trimf(lateral_sensor.universe, [0, 0, 10])
lateral_sensor['close'] = fuzz.trimf(lateral_sensor.universe, [9, 21, 30])
lateral_sensor['medium'] = fuzz.trimf(lateral_sensor.universe, [26, 38, 50])
lateral_sensor['far'] = fuzz.trapmf(lateral_sensor.universe, [46, 60, 100, 100])

diagonal_sensor['very_close'] = fuzz.trimf(diagonal_sensor.universe, [0, 0, 10])
diagonal_sensor['close'] = fuzz.trimf(diagonal_sensor.universe, [9, 21, 30])
diagonal_sensor['medium'] = fuzz.trimf(diagonal_sensor.universe, [26, 38, 50])
diagonal_sensor['far'] = fuzz.trapmf(diagonal_sensor.universe, [46, 60, 100, 100])

front_sensor['very_close'] = fuzz.trimf(front_sensor.universe, [0, 0, 10])
front_sensor['close'] = fuzz.trimf(front_sensor.universe, [9, 21, 30])
front_sensor['medium'] = fuzz.trimf(front_sensor.universe, [26, 38, 50])
front_sensor['far'] = fuzz.trapmf(front_sensor.universe, [46, 60, 100, 100])

left_speed['slow'] = fuzz.trapmf(left_speed.universe, [0, 0, 2, 3])
left_speed['medium'] = fuzz.trimf(left_speed.universe, [2, 4, 6])
left_speed['fast'] = fuzz.trapmf(left_speed.universe, [5, 8, 10, 10])

right_speed['slow'] = fuzz.trapmf(left_speed.universe, [0, 0, 2, 3])
right_speed['medium'] = fuzz.trimf(left_speed.universe, [2, 4, 6])
right_speed['fast'] = fuzz.trapmf(left_speed.universe, [5, 8, 10, 10])

lateral_sensor.view()
diagonal_sensor.view()
front_sensor.view()
left_speed.view()
right_speed.view()

rules = []
#left_speed rules
very_close
close
medium
far
rules.append(ctrl.Rule(lateral_sensor['very_close'] & diagonal_sensor['very_close'] & front_sensor['very_close'], left_speed['slow']))
rules.append(ctrl.Rule(lateral_sensor['close'] & diagonal_sensor['very_close'] & front_sensor['very_close'], right_speed['slow']))
rules.append(ctrl.Rule(lateral_sensor['medium'] & diagonal_sensor['very_close'] & front_sensor['very_close'], left_speed['slow']))
rules.append(ctrl.Rule(lateral_sensor['far'] & diagonal_sensor['very_close'] & front_sensor['very_close'], left_speed['slow']))
rules.append(ctrl.Rule(lateral_sensor['very_close'] & diagonal_sensor['very_close'] & front_sensor['very_close'], left_speed['slow']))
rules.append(ctrl.Rule(lateral_sensor['very_close'] & diagonal_sensor['very_close'] & front_sensor['very_close'], left_speed['slow']))
rules.append(ctrl.Rule(lateral_sensor['very_close'] & diagonal_sensor['very_close'] & front_sensor['very_close'], left_speed['slow']))
rules.append(ctrl.Rule(lateral_sensor['very_close'] & diagonal_sensor['very_close'] & front_sensor['very_close'], left_speed['slow']))
rules.append(ctrl.Rule(lateral_sensor['very_close'] & diagonal_sensor['very_close'] & front_sensor['very_close'], left_speed['slow']))
rules.append(ctrl.Rule(lateral_sensor['very_close'] & diagonal_sensor['very_close'] & front_sensor['very_close'], left_speed['slow']))
rules.append(ctrl.Rule(lateral_sensor['very_close'] & diagonal_sensor['very_close'] & front_sensor['very_close'], left_speed['slow']))
rules.append(ctrl.Rule(lateral_sensor['very_close'] & diagonal_sensor['very_close'] & front_sensor['very_close'], left_speed['slow']))
rules.append(ctrl.Rule(lateral_sensor['very_close'] & diagonal_sensor['very_close'] & front_sensor['very_close'], left_speed['slow']))
rules.append(ctrl.Rule(lateral_sensor['very_close'] & diagonal_sensor['very_close'] & front_sensor['very_close'], left_speed['slow']))
rules.append(ctrl.Rule(lateral_sensor['very_close'] & diagonal_sensor['very_close'] & front_sensor['very_close'], left_speed['slow']))
rules.append(ctrl.Rule(lateral_sensor['very_close'] & diagonal_sensor['very_close'] & front_sensor['very_close'], left_speed['slow']))
rules.append(ctrl.Rule(lateral_sensor['very_close'] & diagonal_sensor['very_close'] & front_sensor['very_close'], left_speed['slow']))
rules.append(ctrl.Rule(lateral_sensor['very_close'] & diagonal_sensor['very_close'] & front_sensor['very_close'], left_speed['slow']))
rules.append(ctrl.Rule(lateral_sensor['very_close'] & diagonal_sensor['very_close'] & front_sensor['very_close'], left_speed['slow']))
rules.append(ctrl.Rule(lateral_sensor['very_close'] & diagonal_sensor['very_close'] & front_sensor['very_close'], left_speed['slow']))
rules.append(ctrl.Rule(lateral_sensor['very_close'] & diagonal_sensor['very_close'] & front_sensor['very_close'], left_speed['slow']))
rules.append(ctrl.Rule(lateral_sensor['very_close'] & diagonal_sensor['very_close'] & front_sensor['very_close'], left_speed['slow']))
rules.append(ctrl.Rule(lateral_sensor['very_close'] & diagonal_sensor['very_close'] & front_sensor['very_close'], left_speed['slow']))
rules.append(ctrl.Rule(lateral_sensor['very_close'] & diagonal_sensor['very_close'] & front_sensor['very_close'], left_speed['slow']))


plt.show()
#class WFFController: