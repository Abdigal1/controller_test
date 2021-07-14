#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_flexbe_states.Initial_state import InitActionState
from flexbe_flexbe_states.Navigation_state import NavigationActionState
from flexbe_flexbe_states.Pose_state import PositionActionState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Jul 04 2021
@author: yo
'''
class PruebaSM(Behavior):
	'''
	Prueba
	'''


	def __init__(self):
		super(PruebaSM, self).__init__()
		self.name = 'Prueba'

		# parameters of this behavior
		self.add_parameter('position_error', 0)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:828 y:575, x:908 y:233
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.B = 0
		_state_machine.userdata.A = 0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:59 y:43
			OperatableStateMachine.add('Inicio',
										InitActionState(),
										transitions={'ready': 'Nav', 'command_error': 'failed'},
										autonomy={'ready': Autonomy.Off, 'command_error': Autonomy.Off})

			# x:243 y:145
			OperatableStateMachine.add('Nav',
										NavigationActionState(positional_error=0.1),
										transitions={'target_detected': 'irrigación', 'next_step': 'Nav', 'end': 'finished', 'command_error': 'failed'},
										autonomy={'target_detected': Autonomy.Off, 'next_step': Autonomy.Off, 'end': Autonomy.Off, 'command_error': Autonomy.Off},
										remapping={'nav_current_step_in': 'A', 'nav_current_step_out': 'A'})

			# x:247 y:371
			OperatableStateMachine.add('irrigación',
										PositionActionState(positional_error=0.1, shift=0.3),
										transitions={'goal': 'Nav', 'no_goal': 'irrigación', 'out_of_range': 'failed', 'command_error': 'failed'},
										autonomy={'goal': Autonomy.Off, 'no_goal': Autonomy.Off, 'out_of_range': Autonomy.Off, 'command_error': Autonomy.Off},
										remapping={'pos_current_step_in': 'B', 'pos_current_step_out': 'B'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
