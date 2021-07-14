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
Created on Wed Jul 14 2021
@author: Guillermo
'''
class IntegracionSM(Behavior):
	'''
	Integration state
	'''


	def __init__(self):
		super(IntegracionSM, self).__init__()
		self.name = 'Integracion'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:834 y:71, x:854 y:422
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.Step_A = 1
		_state_machine.userdata.Step_B = 1

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:63 y:354
			OperatableStateMachine.add('Inicio',
										InitActionState(),
										transitions={'ready': 'Navegacion', 'command_error': 'failed'},
										autonomy={'ready': Autonomy.Off, 'command_error': Autonomy.Off})

			# x:235 y:91
			OperatableStateMachine.add('Manipulacion',
										PositionActionState(positional_error=0.1, shift=0.3),
										transitions={'goal': 'Navegacion', 'no_goal': 'Manipulacion', 'out_of_range': 'failed', 'command_error': 'failed'},
										autonomy={'goal': Autonomy.Off, 'no_goal': Autonomy.Off, 'out_of_range': Autonomy.Off, 'command_error': Autonomy.Off},
										remapping={'pos_current_step_in': 'Step_A', 'pos_current_step_out': 'pos_current_step_out'})

			# x:246 y:288
			OperatableStateMachine.add('Navegacion',
										NavigationActionState(positional_error=0.1),
										transitions={'target_detected': 'Manipulacion', 'next_step': 'Navegacion', 'end': 'finished', 'command_error': 'failed'},
										autonomy={'target_detected': Autonomy.Off, 'next_step': Autonomy.Off, 'end': Autonomy.Off, 'command_error': Autonomy.Off},
										remapping={'nav_current_step_in': 'Step_B', 'nav_current_step_out': 'Step_A'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
