#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_flexbe_states.Pose_state import PositionActionState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Jul 12 2021
@author: Guillermo
'''
class Manip_bSM(Behavior):
	'''
	manipulator state Debugger
	'''


	def __init__(self):
		super(Manip_bSM, self).__init__()
		self.name = 'Manip_b'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:740 y:235, x:745 y:88
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.pos_current_step_out = 0
		_state_machine.userdata.pos_current_step_in = 0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:206 y:100
			OperatableStateMachine.add('Position',
										PositionActionState(positional_error=0.1, shift=0.3),
										transitions={'goal': 'finished', 'no_goal': 'Position', 'out_of_range': 'finished', 'command_error': 'failed'},
										autonomy={'goal': Autonomy.Off, 'no_goal': Autonomy.Off, 'out_of_range': Autonomy.Off, 'command_error': Autonomy.Off},
										remapping={'pos_current_step_in': 'pos_current_step_in', 'pos_current_step_out': 'pos_current_step_out'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
