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
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jul 13 2021
@author: Guillermo
'''
class Nav_dbgSM(Behavior):
	'''
	Navigation state debugger
	'''


	def __init__(self):
		super(Nav_dbgSM, self).__init__()
		self.name = 'Nav_dbg'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:710 y:133, x:697 y:376
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.nav_current_step_in = 0
		_state_machine.userdata.nav_current_step_out = 0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:114 y:209
			OperatableStateMachine.add('Inicio',
										InitActionState(),
										transitions={'ready': 'Navigation', 'command_error': 'failed'},
										autonomy={'ready': Autonomy.Off, 'command_error': Autonomy.Off})

			# x:385 y:117
			OperatableStateMachine.add('Navigation',
										NavigationActionState(positional_error=0.1),
										transitions={'target_detected': 'finished', 'next_step': 'Navigation', 'command_error': 'failed'},
										autonomy={'target_detected': Autonomy.Off, 'next_step': Autonomy.Off, 'command_error': Autonomy.Off},
										remapping={'nav_current_step_in': 'nav_current_step_in', 'nav_current_step_out': 'nav_current_step_out'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
