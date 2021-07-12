#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
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
		_state_machine.userdata.positional_error = 0.0
		_state_machine.userdata.pose_goal = "prueba"

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:217 y:295
			OperatableStateMachine.add('Nav',
										NavigationActionState(positional_error=0.1),
										transitions={'target_detected': 'A', 'command_error': 'failed'},
										autonomy={'target_detected': Autonomy.Off, 'command_error': Autonomy.Off},
										remapping={'pose_goal': 'pose_goal', 'position_error': 'position_error'})

			# x:479 y:495
			OperatableStateMachine.add('A',
										PositionActionState(positional_error=0.1),
										transitions={'goal': 'finished', 'no_goal': 'A', 'command_error': 'failed'},
										autonomy={'goal': Autonomy.Off, 'no_goal': Autonomy.Off, 'command_error': Autonomy.Off},
										remapping={'pose_goal': 'pose_goal', 'position_error': 'positional_error'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
