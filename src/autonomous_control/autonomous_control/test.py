# import them like this:

from rov_interfaces.msg import ModelReadings, AutonomousCommand

# Then create instances:

readings = ModelReadings()
command = AutonomousCommand()

# Access their fields directly:

print(readings.obj_1.x)
print(command.movements_direction.linear.x)
print(command.gripper)

# For example, to assign values:

command.movements_direction.linear.x = 1.0
command.movements_direction.angular.z = 0.5
command.gripper = 1