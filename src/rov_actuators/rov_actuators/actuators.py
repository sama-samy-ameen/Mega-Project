import rclpy
from rclpy.node import Node
from rov_interfaces.msg import ActuatorsCommands 

class Actuators(Node):
    def __init__(self):
        super().__init__('actuators')

        #publish actuators commands
        self.publisher=self.create_publisher(ActuatorsCommands,'/actuators',10)

        #data publish rate 
        self.timer=self.create_timer(0.1,self.commands)

        #subscriber taking data from serial node
        self.serial_subscriber=self.create_subscription(ActuatorsCommands,'/joystick_actuators',self.actions,10)
        
        self.command=0
        self.speed=0


    
    def actions(self,msg:ActuatorsCommands):
        #from the joystick and accordingly controlling pistons and motors
        '''0b00000000:    0,1,2: pistons on 1 gribber of tasks 3 and 4    3 piston for another gribber 4,5: dir1 and dir2 , with const values '''
        #p1
        if msg.p1 == True :
           self.command |= (1<<0)
        else:
            self.command &= ~(1<<0)

        #p2    
        if msg.p2:
            self.command |= (1<<1)
        else:
            self.command &= ~(1 << 1)

        #p3
        if msg.p3: 
           self.command |= (1<<2)
        else :
            self.command &= ~(1<<2)

        #p4
        if msg.p4:
            self.command |=(1<<3)
        else:
            self.command &= ~(1<<3) 

        #dir1 'rotate in dir1'
        if msg.dir1:
           self.command |=(1<<4)
        else:
            self.command &= ~(1<<4)

        #dir2 'rotate in dir2'
        if msg.dir2:
            self.command |=(1<<5)
        else:
            self.command &= ~(1<<5)
            

        msg.command =self.command
        self.publisher.publish(msg)

         
    


def main(args=None):
    rclpy.init(args=args)
    node=ActuatorsCommands()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()