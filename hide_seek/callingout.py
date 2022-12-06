from rclpy.node import Node                     # Handles the creation of nodes
from std_msgs.msg import String
from gtts import gTTS                           # Text-to-speech
import playsound 
import time
import rclpy                                    # Python library for ROS 2


class CountDown(Node):
    def __init__(self):
        super().__init__('turtlebot_calling')
        self.get_logger().info("calling enabled")
    
        self.count_down_signal_sub = self.create_subscription(
            String, "/hide/callingout", self.callingout_trigger, 1
        )

    def robot_talker(self, robot_phrase='Welcome to human robot interaction', output_filename='robot_talker.mp3'):
        """Uses text to speech software to enable to robot to 
            alert users when they are in the intimate and public zones                                                    
        ----------
        robot_phrase : robot phrase
            String of text phrase 
        output_filename : name of file to store audio file
            String of outputfile name
        Returns
        -------
        None
        """
        # Language in which you want to convert
        language = 'en'
        
        # Passing the text and language to the engine, 
        # here we have marked slow=False. Which tells 
        # the module that the converted audio should 
        # have a high speed
        myobj = gTTS(text=robot_phrase, lang=language, slow=True)
        
        # Saving the converted audio in a mp3 file named
        # welcome 
        myobj.save(output_filename)

        # Play audio file with playsound library
        playsound.playsound(output_filename, True)

    def callingout_trigger(self, msg):
        msg = str(msg.data)
        calling_type, calling_name = msg.split(',')
        if calling_type == 'audio':
            self.robot_talker(f"{calling_name} is out")
        
        

def main():
    rclpy.init()
    countdown = CountDown()
    rclpy.spin(countdown)
    countdown.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
