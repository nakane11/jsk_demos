action_clients = []


class Main():
    def __init__():
        self.speech_msg = queue()

    def callback(self, speech_msg):
        self.speech_msg = msg

    def run(self):
        text = receive()
        while not rospy.is_shutdown():
            if len(self.queue) > 0:
                text = queue.pop()
                if text == "go":
                    go_client.send_goal()
                elif text == "speak":
                    speak_client.send_goal()
                elif text == "hand":
                    hand_client.send_goal()

            if check_hand():
                hogefunc()


            


def check_hand():
