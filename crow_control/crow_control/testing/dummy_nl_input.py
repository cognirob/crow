import os
import json
import rclpy
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from crow_msgs.msg import SentenceProgram

class DummyNlInput(Node):

    def __init__(self, node_name="dummy_nl_input"):
        super().__init__(node_name)
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        self.nl_publisher = self.create_publisher(SentenceProgram, "/nl_input", qos)

        self.bindings_com = {'1':'ukaž', '2':'seber', '3':'polož', '4':'podej', '5':'ukliď', '6':'pustit', '7':'úložiště', '8':'Odstraň poslední příkaz', '9':'Odstraň příkaz', ' ':' '}
        self.bindings_obj = {'q':'kostka', 'w':'kolo', 'e':'střecha', 'r':'lžíce', 't':'koule', 'y':'destička', 'u':'matka', 'i':'šroub', 'm':'klíč', 'n':'šroubovák', 'b':'kleště', 'v':'kladivo', 'c':'to', ' ':' '}
        self.bindings_col = {'a':'červená', 's':'vínová', 'd':'modrá', 'f':'zelená', 'g':'fialová', 'h':'zlatá', 'j':'bílá', ' ':' '}
        self.bindings_loc = {'z':'sklad', 'x':'stůl', ' ':' '}
        self.print_dict()

    def print_dict(self):
        print('=== Available key bindings ==')
        print('--Commands--')
        print(self.bindings_com)
        print('')
        print('--Objects--')
        print(self.bindings_obj)
        print('')
        print('--Colors--')
        print(self.bindings_col)
        print('')
        print('--Locations--')
        print(self.bindings_loc)
        print('')

    def publish_command(self, command, object, color, location):

        recog_text = command + ' ' + object  + ' ' + color + ' ' + location
        msg = SentenceProgram()
        msg.data.append(recog_text)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.nl_publisher.publish(msg)
        print('Publishing text:', msg.data)

def main():
    rclpy.init()
    time.sleep(1)
    dummy_nl = DummyNlInput()
    #rclpy.spin(dummy_nl)

    while True:
        key = input("Please press a key <1, ..., 9> to choose a command, or spacebar to skip (then press ENTER):\n")
        value_com = dummy_nl.bindings_com.get(key, None)
        if value_com:
            key = input("Please press a key <q, ..., i, c, ..., m> to choose a object, or spacebar to skip (then press ENTER):\n")
            value_obj = dummy_nl.bindings_obj.get(key, None)
            if value_obj:
                key = input("Please press a key <a, ..., j> to choose a color, or spacebar to skip (then press ENTER):\n")
                value_col = dummy_nl.bindings_col.get(key, None)
                if value_col:
                    key = input("Please press a key <z, ..., x> to choose a location, or spacebar to skip (then press ENTER):\n")
                    value_loc = dummy_nl.bindings_loc.get(key, None)
                    if value_loc:
                        dummy_nl.publish_command(value_com, value_obj, value_col, value_loc)
                    else:
                        print(f'Wrong location key')
                else:
                    print(f'Wrong color key')
            else:
                print(f'Wrong object key')
        else:
            print(f'Wrong command key')

    dummy_nl.destroy_node()

if __name__ == "__main__":
    main()